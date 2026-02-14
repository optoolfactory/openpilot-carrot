import json
import os
import subprocess
from datetime import datetime
from ftplib import FTP

class TmuxUploader:
  def __init__(self, params, params_memory):
    self.params = params
    self.params_memory = params_memory

  def save_toggle_values(self):
    try:
      import openpilot.selfdrive.frogpilot.fleetmanager.helpers as fleet
      toggle_values = fleet.get_all_toggle_values()
      file_path = os.path.join('/data', 'toggle_values.json')
      with open(file_path, 'w') as f:
        json.dump(toggle_values, f, indent=2)
    except Exception as e:
      print(f"[tmux] save_toggle_values error: {e}")

  def make_tmux_data(self):
    try:
      # 최근 1000라인을 /data/media/tmux.log로
      subprocess.run("rm -f /data/media/tmux.log; tmux capture-pane -pq -S-1000 > /data/media/tmux.log",
                     shell=True, capture_output=True, text=False)
      # 당신 원본처럼 apilot 스냅샷도 남김
      subprocess.run("/data/openpilot/selfdrive/apilot.py", shell=True, capture_output=True, text=False)
    except Exception as e:
      print(f"[tmux] make_tmux_data error: {e}")

  def send_tmux(self, ftp_password: str, tmux_why: str, send_settings: bool = False):
    ftp_server = "shind0.synology.me"
    ftp_port = 8021
    ftp_username = "carrotpilot"

    ftp = FTP()
    ftp.connect(ftp_server, ftp_port)
    ftp.login(ftp_username, ftp_password)

    car_selected = self.params.get("CarName")
    if car_selected is None:
      car_selected = "none"

    git_branch = self.params.get("GitBranch") or "unknown_branch"

    # branch dir
    try:
      ftp.mkd(git_branch)
    except Exception:
      pass
    ftp.cwd(git_branch)

    dongle = self.params.get("DongleId") or "unknown_dongle"
    directory = f"{car_selected} {dongle}"

    current_time = datetime.now().strftime("%Y%m%d-%H%M%S")
    filename = f"{tmux_why}-{current_time}-{git_branch}.txt"

    # device dir
    try:
      ftp.mkd(directory)
    except Exception:
      pass
    ftp.cwd(directory)

    # tmux log 업로드
    try:
      with open("/data/media/tmux.log", "rb") as f:
        ftp.storbinary(f"STOR {filename}", f)
    except Exception as e:
      print(f"[tmux] ftp sending error: {e}")

    # toggles 업로드
    if send_settings:
      self.save_toggle_values()
      try:
        with open("/data/toggle_values.json", "rb") as f:
          ftp.storbinary(f"STOR toggles-{current_time}.json", f)
      except Exception as e:
        print(f"[tmux] ftp params sending error: {e}")

    ftp.quit()
