import subprocess
import time
import threading

class PandaDebugRunner:
  def __init__(self, cmd="/data/openpilot/selfdrive/debug/debug_console_carrot.py"):
    self.cmd = cmd
    self._flag = False
    self._thread = None

  def start(self):
    if self._thread is not None:
      return
    self._thread = threading.Thread(target=self._run, daemon=True)
    self._thread.start()

  def trigger(self):
    # 외부에서 "한 번 실행" 요청
    self._flag = True

  def _run(self):
    while True:
      if self._flag:
        self._flag = False
        try:
          subprocess.run(self.cmd, shell=True)
        except Exception as e:
          print(f"[panda_debug] error: {e}")
          time.sleep(2)
      else:
        time.sleep(1)
