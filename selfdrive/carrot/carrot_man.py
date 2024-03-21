import numpy as np
import time
import threading
import zmq
import os
import subprocess
import json
from datetime import datetime

from ftplib import FTP
from openpilot.common.realtime import Ratekeeper
from openpilot.common.params import Params
import cereal.messaging as messaging
from cereal import log
NetworkType = log.DeviceState.NetworkType

class CarrotMan:
  def __init__(self):
    self.params = Params()

    self.carrot_zmq_thread = threading.Thread(target=self.carrot_cmd_zmq, args=[])
    self.carrot_zmq_thread.daemon = True
    self.carrot_zmq_thread.start()


  def carrot_man_thread(self):
    context = zmq.Context()
    socket = context.socket(zmq.REP)
    socket.bind("tcp://*:7711")

    poller = zmq.Poller()
    poller.register(socket, zmq.POLLIN)

    isOnroadCount = 0
    is_tmux_sent = False
    sm = messaging.SubMaster(['deviceState'])

    while True:
      sm.update(0)

      socks = dict(poller.poll(100))

      if socket in socks and socks[socket] == zmq.POLLIN:
        message = socket.recv(zmq.NOBLOCK)
        data = json.loads(message)
        print(f"Received request: {message}")
        response = {
            "status": "ok",
            "data": "Hello from Python ZeroMQ server!"
        }
        socket.send(json.dumps(response).encode('utf-8'))
      else:
        isOnroadCount = isOnroadCount + 1 if self.params.get_bool("IsOnroad") else 0
        if isOnroadCount == 0:
          is_tmux_sent = False

        network_type = sm['deviceState'].networkType# if not force_wifi else NetworkType.wifi
        networkConnected = False if network_type == NetworkType.none else True

        if isOnroadCount == 500:
          self.make_tmux_data()
        if isOnroadCount > 500 and not is_tmux_sent and networkConnected:
          self.send_tmux("Ekdrmsvkdlffjt7710", "onroad", send_settings = True)
          is_tmux_sent = True
        if self.params.get_bool("CarrotException") and networkConnected:
          self.params.put_bool("CarrotException", False)
          self.make_tmux_data()
          self.send_tmux("Ekdrmsvkdlffjt7710", "exception")

  def make_tmux_data(self):
    try:
      result = subprocess.run("rm /data/media/tmux.log; tmux capture-pane -pq -S-1000 > /data/media/tmux.log", shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=False)
      result = subprocess.run("/data/openpilot/selfdrive/apilot.py", shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=False)
    except Exception as e:
      print("TMUX creation error")
      return

  def send_tmux(self, ftp_password, tmux_why, send_settings=False):

    ftp_server = "shind0.synology.me"
    ftp_port = 8021
    ftp_username = "carrotpilot"
    ftp = FTP()
    ftp.connect(ftp_server, ftp_port)
    ftp.login(ftp_username, ftp_password)
    car_selected = Params().get("CarName")
    if car_selected is None:
      car_selected = "none"
    else:
      car_selected = car_selected.decode('utf-8')

    directory = car_selected + " " + Params().get("DongleId").decode('utf-8')
    current_time = datetime.now().strftime("%Y%m%d-%H%M%S")
    filename = tmux_why + "-" + current_time + ".txt"

    try:
      ftp.mkd(directory)
    except Exception as e:
      print(f"Directory creation failed: {e}")
    ftp.cwd(directory)

    try:
      with open("/data/media/tmux.log", "rb") as file:
        ftp.storbinary(f'STOR {filename}', file)
    except Exception as e:
      print(f"ftp sending error...: {e}")

    if send_settings:
      #try:
      #    ftp.delete('settings.json')
      #except Exception as e:
      #    print(f"ftp settings.json delete error: {e}")
      try:
        with open("/data/backup_params.json", "rb") as file:
          ftp.storbinary(f'STOR settings-{current_time}.json', file)
      except Exception as e:
        print(f"ftp params sending error...: {e}")

    ftp.quit()

  def carrot_cmd_zmq(self):

    context = zmq.Context()
    socket = context.socket(zmq.REP)
    socket.bind("tcp://*:7710")

    while True:
      message = socket.recv()
      #print(f"Received request: {message}")
      json_obj = json.loads(message.decode())
      if 'echo_cmd' in json_obj:
        try:
          result = subprocess.run(json_obj['echo_cmd'], shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=False)
          try:
            stdout = result.stdout.decode('utf-8')
          except UnicodeDecodeError:
            stdout = result.stdout.decode('euc-kr', 'ignore')
                
          echo = json.dumps({"echo_cmd": json_obj['echo_cmd'], "result": stdout})
        except Exception as e:
          echo = json.dumps({"echo_cmd": json_obj['echo_cmd'], "result": f"exception error: {str(e)}"})
        #print(echo)
        socket.send(echo.encode())
      elif 'tmux_send' in json_obj:
        self.make_tmux_data()
        self.send_tmux(json_obj['tmux_send'], "tmux_send")
        echo = json.dumps({"tmux_send": json_obj['tmux_send'], "result": "success"})
        socket.send(echo.encode())

def main():
  print("CarrotManager Started")
  #print("Carrot GitBranch = {}, {}".format(Params().get("GitBranch"), Params().get("GitCommitDate")))
  carrot_man = CarrotMan()
  while True:
    try:
      carrot_man.carrot_man_thread()
    except Exception as e:
      print(f"carrot_man error...: {e}")
      time.sleep(10)


if __name__ == "__main__":
  main()
