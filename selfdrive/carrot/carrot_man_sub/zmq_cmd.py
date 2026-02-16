import json
import time
import traceback
import zmq

from cereal import log

NetworkType = log.DeviceState.NetworkType

class ZmqCmdServer:
  def __init__(self, params, sm, tmux_uploader, panda_debug_runner=None, bind_addr="tcp://*:7710"):
    self.params = params
    self.sm = sm
    self.tmux = tmux_uploader
    self.panda_debug = panda_debug_runner
    self.bind_addr = bind_addr

    self._thread = None

  def start(self):
    import threading
    self._thread = threading.Thread(target=self._run, daemon=True)
    self._thread.start()

  def _run(self):
    context = zmq.Context()

    def setup_socket():
      s = context.socket(zmq.REP)
      s.bind(self.bind_addr)
      p = zmq.Poller()
      p.register(s, zmq.POLLIN)
      return s, p

    sock, poller = setup_socket()

    isOnroadCount = 0
    is_tmux_sent = False

    print(f"[zmq] started: {self.bind_addr}")
    while True:
      try:
        socks = dict(poller.poll(100))
        if sock in socks and socks[sock] == zmq.POLLIN:
          msg = sock.recv(zmq.NOBLOCK)
          print(f"[zmq] request: {msg}")
          try:
            json_obj = json.loads(msg.decode())
          except Exception:
            json_obj = None
        else:
          json_obj = None

        # --------- background logic (no request) ----------
        if json_obj is None:
          isOnroadCount = isOnroadCount + 1 if self.params.get_bool("IsOnroad") else 0
          if isOnroadCount == 0:
            is_tmux_sent = False

          # onroad 첫 시작에 디버그 한번
          if isOnroadCount == 1 and self.panda_debug is not None:
            self.panda_debug.trigger()

          # 네트워크 연결 판단
          try:
            network_type = self.sm['deviceState'].networkType
            networkConnected = (network_type != NetworkType.none)
          except Exception:
            networkConnected = False

          # 500틱(= 20Hz라면 25초)쯤 스냅샷 생성
          if isOnroadCount == 500:
            self.tmux.make_tmux_data()

          # 이후 네트워크 되면 1회 업로드
          if isOnroadCount > 500 and (not is_tmux_sent) and networkConnected:
            self.tmux.send_tmux("Ekdrmsvkdlffjt7710", "onroad", send_settings=True)
            is_tmux_sent = True

          # 예외 플래그 있으면 즉시 업로드
          if self.params.get_bool("CarrotException") and networkConnected:
            self.params.put_bool("CarrotException", False)
            self.tmux.make_tmux_data()
            self.tmux.send_tmux("Ekdrmsvkdlffjt7710", "exception")

          continue

        # --------- request handlers ----------
        if 'echo_cmd' in json_obj:
          import subprocess
          try:
            result = subprocess.run(json_obj['echo_cmd'], shell=True, capture_output=True, text=False)
            rc = result.returncode
            try:
              stdout = result.stdout.decode('utf-8')
              stderr = result.stderr.decode('utf-8')
            except UnicodeDecodeError:
              stdout = result.stdout.decode('euc-kr', 'ignore')
              stderr = result.stderr.decode('euc-kr', 'ignore')

            echo = json.dumps({"echo_cmd": json_obj['echo_cmd'], "exitStatus": rc, "result": stdout, "error": stderr})
          except Exception as e:
            echo = json.dumps({"echo_cmd": json_obj.get('echo_cmd', ''), "exitStatus": -1, "result": "", "error": f"exception error: {str(e)}"})
          sock.send(echo.encode())

        elif 'tmux_send' in json_obj:
          self.tmux.make_tmux_data()
          self.tmux.send_tmux(json_obj['tmux_send'], "tmux_send")
          echo = json.dumps({"tmux_send": json_obj['tmux_send'], "result": "success"})
          sock.send(echo.encode())

        else:
          sock.send(json.dumps({"ok": False, "error": "unknown request"}).encode())

      except Exception as e:
        print(f"[zmq] error: {e}")
        traceback.print_exc()
        try:
          sock.close()
        except Exception:
          pass
        time.sleep(1)
        sock, poller = setup_socket()
