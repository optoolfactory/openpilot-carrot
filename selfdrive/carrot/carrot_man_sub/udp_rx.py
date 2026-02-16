import json
import socket
import time
import traceback

class UdpReceiver:
  def __init__(self, carrot_serv, port: int):
    self.carrot_serv = carrot_serv
    self.port = port
    self.remote_addr = None
    self._thread = None
    self._running = False

  def start(self):
    self._running = True
    import threading
    self._thread = threading.Thread(target=self._run, daemon=True)
    self._thread.start()

  def _run(self):
    while self._running:
      try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
          sock.settimeout(10)
          sock.bind(('0.0.0.0', self.port))
          print(f"[udp_rx] listening udp:{self.port}")

          while self._running:
            try:
              data, addr = sock.recvfrom(4096)
              if not data:
                continue
              if self.remote_addr is None:
                print("[udp_rx] connected:", addr)
              self.remote_addr = addr
              try:
                obj = json.loads(data.decode())
                self.carrot_serv.update(obj)
              except Exception as e:
                print("[udp_rx] json error:", e)
                print(data)
            except TimeoutError:
              self.remote_addr = None
              time.sleep(1)
            except Exception as e:
              print("[udp_rx] error:", e)
              traceback.print_exc()
              self.remote_addr = None
              break
        time.sleep(1)
      except Exception as e:
        print("[udp_rx] bind error, retry:", e)
        time.sleep(2)
