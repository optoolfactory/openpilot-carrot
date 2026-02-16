import socket
import time
import traceback

class KisaReceiver:
  def __init__(self, carrot_serv, port: int = 12345):
    self.carrot_serv = carrot_serv
    self.port = port
    self._thread = None
    self._running = False

  def start(self):
    self._running = True
    import threading
    self._thread = threading.Thread(target=self._run, daemon=True)
    self._thread.start()

  def _parse(self, data: bytes):
    out = {}
    try:
      decoded = data.decode('utf-8')
    except UnicodeDecodeError:
      print("[kisa] decode error:", data)
      return out
    parts = decoded.split('/')
    for part in parts:
      if ':' in part:
        k, v = part.split(':', 1)
        try:
          out[k] = int(v)
        except ValueError:
          out[k] = v
    return out

  def _run(self):
    while self._running:
      try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
          sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
          sock.settimeout(10)
          sock.bind(('', self.port))
          print(f"[kisa] listening udp:{self.port}")

          while self._running:
            try:
              data, _ = sock.recvfrom(4096)
              if not data:
                continue
              kisa = self._parse(data)
              self.carrot_serv.update_kisa(kisa)
            except TimeoutError:
              time.sleep(1)
            except Exception as e:
              print("[kisa] error:", e)
              traceback.print_exc()
              break
        time.sleep(1)
      except Exception as e:
        print("[kisa] bind error, retry:", e)
        time.sleep(2)
