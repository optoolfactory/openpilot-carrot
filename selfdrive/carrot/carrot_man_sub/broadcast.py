import fcntl
import json
import socket
import struct
import time
import traceback
import ssl
import urllib.request
import urllib.error

from openpilot.common.realtime import Ratekeeper
from openpilot.system.hardware import PC

from .curve_speed import CurveSpeedEstimator

class Broadcaster:
  def __init__(self, params, params_memory, sm, pm, carrot_serv, route_engine, speed_serv, udp_rx, gps_service, broadcast_port=7705):
    self.params = params
    self.params_memory = params_memory
    self.sm = sm
    self.pm = pm
    self.carrot_serv = carrot_serv
    self.route_engine = route_engine
    self.speed_serv = speed_serv
    self.udp_rx = udp_rx
    self.gps_service = gps_service

    self.broadcast_port = broadcast_port
    self.carrot_man_port = 7706

    self.broadcast_ip = None
    self.ip_address = "0.0.0.0"

    self.curve_speed = CurveSpeedEstimator(self.params)

    self._thread = None
    self._running = False

  def start(self):
    import threading
    self._running = True
    self._thread = threading.Thread(target=self._run, daemon=True)
    self._thread.start()

  def _get_broadcast_address(self):
    iface = b'br0' if PC else b'wlan0'
    try:
      with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        ip = fcntl.ioctl(s.fileno(), 0x8919, struct.pack('256s', iface))[20:24]
        return socket.inet_ntoa(ip)
    except Exception:
      return None

  def _get_local_ip(self):
    try:
      with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        s.connect(("8.8.8.8", 80))
        return s.getsockname()[0]
    except Exception as e:
      return f"Error: {e}"

  def _make_send_message(self, remote_addr):
    msg = {}
    msg['Carrot2'] = self.params.get("Version")
    isOnroad = self.params.get_bool("IsOnroad")
    msg['IsOnroad'] = isOnroad
    msg['CarrotRouteActive'] = self.route_engine.navi_points_active
    msg['ip'] = self.ip_address
    msg['port'] = self.carrot_man_port

    controls_active = False
    xState = 0
    trafficState = 0
    v_ego_kph = 0
    log_carrot = ""
    v_cruise_kph = 0
    carcruiseSpeed = 0

    if isOnroad:
      if self.sm.alive['carState']:
        cs = self.sm['carState']
        v_ego_kph = int(cs.vEgoCluster * 3.6 + 0.5)
        log_carrot = cs.logCarrot
        v_cruise_kph = cs.vCruise
        carcruiseSpeed = cs.cruiseState.speed * 3.6
      if self.sm.alive['selfdriveState']:
        controls_active = self.sm['selfdriveState'].active
      if self.sm.alive['longitudinalPlan']:
        lp = self.sm['longitudinalPlan']
        xState = lp.xState
        trafficState = lp.trafficState

    msg['log_carrot'] = log_carrot
    msg['v_cruise_kph'] = v_cruise_kph
    msg['carcruiseSpeed'] = carcruiseSpeed
    msg['v_ego_kph'] = v_ego_kph
    msg['tbt_dist'] = self.carrot_serv.xDistToTurn
    msg['sdi_dist'] = self.carrot_serv.xSpdDist
    msg['active'] = controls_active
    msg['xState'] = xState
    msg['trafficState'] = trafficState
    return json.dumps(msg)

  def _run(self):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

    rk = Ratekeeper(20, print_delay_threshold=None)
    frame = 0

    while self._running:
      try:
        self.sm.update(0)

        # navd route -> route_engine
        if self.sm.updated['navRouteNavd']:
          self.route_engine.update_from_navd(self.sm['navRouteNavd'].coordinates)

        # route speed
        coords, distances, route_speed = self.route_engine.carrot_navi_route()

        # vturn speed (원복)
        vturn_speed = self.curve_speed.calc(self.sm)

        # remote ip (udp_rx에서 공유)
        remote_addr = self.udp_rx.remote_addr
        remote_ip = remote_addr[0] if remote_addr else ""

        # update_navi (원본처럼 gps service name 전달)
        self.carrot_serv.update_navi(remote_ip, self.sm, self.pm, vturn_speed, coords, distances, route_speed, self.gps_service)

        # CarrotSpeed
        self.speed_serv.step(frame)

        if frame % 20 == 0 or remote_addr is not None:
          self.broadcast_ip = self._get_broadcast_address() if remote_addr is None else remote_addr[0]
          ip_address = self._get_local_ip()
          if ip_address != self.ip_address:
            self.ip_address = ip_address
          self.params_memory.put_nonblocking("NetworkAddress", self.ip_address)

          if self.broadcast_ip is not None:
            dat = self._make_send_message(remote_addr).encode('utf-8')
            sock.sendto(dat, (self.broadcast_ip, self.broadcast_port))

        rk.keep_time()
        frame += 1

      except Exception as e:
        print("[broadcast] error:", e)
        traceback.print_exc()
        time.sleep(1)
