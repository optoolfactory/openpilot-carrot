import socket
import struct
import time
import traceback

import cereal.messaging as messaging
from openpilot.selfdrive.navd.helpers import Coordinate

try:
  from shapely.geometry import LineString
  SHAPELY_AVAILABLE = True
except ImportError:
  SHAPELY_AVAILABLE = False

from .geo import get_path_after_distance, gps_to_relative_xy, calculate_curvature, curvature_to_speed

class RouteEngine:
  def __init__(self, params, params_memory, sm, pm, carrot_serv, gps_service):
    self.params = params
    self.params_memory = params_memory
    self.sm = sm
    self.pm = pm
    self.carrot_serv = carrot_serv
    self.gps_service = gps_service

    self.navi_points = []
    self.navi_points_start_index = 0
    self.navi_points_active = False
    self.navd_active = False
    self.active_carrot_last = False

    self._thread = None

  def start(self):
    import threading
    self._thread = threading.Thread(target=self._tcp_route_server, daemon=True)
    self._thread.start()

  def update_from_navd(self, coords):
    if len(coords) > 0:
      self.navi_points = [(c.longitude, c.latitude) for c in coords]
      self.navi_points_start_index = 0
      self.navi_points_active = True
      self.navd_active = True
      print("[route] received from navd:", len(self.navi_points))
    else:
      self.navd_active = False

    msg = messaging.new_message('navRoute', valid=True)
    msg.navRoute.coordinates = coords
    self.pm.send('navRoute', msg)

  def send_routes(self, coords_dicts):
    msg = messaging.new_message('navRoute', valid=True)
    msg.navRoute.coordinates = coords_dicts
    self.pm.send('navRoute', msg)

  def carrot_navi_route(self):
    # 원본 로직 유지: 조건/활성 체크
    if not self.navi_points_active or not SHAPELY_AVAILABLE or (self.carrot_serv.active_carrot <= 1 and not self.navd_active):
      if self.navi_points_active:
        self.navi_points = []
        self.navi_points_active = False
      self.active_carrot_last = self.carrot_serv.active_carrot
      return [], [], 300

    cur_pos = (self.carrot_serv.vpPosPointLon, self.carrot_serv.vpPosPointLat)
    heading = self.carrot_serv.bearing

    distance_interval = 10.0
    out_speed = 300

    path, self.navi_points_start_index, start_point = get_path_after_distance(
      self.navi_points_start_index, self.navi_points, cur_pos, 300
    )

    if not path:
      return [], [], 300

    rel = gps_to_relative_xy(path, start_point, heading)
    line = LineString(rel)
    resampled_points = []
    resampled_distances = []
    d = 0.0
    while d <= line.length:
      p = line.interpolate(d)
      resampled_points.append((p.x, p.y))
      resampled_distances.append(d)
      d += distance_interval

    sample = 4
    if len(resampled_points) < sample*2 + 1:
      return resampled_points, resampled_distances, 300

    speeds = []
    distance = 10.0
    for i in range(len(resampled_points) - sample*2):
      distance += distance_interval
      p1 = resampled_points[i]
      p2 = resampled_points[i + sample]
      p3 = resampled_points[i + sample*2]
      curv = calculate_curvature(p1, p2, p3)
      speed = curvature_to_speed(curv, self.carrot_serv.nRoadLimitSpeed)
      speeds.append(speed)

    # 원본: 역방향 감속 제한
    accel_limit = self.carrot_serv.autoNaviSpeedDecelRate
    accel_limit_kmh = accel_limit * 3.6
    out_speeds = [0.0] * len(speeds)
    out_speeds[-1] = speeds[-1]

    v_ego_kph = self.sm['carState'].vEgo * 3.6
    time_delay = self.carrot_serv.autoNaviSpeedCtrlEnd
    time_wait = 0.0

    for i in range(len(speeds) - 2, -1, -1):
      target = speeds[i]
      next_v = out_speeds[i + 1]
      if target < next_v:
        time_delay = max(0.0, ((v_ego_kph - target) / accel_limit_kmh))
        time_wait = -time_delay

      time_interval = distance_interval / (next_v / 3.6) if next_v > 0 else 0.0
      time_apply = min(time_interval, max(0.0, time_interval + time_wait))
      max_allowed = next_v + (accel_limit_kmh * time_apply)
      out_speeds[i] = min(target, max_allowed)

      time_wait += min(2.0, time_interval)

    out_speed = out_speeds[0] if out_speeds else 300
    return resampled_points, resampled_distances, out_speed

  # ---------- TCP route server (7709) ----------
  def _recvall(self, sock, n):
    data = bytearray()
    while len(data) < n:
      pkt = sock.recv(n - len(data))
      if not pkt:
        return None
      data.extend(pkt)
    return data

  def _tcp_route_server(self):
    host = '0.0.0.0'
    port = 7709
    while True:
      try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
          s.bind((host, port))
          s.listen()
          print("[route] tcp server listening:", port)

          while True:
            print("[route] waiting connection...")
            conn, addr = s.accept()
            with conn:
              print("[route] connected:", addr)
              size_bytes = self._recvall(conn, 4)
              if not size_bytes:
                continue
              total_size = struct.unpack('!I', size_bytes)[0]
              all_data = self._recvall(conn, total_size)
              if all_data is None:
                continue

              self.navi_points = []
              points = []
              for i in range(0, len(all_data), 8):
                x, y = struct.unpack('!ff', all_data[i:i+8])
                self.navi_points.append((x, y))
                coord = Coordinate.from_mapbox_tuple((x, y))
                points.append(coord)

              coords = [c.as_dict() for c in points]
              self.navi_points_start_index = 0
              self.navi_points_active = True
              print("[route] received points:", len(self.navi_points))

              self.send_routes(coords)

              if len(coords):
                dest = coords[-1]
                dest['place_name'] = "External Navi"
                self.params.put("NavDestination", __import__("json").dumps(dest))
      except Exception as e:
        print("[route] server error:", e)
        traceback.print_exc()
        time.sleep(2)
