import time
from openpilot.selfdrive.carrot.carrot_speed import CarrotSpeed

class SpeedService:
  def __init__(self, params, params_memory, sm, carrot_serv):
    self.params = params
    self.params_memory = params_memory
    self.sm = sm
    self.carrot_serv = carrot_serv

    self.carrot_speed = CarrotSpeed(neighbor_ring=3)

    self.v_cruise_last = 0
    self.long_active = False
    self.v_cruise_change = 0
    self._last_vt = 0.0
    self.gas_pressed_count = 0
    self.brake_pressed_count = 0
    self._last_viz_t = 0.0

    self.params_memory.put_int_nonblocking("CarrotSpeed", 0)

  def step(self, frame: int):
    # 원본 carrot_speed_serv 로직을 거의 그대로 유지 (핵심만)
    if not (self.sm.alive['carState'] and self.sm.alive['carControl']):
      self.v_cruise_change = 0
      return

    CS = self.sm['carState']
    CC = self.sm['carControl']

    v_ego = CS.vEgo
    a_ego = CS.aEgo
    v_ego_kph = v_ego * 3.6

    if CS.brakePressed:
      self.brake_pressed_count = 200

    gas_pressed = CS.gasPressed
    if gas_pressed:
      self.gas_pressed_count = 200
      self.v_cruise_change = 0
    else:
      # 기존 로직 유지
      if self.long_active and CC.longActive:
        if self.v_cruise_last < CS.vCruise:
          self.v_cruise_change = 100 if v_ego_kph < CS.vCruise else 0
        elif self.v_cruise_last > CS.vCruise:
          self.v_cruise_change = -100

        if self.v_cruise_change != 0:
          self.gas_pressed_count = 0
      else:
        self.v_cruise_change = 0

    self.long_active = CC.longActive
    self.v_cruise_last = CS.vCruise

    v_cruise_apply = max(min(CS.vCruise, v_ego_kph), 20)
    vt_last = self.params_memory.get_int("CarrotSpeed")
    if vt_last != 0:
      self.v_cruise_change = 0
    self.params_memory.put_int("CarrotSpeed", 0)

    now = time.monotonic()
    heading = self.carrot_serv.bearing
    lat = self.carrot_serv.vpPosPointLat
    lon = self.carrot_serv.vpPosPointLon

    viz_json, vt = self.carrot_speed.export_cells_around_with_here(lat, lon, heading, ring=4, max_points=64, lateral_m=6.0)

    if self.v_cruise_change != 0:
      self.carrot_speed.add_sample(lat, lon, heading, v_cruise_apply if self.v_cruise_change > 0 else (-v_cruise_apply))
      self.v_cruise_change += -1 if self.v_cruise_change > 0 else 1
    else:
      if self.brake_pressed_count > 0:
        pass
      elif self.gas_pressed_count > 0:
        vt = max(vt, v_cruise_apply)
        self.carrot_speed.add_sample(lat, lon, heading, vt)
      else:
        self.params_memory.put_int_nonblocking("CarrotSpeed", int(vt))

    self._last_vt = vt
    if gas_pressed and a_ego < -0.5:
      self.carrot_speed.invalidate_last_hit(window_s=2.0, action="clear")

    self.gas_pressed_count = max(0, self.gas_pressed_count - 1)
    self.brake_pressed_count = max(0, self.brake_pressed_count - 1)

    if now - self._last_viz_t > 0.5:
      self._last_viz_t = now
      self.params_memory.put_nonblocking("CarrotSpeedViz", viz_json)

    self.carrot_speed.maybe_save()
