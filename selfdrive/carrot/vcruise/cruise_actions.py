import numpy as np
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car import ButtonType

class CruiseActions:
  def __init__(self):
    self.cancel_timer = 0
    self.activate_cruise = 0  # +1 enable, -1 off, -2 off&ready(원 코드 스타일)
    self.cruise_ready = False
    self.cruise_cancel_state = False
    self.v_cruise_kph_at_brake = 0

    self.paddle_decel_active = False
    self.pause_auto_speed_up = False  # road speed limit 삭제했지만 “가속 자동 재개” 억제 플래그는 남겨둠(원하면 제거 가능)

  def step_timers(self):
    self.cancel_timer = max(0, self.cancel_timer - 1)
    self.activate_cruise = 0

  def request(self, enable: int, cancel_timer_sec: float, reason: str,
              auto_cruise_control: int, auto_cruise_control_cancel_timer: int) -> str:
    """
    return: log 메시지(비워도 됨)
    """
    if self.cruise_cancel_state:
      return reason + " > Cancel state"

    if enable > 0 and self.cancel_timer > 0 and cancel_timer_sec >= 0:
      return reason + " > Canceled"

    if auto_cruise_control == 0 and enable != 0:
      self.soft_reset()
      return ""

    if auto_cruise_control_cancel_timer > 0 and enable != 0:
      self.soft_reset()
      return reason + " > timer Canceled"

    self.activate_cruise = enable
    self.cancel_timer = int(cancel_timer_sec / 0.01) if cancel_timer_sec >= 0 else 0
    return reason

  def soft_reset(self):
    self.activate_cruise = 0
    self.paddle_decel_active = False
    self.cruise_ready = False

  def apply_button_side_effects(self, button_type: int):
    # accel/decel 누르면 cancel state / timer reset (핵심만)
    if button_type in (ButtonType.accelCruise, ButtonType.decelCruise):
      self.paddle_decel_active = False
      self.cruise_cancel_state = False

  def compute_desired_set_speed(self, v_cruise_kph: float, v_ego_kph_set: int,
                                cruise_button_mode: int,
                                cruise_speed_min: int, cruise_speed_max: int,
                                cruise_speed_unit: int) -> float:
    """
    cruise speed table 제거 버전.
    - 기본: 최소 30 이상, 그 다음은 cruise_speed_unit 간격으로 올림
    """
    if v_cruise_kph < 30:
      v = 30
    else:
      v = ((int(v_cruise_kph) // cruise_speed_unit) + 1) * cruise_speed_unit
    return float(np.clip(v, cruise_speed_min, cruise_speed_max))
