import math
from typing import Tuple

from opendbc.car import DT_CTRL
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car import ButtonType

class ButtonHandler:
  def __init__(self):
    self.long_pressed = False
    self.button_cnt = 0
    self.button_prev = ButtonType.unknown
    self.button_long_time = 40

  def _tick(self):
    if self.button_cnt > 0:
      self.button_cnt += 1

  def process(self, button_events, v_cruise_kph: float, is_metric: bool, cfg) -> Tuple[float, int, bool]:
    self._tick()

    button_kph = v_cruise_kph
    button_type = 0

    # button mode
    # 0: up/down: cruise_speed_unit_basic, long up/down: speed unit
    SPEED_UP_UNIT = cfg.cruise_speed_unit_basic
    SPEED_DOWN_UNIT = cfg.cruise_speed_unit if cfg.cruise_button_mode in [1, 2, 3] else cfg.cruise_speed_unit_basic

    V_CRUISE_DELTA = cfg.cruise_speed_unit # 10

    for b in button_events:
      bt = b.type

      # Paddle: press 순간 즉시 이벤트
      if bt in (ButtonType.paddleLeft, ButtonType.paddleRight) and b.pressed:
        button_type = bt
        self.long_pressed = False
        self.button_cnt = 0
        continue

      # press 시작
      if b.pressed and self.button_cnt == 0 and bt in (
        ButtonType.accelCruise, ButtonType.decelCruise,
        ButtonType.gapAdjustCruise, ButtonType.cancel,
      ):
        self.button_cnt = 1
        self.button_prev = bt
        self.button_long_time = 40 if bt in (ButtonType.accelCruise, ButtonType.decelCruise) else 70

      # release
      elif (not b.pressed) and self.button_cnt > 0 and bt == self.button_prev:
        if bt == ButtonType.cancel:
          button_type = bt
        elif not self.long_pressed:
          if bt == ButtonType.accelCruise:
            unit = SPEED_UP_UNIT if is_metric else SPEED_UP_UNIT * CV.MPH_TO_KPH
            button_kph = math.ceil((button_kph + 0.01) / unit) * unit
          elif bt == ButtonType.decelCruise:
            unit = SPEED_DOWN_UNIT if is_metric else SPEED_DOWN_UNIT * CV.MPH_TO_KPH
            button_kph = math.floor((button_kph - 0.01) / unit) * unit
          button_type = bt

        self.long_pressed = False
        self.button_cnt = 0

    # long press
    if self.button_cnt > self.button_long_time:
      self.long_pressed = True
      bt = self.button_prev

      if bt in (ButtonType.accelCruise, ButtonType.decelCruise):
        mod = button_kph % V_CRUISE_DELTA
        if bt == ButtonType.accelCruise:
          button_kph += V_CRUISE_DELTA - mod
        else:
          button_kph -= V_CRUISE_DELTA - (-mod % V_CRUISE_DELTA)
        button_type = bt
        self.button_cnt %= self.button_long_time

      else:
        # gap/cancel long press는 “1회 이벤트”만 주도록 유지
        if self.button_cnt < self.button_long_time + 2:
          button_type = bt

    return button_kph, button_type, self.long_pressed
