import math

from cereal import car
from openpilotcommon.params import Params
from opendbc.car.common.conversions import Conversions as CV

ButtonType = car.CarState.ButtonEvent.Type

class LogBuffer:
  __slots__ = ("_timer", "_timeout", "text")
  def __init__(self, dt=0.01, hold_sec=3.0):
    self._timeout = int(hold_sec / dt)
    self._timer = 0
    self.text = ""

  def tick(self):
    if self._timer > 0:
      self._timer -= 1
      if self._timer <= 0:
        self.text = ""

  def set(self, msg: str):
    self.text = msg
    self._timer = self._timeout
  

class ButtonInterpreter:
  __slots__ = ("long_pressed", "button_cnt", "button_prev", "button_long_time")

  def __init__(self):
    self.long_pressed = False
    self.button_cnt = 0
    self.button_prev = ButtonType.unknown
    self.button_long_time = 40

  def interpret(self, inp, pc, v_cruise_kph: float, is_metric: bool):
    buttonEvents = inp.buttonEvents

    button_kph = v_cruise_kph
    button_type = 0

    SPEED_UP_UNIT = pc.cruise_speed_unit_basic
    SPEED_DOWN_UNIT = pc.cruise_speed_unit if pc.cruise_button_mode in (1, 2, 3) else pc.cruise_speed_unit_basic
    V_CRUISE_DELTA = 10

    if self.button_cnt > 0:
      self.button_cnt += 1

    for b in buttonEvents:
      bt = b.type

      if bt in (ButtonType.paddleLeft, ButtonType.paddleRight) and b.pressed:
        button_type = bt
        self.long_pressed = False
        self.button_cnt = 0
        continue

      if b.pressed and self.button_cnt == 0 and bt in (
        ButtonType.accelCruise, ButtonType.decelCruise,
        ButtonType.gapAdjustCruise, ButtonType.cancel,
        ButtonType.lfaButton
      ):
        self.button_cnt = 1
        self.button_prev = bt
        self.button_long_time = 40 if bt in (ButtonType.accelCruise, ButtonType.decelCruise) else 70

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
        if self.button_cnt < self.button_long_time + 2:
          button_type = bt

    return button_kph, button_type, self.long_pressed
