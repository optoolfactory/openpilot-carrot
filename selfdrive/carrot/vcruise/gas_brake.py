from opendbc.car import GearShifter

class GasBrakeTracker:
  def __init__(self):
    self.gas_pressed_count = 0
    self.gas_pressed_count_last = 0
    self.gas_pressed_value = 0.0
    self.gas_tok_timer = int(0.4 / 0.01)  # 0.4 sec
    self.gas_tok = False

    self.brake_pressed_count = 0
    self.soft_hold_count = 0
    self.soft_hold_active = 0  # 0: off, 1: active, 2: latched(원 코드처럼 유지)

  def update(self, CS, enabled_last: bool, v_cruise_kph: float,
             auto_cruise_control: int, pcm_cruise: bool):
    # gas
    if CS.gasPressed:
      self.gas_pressed_count = max(1, self.gas_pressed_count + 1)
      self.gas_pressed_count_last = self.gas_pressed_count
      self.gas_pressed_value = max(CS.gas, self.gas_pressed_value) if self.gas_pressed_count > 1 else CS.gas
      self.gas_tok = False
      self.soft_hold_active = 0
    else:
      self.gas_tok = True if 0 < self.gas_pressed_count < self.gas_tok_timer else False
      self.gas_pressed_count = min(-1, self.gas_pressed_count - 1)
      if self.gas_pressed_count < -1:
        self.gas_pressed_count_last = 0
        self.gas_tok = False

    # brake
    if CS.brakePressed:
      self.brake_pressed_count = max(1, self.brake_pressed_count + 1)
      self.soft_hold_count = self.soft_hold_count + 1 if CS.vEgo < 0.1 and CS.gearShifter == GearShifter.drive else 0

      # autoCruiseControl==0 or pcmCruise면 soft-hold 불가
      if auto_cruise_control == 0 or pcm_cruise:
        self.soft_hold_active = 0
      else:
        self.soft_hold_active = 1 if self.soft_hold_count > 60 else 0
    else:
      self.soft_hold_count = 0
      self.brake_pressed_count = min(-1, self.brake_pressed_count - 1)
