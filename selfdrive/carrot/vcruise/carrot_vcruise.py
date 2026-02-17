import numpy as np
from typing import Optional

from openpilot.common.params import Params
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car import structs

from .params_mgr import ParamsManager
from .button_handler import ButtonHandler
from .gas_brake import GasBrakeTracker
from .cruise_actions import CruiseActions
from .lead_logic import LeadSafetyStop, LeadState
from .personality import PersonalityController
from .vcruise_output import VCruiseOutput


V_CRUISE_UNSET = 255  # 필요하면 openpilot constants로 교체하세요
GearShifter = structs.CarState.GearShifter
ButtonType = structs.CarState.ButtonEvent.Type


class VCruiseCarrot:
  def __init__(self, CP):
    self.CP = CP
    self.frame = 0

    self.params_memory = Params("/dev/shm/params")

    self.v_cruise_kph = 20
    self.v_cruise_cluster_kph = 20
    self.v_cruise_kph_last = 20

    self.enabled_last = False
    self.is_metric = True

    # core inputs
    self.v_ego_kph_set = 0
    self.v_lead_kph = 0.0

    # logging
    self._log_timer = 0
    self._log_timeout = int(3 / 0.01)
    self.log = ""

    # autoCruiseControl cancel timer(기어 D 아닐 때 유지하던 것)
    self.autoCruiseControl_cancel_timer = 0

    # modules
    self.pm = ParamsManager()
    self.buttons = ButtonHandler()
    self.gb = GasBrakeTracker()
    self.act = CruiseActions()
    self.lead = LeadState()
    self.lead_safety = LeadSafetyStop(decel_rate=1.5)
    self.personality = PersonalityController()
    self.output = VCruiseOutput()

    # cached config
    self.cfg = self.pm.read(is_metric=True)

  @property
  def v_cruise_initialized(self):
    return self.v_cruise_kph != V_CRUISE_UNSET

  def _add_log(self, msg: str):
    if not msg:
      self._log_timer = max(0, self._log_timer - 1)
      if self._log_timer <= 0:
        self.log = ""
    else:
      self.log = msg
      self._log_timer = self._log_timeout

  def _update_lead(self, sm):
    if sm.alive['radarState']:
      lead = sm['radarState'].leadOne
      if lead.status:
        self.lead.d_rel = float(lead.dRel)
        self.lead.v_rel = float(lead.vRel)
        self.v_lead_kph = float(lead.vLeadK * CV.MS_TO_KPH)
      else:
        self.lead.d_rel = 0.0
        self.lead.v_rel = 0.0
        self.v_lead_kph = 0.0
    else:
      self.lead.d_rel = 0.0
      self.lead.v_rel = 0.0
      self.v_lead_kph = 0.0

  def update_v_cruise(self, CS, sm, is_metric: bool):
    self._add_log("")
    self.frame += 1
    self.is_metric = is_metric

    # 주기적으로 params 갱신
    if self.frame % 10 == 0:
      self.cfg = self.pm.read(is_metric)

    # gear 체크로 cancel timer 유지
    if CS.gearShifter != GearShifter.drive:
      self.autoCruiseControl_cancel_timer = 20 * 100  # 20 sec
    else:
      self.autoCruiseControl_cancel_timer = max(0, self.autoCruiseControl_cancel_timer - 1)

    CC = sm['carControl']

    self._update_lead(sm)

    self.v_cruise_kph_last = self.v_cruise_kph
    self.act.step_timers()

    self.v_ego_kph_set = int(CS.vEgoCluster * CV.MS_TO_KPH + 0.5)

    # gas/brake tracker
    self.gb.update(CS, self.enabled_last, self.v_cruise_kph, self.cfg.auto_cruise_control, self.CP.pcmCruise)

    # brake 첫 눌림 시 set speed 기억(원 코드 핵심)
    if CS.brakePressed and self.gb.brake_pressed_count == 1 and self.enabled_last:
      self.act.v_cruise_kph_at_brake = self.v_cruise_kph
      self._add_log(f"{self.v_cruise_kph} Cruise speed at brake")

    # 버튼 처리
    button_kph, button_type, long_pressed = self.buttons.process(CS.buttonEvents, self.v_cruise_kph, self.is_metric, self.cfg)

    self.act.apply_button_side_effects(button_type)

    v_cruise_kph = self.v_cruise_kph

    if button_type == ButtonType.cancel:
      self.act.cruise_cancel_state = True

    if not long_pressed:
      if button_type == ButtonType.accelCruise:
        self.act.pause_auto_speed_up = False
        if self.act.cruise_ready or (not CC.enabled) or CS.cruiseState.standstill:
          # enable 성격
          msg = self.act.request(1, -1, "Cruise on (button)", self.cfg.auto_cruise_control, self.autoCruiseControl_cancel_timer)
          if msg:
            self._add_log(msg)
        elif self.act.v_cruise_kph_at_brake > 0 and v_cruise_kph < self.act.v_cruise_kph_at_brake:
          v_cruise_kph = self.act.v_cruise_kph_at_brake
          self.act.v_cruise_kph_at_brake = 0
        elif self.cfg.cruise_button_mode == 0:
          v_cruise_kph = button_kph
        else:
          v_cruise_kph = self.act.compute_desired_set_speed(
            v_cruise_kph, self.v_ego_kph_set,
            self.cfg.cruise_button_mode,
            self.cfg.cruise_speed_min, self.cfg.cruise_speed_max,
            self.cfg.cruise_speed_unit
          )

      elif button_type == ButtonType.decelCruise:
        self.act.pause_auto_speed_up = True
        if self.gb.soft_hold_active > 0:
          msg = self.act.request(-1, -1, "Cruise off, softhold (decel)", self.cfg.auto_cruise_control, self.autoCruiseControl_cancel_timer)
          if msg:
            self._add_log(msg)
        elif self.act.cruise_ready:
          self.act.paddle_decel_active = True
        elif not CC.enabled:
          v_cruise_kph = max(self.v_ego_kph_set, self.cfg.cruise_speed_min)
        else:
          # 감속 버튼: 단순히 버튼 계산값 적용 or 현재속도에 맞춤
          if self.v_ego_kph_set > self.cfg.cruise_speed_min and v_cruise_kph > self.v_ego_kph_set:
            v_cruise_kph = self.v_ego_kph_set
          elif self.cfg.cruise_button_mode in (0, 1):
            v_cruise_kph = button_kph

        self.act.v_cruise_kph_at_brake = 0

      elif button_type == ButtonType.gapAdjustCruise:
        new_p = self.personality.cycle(CS.pcmCruiseGap, self.cfg)
        self._add_log(f"Personality -> {new_p}")
    else:
      # long pressed: accel/decel만 속도 점프 허용
      if button_type == ButtonType.accelCruise:
        v_cruise_kph = button_kph
        self.act.v_cruise_kph_at_brake = 0
      elif button_type == ButtonType.decelCruise:
        self.act.pause_auto_speed_up = True
        v_cruise_kph = button_kph
        self.act.v_cruise_kph_at_brake = 0

    # paddle mode(원 코드 핵심만)
    if self.cfg.paddle_mode > 0 and button_type in (ButtonType.paddleLeft, ButtonType.paddleRight):
      # paddle_mode==3 같은 동작을 유지하고 싶으면 여기서 분기
      msg = self.act.request(-2, -1, "Cruise off & Ready (paddle)", self.cfg.auto_cruise_control, self.autoCruiseControl_cancel_timer)
      if msg:
        self._add_log(msg)
      if self.cfg.paddle_mode == 2:
        self.act.paddle_decel_active = True

    elif self.act.paddle_decel_active:
      if not CC.enabled:
        msg = self.act.request(1, -1, "Cruise on (paddle decel)", self.cfg.auto_cruise_control, self.autoCruiseControl_cancel_timer)
        if msg:
          self._add_log(msg)

    # --- lead safety stop auto engage (optional core) ---
    if (not CC.enabled) and (self.lead.d_rel > 0.0) and (CS.vEgo > 0.02):
      safe_ok, safe_dist = self.lead_safety.check_safe_stop(CS.vEgo, self.lead, safe_distance=4.0)
      if not safe_ok:
        msg = self.act.request(1, -1, "Cruise on (fcw)", self.cfg.auto_cruise_control, self.autoCruiseControl_cancel_timer)
        if msg:
          self._add_log(f"{msg} d_final={safe_dist:.1f}")

    # enable/cancel 결과 반영(PCM cruise 여부에 따라 v_cruise 선택)
    if CS.cruiseState.available:
      if not self.CP.pcmCruise:
        self.v_cruise_kph = float(np.clip(v_cruise_kph, self.cfg.cruise_speed_min, self.cfg.cruise_speed_max))
        self.v_cruise_cluster_kph = self.v_cruise_kph
      else:
        if self.cfg.speed_from_pcm == 1:
          self.v_cruise_kph = CS.cruiseState.speed * CV.MS_TO_KPH
          self.v_cruise_cluster_kph = CS.cruiseState.speedCluster * CV.MS_TO_KPH
        else:
          self.v_cruise_kph = float(np.clip(v_cruise_kph, 30, self.cfg.cruise_speed_max))
          self.v_cruise_cluster_kph = self.v_cruise_kph
    else:
      self.v_cruise_kph = float(np.clip(v_cruise_kph, self.cfg.cruise_speed_min, self.cfg.cruise_speed_max))
      self.v_cruise_cluster_kph = self.v_cruise_kph

    # cruise_ready 상태 업데이트(원 코드 느낌만 유지)
    if self.act.activate_cruise > 0:
      self.act.cruise_ready = False
    elif self.act.activate_cruise < 0:
      self.act.cruise_ready = True if self.act.activate_cruise == -2 else False

    self.enabled_last = CC.enabled
    
    self.output.soft_hold_active = self.gb.soft_hold_active
    self.output.activate_cruise = self.act.activate_cruise
    self.output.carrot_cruise = 1 if self.act.paddle_decel_active else 0

    self.output.paddle_decel_active = self.act.paddle_decel_active
    self.output.v_cruise_kph = self.v_cruise_kph
    self.output.v_cruise_cluster_kph = self.v_cruise_cluster_kph
    
    return self.output
