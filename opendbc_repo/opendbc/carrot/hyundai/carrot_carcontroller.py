from pickle import NONE
from types import NoneType
import numpy as np
from opendbc.car import DT_CTRL, structs
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.hyundai.values import HyundaiFlags
from opendbc.carrot.hyundai import carrot_hyundaicanfd
from openpilot.common.params import Params

VisualAlert = structs.CarControl.HUDControl.VisualAlert
LongCtrlState = structs.CarControl.Actuators.LongControlState

def rate_limit(x, x_last, lo, hi):
  return float(np.clip(x, x_last + lo, x_last + hi))

def process_hud_alert(enabled, fingerprint, hud_control):
  sys_warning = (hud_control.visualAlert in (VisualAlert.steerRequired, VisualAlert.ldw))

  # initialize to no line visible
  # TODO: this is not accurate for all cars
  sys_state = 1
  if hud_control.leftLaneVisible and hud_control.rightLaneVisible or sys_warning:  # HUD alert only display when LKAS status is active
    sys_state = 3 if enabled or sys_warning else 4
  elif hud_control.leftLaneVisible:
    sys_state = 5
  elif hud_control.rightLaneVisible:
    sys_state = 6

  # initialize to no warnings
  left_lane_warning = 0
  right_lane_warning = 0
  if hud_control.leftLaneDepart:
    left_lane_warning = 1 if fingerprint in (CAR.GENESIS_G90, CAR.GENESIS_G80) else 2
  if hud_control.rightLaneDepart:
    right_lane_warning = 1 if fingerprint in (CAR.GENESIS_G90, CAR.GENESIS_G80) else 2

  return sys_warning, sys_state, left_lane_warning, right_lane_warning

def apply_steer_angle_limits_physics(desired_sw_deg: float,
                                     last_sw_deg: float,
                                     v_ego: float,
                                     steering_sw_deg: float,
                                     lat_active: bool,
                                     wheelbase_m: float,
                                     steer_ratio: float,
                                     steer_sw_max_deg: float) -> float:
  max_lat_accel = 5.0   # m/s^2
  max_lat_jerk  = 4.0   # m/s^3
  max_sw_rate_deg_per_tick = 2.0   # EPS 보호용 상한

  v = max(float(v_ego), 1.0)

  target_sw = float(np.clip(desired_sw_deg, -steer_sw_max_deg, steer_sw_max_deg))

  target_rw = target_sw / steer_ratio
  last_rw   = float(last_sw_deg) / steer_ratio

  # --- accel limit ---
  rw_max_rad = np.arctan((max_lat_accel * wheelbase_m) / (v * v))
  rw_max = float(np.degrees(rw_max_rad))

  # --- jerk -> rate limit ---
  sec2 = 1.2
  max_drw_dt = (max_lat_jerk * wheelbase_m) / (v * v * sec2)     # rad/s
  max_drw_per_tick = max_drw_dt * DT_CTRL                        # rad/tick
  max_drw_per_tick_deg = float(np.degrees(max_drw_per_tick))

  max_drw_per_tick_deg = min(
    max_drw_per_tick_deg,
    max_sw_rate_deg_per_tick / steer_ratio
  )
  err = abs(target_sw - last_sw_deg)
  if err > 20.0:
    max_drw_per_tick_deg *= 0.5
  
  # --- rate limit ---
  cmd_rw = rate_limit(target_rw, last_rw, -max_drw_per_tick_deg, max_drw_per_tick_deg)

  # --- accel clip ---
  cmd_rw = float(np.clip(cmd_rw, -rw_max, rw_max))

  if not lat_active:
    cmd_rw = float(steering_sw_deg) / steer_ratio

  cmd_sw = cmd_rw * steer_ratio
  return float(np.clip(cmd_sw, -steer_sw_max_deg, steer_sw_max_deg))

class CarrotCarController(CarControllerBase):
  def _carrot_init(self, CP):
    self.angle_control = CP.flags & HyundaiFlags.ANGLE_CONTROL
    self.apply_angle_last = 0
    self.lkas_max_torque = 0
    self.angle_max_torque = self.params.ANGLE_MAX_TORQUE
    self.hyundai_jerk = HyundaiJerk()


  def _carrot_process_steering_angle(self, CS, CC):
    apply_angle = apply_steer_angle_limits_physics(
      CC.actuators.steeringAngleDeg,
      self.apply_angle_last,
      CS.out.vEgoRaw,
      CS.out.steeringAngleDeg,
      CC.latActive,
      self.CP.wheelbase,
      self.CP.steerRatio,
      self.params.ANGLE_LIMITS.STEER_ANGLE_MAX
    )
    
    if CS.out.steeringPressed:
      self.lkas_max_torque = max(self.lkas_max_torque - 20, self.params.ANGLE_MIN_TORQUE)
    else:
      target_torque = self.angle_max_torque

      max_steering_tq = self.params.STEER_DRIVER_ALLOWANCE * 0.7
      rate_ratio = max(20, max_steering_tq - abs(CS.out.steeringTorque)) / max_steering_tq
      rate_up = self.params.ANGLE_TORQUE_UP_RATE * rate_ratio
      rate_down = self.params.ANGLE_TORQUE_DOWN_RATE * rate_ratio

      if self.lkas_max_torque > target_torque:
        self.lkas_max_torque = max(self.lkas_max_torque - rate_down, target_torque)
      else:
        self.lkas_max_torque = min(self.lkas_max_torque + rate_up, target_torque)


    if not CC.latActive:
      self.lkas_max_torque = 0

    self.apply_angle_last = apply_angle
  
  def _carrot_canfd_camera_scc_msg(self, apply_steer_req, apply_torque, accel, stopping, hud_control, CS, CC, set_speed_in_units, actuators):

    sys_warning, sys_state, left_lane_warning, right_lane_warning = process_hud_alert(CC.enabled, self.car_fingerprint,
                                                                                     hud_control)
    can_sends = []
    self._carrot_process_steering_angle(CS, CC)
    if self.angle_control:
      apply_steer_req = CC.latActive
    can_sends.extend(carrot_hyundaicanfd.create_steering_messages_camera_scc(self.frame, self.packer, self.CAN, CC, apply_steer_req, apply_torque, CS, self.apply_angle_last, self.lkas_max_torque, self.angle_control))
    if self.frame % 5 == 0:
      can_sends.extend(carrot_hyundaicanfd.create_lfahda_cluster(self.packer, CS, self.CAN, CC.longActive, CC.latActive))

    #if self.camera_scc_params in [2, 3]:
    #  self.canfd_toggle_adas(CC, CS)
    if self.CP.openpilotLongitudinalControl:
      self.hyundai_jerk.make_jerk(self.CP, CS, accel, actuators, hud_control)
      self.hyundai_jerk.check_carrot_cruise(CC, CS, hud_control, stopping, accel, actuators.aTarget)

      can_sends.extend(carrot_hyundaicanfd.create_ccnc_messages(self.CP, self.packer, self.CAN, self.frame, CC, CS, hud_control, self.apply_angle_last, left_lane_warning, right_lane_warning))
      if self.frame % 2 == 0:
        msg = carrot_hyundaicanfd.create_acc_control_scc2(self.packer, self.CAN, CC.enabled, self.accel_last, accel, stopping, CC.cruiseControl.override,
                                                              set_speed_in_units, hud_control, self.hyundai_jerk, CS)
        if msg is not None:
          can_sends.append(msg)
        can_sends.extend(carrot_hyundaicanfd.create_tcs_messages(self.packer, self.CAN, CS)) # for sorento SCC radar...
        self.accel_last = accel
    else:
      # button presses
      """
      if self.camera_scc_params == 3: # camera scc but stock long
        send_button = self.make_spam_button(CC, CS)
        can_sends.extend(hyundaicanfd.forward_button_message(self.packer, self.CAN, self.frame, CS, send_button, self.MainMode_ACC_trigger, self.LFA_trigger))
      else:
        can_sends.extend(self.create_button_messages(CC, CS, use_clu11=False))
      """
      pass
    return can_sends


class HyundaiJerk:
  def __init__(self):
    self.params = Params()
    self.jerk = 0.0
    self.jerk_u = self.jerk_l = 0.0
    self.cb_upper = self.cb_lower = 0.0
    self.jerk_u_min = 0.5
    self.carrot_cruise = 1
    self.carrot_cruise_accel = 0.0

  def check_carrot_cruise(self, CC, CS, hud_control, stopping, accel, a_target):
    carrot_cruise_decel = 0 #self.params.get_float("CarrotCruiseDecel")
    #carrot_cruise_atc_decel = self.params.get_float("CarrotCruiseAtcDecel")
    #if carrot_cruise_atc_decel >= 0 and 0 < hud_control.atcDistance < 500:
    #  carrot_cruise_decel = max(carrot_cruise_decel, carrot_cruise_atc_decel)
    self.carrot_cruise = 0
    if CS.out.carrotCruise > 0 and not CC.cruiseControl.override:
      if CS.softHoldActive == 0 and not stopping:
        if CS.out.vEgo > 10/3.6:
          if carrot_cruise_decel < 0:
            if (a_target > -0.1 or accel > -0.1):
              self.carrot_cruise = 1
              self.carrot_cruise_accel = 0.0
          else:
            self.carrot_cruise = 2
            carrot_cruise = min(accel, -carrot_cruise_decel * 0.01)
            self.carrot_cruise_accel = max(carrot_cruise, self.carrot_cruise_accel - 1.0 * DT_CTRL) #  점진적으로 줄임.
    if self.carrot_cruise == 0:
      self.carrot_cruise_accel = CS.out.aEgo
    
  def make_jerk(self, CP, CS, accel, actuators, hud_control):
    if actuators.longControlState == LongCtrlState.stopping:
      self.jerk = self.jerk_u_min / 2 - CS.out.aEgo
    else:
      jerk = actuators.jerk if actuators.longControlState == LongCtrlState.pid else 0.0
      #a_error = actuators.aTarget - CS.out.aEgo
      self.jerk = jerk# + a_error

    jerk_max_l = 5.0
    jerk_max_u = jerk_max_l
    if actuators.longControlState == LongCtrlState.off:
      self.jerk_u = jerk_max_u
      self.jerk_l = jerk_max_l
      self.cb_upper = self.cb_lower = 0.0
    else:
      if CP.flags & HyundaiFlags.CANFD:
        self.jerk_u = min(max(self.jerk_u_min, self.jerk * 2.0), jerk_max_u)
        self.jerk_l = min(max(1.0, -self.jerk * 4.0), jerk_max_l)
        self.cb_upper = self.cb_lower = 0.0
      else:
        self.jerk_u = min(max(self.jerk_u_min, self.jerk * 2.0), jerk_max_u)
        self.jerk_l = min(max(1.0, -self.jerk * 2.0), jerk_max_l)
        self.cb_upper = np.clip(0.9 + accel * 0.2, 0, 1.2)
        self.cb_lower = np.clip(0.8 + accel * 0.2, 0, 1.2)

