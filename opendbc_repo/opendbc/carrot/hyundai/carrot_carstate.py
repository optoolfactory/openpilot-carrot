from opendbc.car.interfaces import CarStateBase
from openpilot.common.params import Params
from opendbc.car import Bus, create_button_events, structs, DT_CTRL
from opendbc.car.hyundai.values import HyundaiFlags

ButtonType = structs.CarState.ButtonEvent.Type
GearShifter = structs.CarState.GearShifter

class CarrotCarState(CarStateBase):
  def _carrot_init(self, CP):
    self.controls_ready_count = 0
    self.cp_bsm = None
    self._params = Params()

    self.cp = None
    self.cp_cam = None
    self.cp_alt = None

    self.mdps = None
    self.steer_touch_2af = None
    self.tcs = None
    self.adrv_161 = None
    self.lfa_alt = None
    self.lfa = None
    self.ccnc_162 = None
    self.lfahda_cluster = None
    self.scc_control = None
    self.adrv_200 = None
    self.adrv_1ea = None
    self.cruise_buttons_msg = None
    self.gear_step = None
    self.lane_info = None

    self.paddle_button_prev = 0


  def _carrot_monitor_fingerprint(self, can_parsers):
    self.cp = can_parsers[Bus.pt]
    self.cp_cam = can_parsers[Bus.cam]
    self.cp_alt = can_parsers[Bus.alt] if Bus.alt in can_parsers else None

    if self.controls_ready_count <= 200:
      def add_if_seen(parser, name):
        msg = parser.dbc.name_to_msg.get(name)
        if not msg:
          print(f"{name} not in DBC")
          return
        if msg.address not in parser.seen_addresses:
          return
        if msg.address in parser.addresses:
          return
        parser._add_message(name)   # ← 이름으로 등록

      def add_and_cache(parser, name: str, attr: str):
        add_if_seen(parser, name)
        if name in parser.vl:   # 등록 성공했을 때만
          setattr(self, attr, parser.vl[name])

      if self._params.get_bool("ControlsReady"):
        self.controls_ready_count += 1

      if self.controls_ready_count == 50:  # after 500msec
        self.cp.enable_capture = self.cp_cam.enable_capture = True
        if self.cp_alt is not None:
          self.cp_alt.enable_capture = True
      elif self.controls_ready_count == 100: # after 1sec
        self.cp.enable_capture = self.cp_cam.enable_capture = False
        if self.cp_alt is not None:
          self.cp_alt.enable_capture = False
        if 69 in self.cp.seen_addresses:
          self.gear_msg_canfd = "GEAR"
        if 442 in self.cp.seen_addresses:
          self.cp_bsm = self.cp
        elif 442 in self.cp_cam.seen_addresses:
          self.cp_bsm = self.cp_cam
      elif self.controls_ready_count == 101:
        print("cp_cam.seen_addresses =", self.cp_cam.seen_addresses)
      elif self.controls_ready_count == 102:
        print("cp.seen_addresses =", self.cp.seen_addresses)
      elif self.controls_ready_count == 103:
        if self.cp_alt is not None:
          print("cp_alt.seen_addresses =", self.cp_alt.seen_addresses)
        else:
          print("cp_alt.seen_addresses = None")
      elif self.controls_ready_count == 104:
        add_and_cache(self.cp, "MDPS", "mdps")
        add_and_cache(self.cp, "STEER_TOUCH_2AF", "steer_touch_2af")
        add_and_cache(self.cp, "TCS", "tcs")
      elif self.controls_ready_count == 105:
        add_and_cache(self.cp_cam, "ADRV_0x161", "adrv_161")
        add_and_cache(self.cp_cam, "LFA_ALT", "lfa_alt")
        add_and_cache(self.cp_cam, "LFA", "lfa")
      elif self.controls_ready_count == 106:
        add_and_cache(self.cp_cam, "CCNC_0x162", "ccnc_162")
        add_and_cache(self.cp_cam, "LFAHDA_CLUSTER", "lfahda_cluster")
        add_and_cache(self.cp_cam, "SCC_CONTROL", "scc_control")
      elif self.controls_ready_count == 107:
        add_and_cache(self.cp_cam, "ADRV_0x200", "adrv_200")
        add_and_cache(self.cp_cam, "ADRV_0x1ea", "adrv_1ea")
        add_and_cache(self.cp, self.cruise_btns_msg_canfd, "cruise_buttons_msg")
      elif self.controls_ready_count == 108:
        add_and_cache(self.cp, "GEAR", "gear_step")
        add_and_cache(self.cp, "GEAR_ALT", "gear_step") # gear_alt가 있으면 우선함.
      elif self.controls_ready_count == 109:
        if self.cp_alt is not None:
          add_and_cache(self.cp_alt, "CAM_0x362", "lane_info")
          add_and_cache(self.cp_alt, "CAM_0x2a4", "lane_info")


  def _carrot_update_canfd(self, ret):

    ret.cruiseState.available = self.scc_control is not None and self.scc_control["MainMode_ACC"] == 1
    vEgoClu, aEgoClu = self.update_clu_speed_kf(ret.vEgoCluster)
    ret.vCluRatio = (ret.vEgo / vEgoClu) if (vEgoClu > 3. and ret.vEgo > 3.) else 1.0

    if self.CP.flags & HyundaiFlags.ANGLE_CONTROL:
      ret.steeringAngleDeg = self.cp.vl["MDPS"]["STEERING_ANGLE_2"]

    # TPMS
    #tpms_unit = self.cp.vl["TPMS"]["UNIT"] * 0.725 if int(self.cp.vl["TPMS"]["UNIT"]) > 0 else 1.
    #ret.tpms.fl = tpms_unit * self.cp.vl["TPMS"]["PRESSURE_FL"]
    #ret.tpms.fr = tpms_unit * self.cp.vl["TPMS"]["PRESSURE_FR"]
    #ret.tpms.rl = tpms_unit * self.cp.vl["TPMS"]["PRESSURE_RL"]
    #ret.tpms.rr = tpms_unit * self.cp.vl["TPMS"]["PRESSURE_RR"]

    # BSM
    if self.cp_bsm is not None:
      bsm_info = self.cp_bsm.vl["BLINDSPOTS_REAR_CORNERS"]
      ret.leftBlindspot = (bsm_info["FL_INDICATOR"] + bsm_info["INDICATOR_LEFT_TWO"] + bsm_info["INDICATOR_LEFT_FOUR"]) > 0
      ret.rightBlindspot = (bsm_info["FR_INDICATOR"] + bsm_info["INDICATOR_RIGHT_TWO"] + bsm_info["INDICATOR_RIGHT_FOUR"]) > 0

    # brakeLights
    ret.brakeLights = ret.brakePressed or self.cp.vl["TCS"]["BrakeLight"] == 1

    ret.gearStep = self.gear_step["GEAR_STEP"] if self.gear_step is not None else 0
    if 1 <= ret.gearStep <= 8 and ret.gearShifter == GearShifter.unknown:
      ret.gearShifter = GearShifter.drive

    if self.lane_info is not None:
      #left_lane_prob = self.lane_info["LEFT_LANE_PROB"]
      #right_lane_prob = self.lane_info["RIGHT_LANE_PROB"]
      left_lane_type = self.lane_info["LEFT_LANE_TYPE"] # 0: dashed, 1: solid, 2: undecided, 3: road edge, 4: DLM Inner Solid, 5: DLM InnerDashed, 6:DLM Inner Undecided, 7: Botts Dots, 8: Barrier
      right_lane_type = self.lane_info["RIGHT_LANE_TYPE"]
      left_lane_color = self.lane_info["LEFT_LANE_COLOR"]
      right_lane_color = self.lane_info["RIGHT_LANE_COLOR"]
      left_lane_info = left_lane_color * 10 + left_lane_type
      right_lane_info = right_lane_color * 10 + right_lane_type
      ret.leftLaneLine = left_lane_info
      ret.rightLaneLine = right_lane_info

    paddle_button = self.paddle_button_prev
    if self.cruise_btns_msg_canfd == "CRUISE_BUTTONS":
      paddle_button = 1 if self.cp.vl["CRUISE_BUTTONS"]["LEFT_PADDLE"] == 1 else 2 if self.cp.vl["CRUISE_BUTTONS"]["RIGHT_PADDLE"] == 1 else 0
    elif self.gear_msg_canfd == "GEAR":
      paddle_button = 1 if self.cp.vl["GEAR"]["LEFT_PADDLE"] == 1 else 2 if self.cp.vl["GEAR"]["RIGHT_PADDLE"] == 1 else 0

    new_events = create_button_events(
      paddle_button, self.paddle_button_prev,
      {1: ButtonType.paddleLeft, 2: ButtonType.paddleRight}
    )
    def add_events(builder_list, extra_events):
      base = [structs.CarState.ButtonEvent(pressed=e.pressed, type=e.type) for e in builder_list]
      return base + extra_events
    ret.buttonEvents = add_events(ret.buttonEvents, new_events)
    
    self.paddle_button_prev = paddle_button
  
    return ret
