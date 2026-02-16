from dataclasses import dataclass
from openpilot.common.params import Params
from opendbc.car.common.conversions import Conversions as CV

@dataclass
class VCruiseConfig:
  cruise_speed_min: int = 5
  cruise_speed_max: int = 161

  # 버튼 단위
  speed_up_unit_basic: int = 1
  speed_down_unit: int = 10

  # 버튼 동작 모드 (원 코드의 핵심만)
  cruise_button_mode: int = 2     # 0/1/2/3 중 사용하던 것. table은 제거했지만 mode 값은 남겨둠(필요 시)
  cancel_button_mode: int = 0
  paddle_mode: int = 0

  # 자동 크루즈 활성 허용 (기존 AutoCruiseControl)
  auto_cruise_control: int = 0

  # gas tok
  auto_gas_tok_speed: float = 0.0
  auto_gas_sync_speed: bool = False

  # 기타
  speed_from_pcm: int = 0
  cruise_speed_unit: int = 10
  cruise_speed_unit_basic: int = 1
  activate_cruise_after_brake: bool = False
  cruise_on_dist: float = 0.0


class ParamsManager:
  def __init__(self):
    self.params = Params()

  def read(self, is_metric: bool) -> VCruiseConfig:
    # unit_factor는 “파라미터 값이 mph 기준으로 저장되는 경우”를 대비한 장치였는데,
    # 지금은 core만 남겨서 그대로 둡니다(필요 없으면 제거 가능).
    unit_factor = 1.0 if is_metric else CV.MPH_TO_KPH

    cfg = VCruiseConfig()
    cfg.auto_cruise_control = 1 #self.params.get_int("AutoCruiseControl")

    cfg.auto_gas_tok_speed = 20 #self.params.get_float("AutoGasTokSpeed") * unit_factor
    cfg.auto_gas_sync_speed = True #bool(self.params.get_bool("AutoGasSyncSpeed"))

    cfg.speed_from_pcm = 0 #self.params.get_int("SpeedFromPCM")
    cfg.cruise_speed_unit = 10 #int(self.params.get_int("CruiseSpeedUnit"))
    cfg.cruise_speed_unit_basic = 1 #int(self.params.get_int("CruiseSpeedUnitBasic"))
    cfg.paddle_mode = 0 #int(self.params.get_int("PaddleMode"))
    cfg.cruise_button_mode = 0 #int(self.params.get_int("CruiseButtonMode"))
    cfg.cancel_button_mode = 0 #int(self.params.get_int("CancelButtonMode"))
    cfg.cruise_on_dist = 400 #float(self.params.get_float("CruiseOnDist") * 0.01)

    cfg.activate_cruise_after_brake = True #bool(self.params.get_bool("ActivateCruiseAfterBrake"))

    return cfg
