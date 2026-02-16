import numpy as np

class CurveSpeedEstimator:
  def __init__(self, params):
    self.params = params
    self.autoCurveSpeedFactor = 1.0
    self.autoCurveSpeedAggressiveness = 1.0

  def _update_params(self):
    # 원본: int * 0.01
    self.autoCurveSpeedFactor = self.params.get_int("AutoCurveSpeedFactor") * 0.01
    self.autoCurveSpeedAggressiveness = self.params.get_int("AutoCurveSpeedAggressiveness") * 0.01

  def calc(self, sm):
    self._update_params()

    if not (sm.alive['carState'] and sm.alive['modelV2']):
      return 250

    modelData = sm['modelV2']
    CS = sm['carState']

    if len(modelData.orientationRate.z) == 0:
      return 250

    TARGET_LAT_A = 1.9  # m/s^2 (원본)

    v_ego = max(CS.vEgo, 0.1)

    orientation_rate = np.array(modelData.orientationRate.z) * self.autoCurveSpeedFactor
    velocity = np.array(modelData.velocity.x)

    max_index = int(np.argmax(np.abs(orientation_rate)))
    curv_direction = float(np.sign(orientation_rate[max_index]))
    max_pred_lat_acc = float(np.amax(np.abs(orientation_rate) * velocity))

    # max_curve
    max_curve = max_pred_lat_acc / (v_ego ** 2) if v_ego > 0 else 0.0
    if abs(max_curve) < 1e-6:
      return 250

    adjusted_target_lat_a = TARGET_LAT_A * self.autoCurveSpeedAggressiveness

    turnSpeed = max(abs(adjusted_target_lat_a / max_curve) ** 0.5 * 3.6, 5.0)
    turnSpeed = min(turnSpeed, 250.0)
    return float(turnSpeed * (curv_direction if curv_direction != 0 else 1.0))
