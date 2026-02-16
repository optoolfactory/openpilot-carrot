# /data/openpilot/selfdrive/carrot/vcruise/personality.py

import numpy as np
from openpilot.common.params import Params

class PersonalityController:
  def __init__(self):
    self.params = Params()

  def cycle(self, pcm_cruise_gap: int):
    longitudinal_personality_max = self.params.get_int("LongitudinalPersonalityMax")

    if pcm_cruise_gap == 0:
      # 차량 gap 정보 없을 때: params 기반 순환
      personality = (self.params.get_int("LongitudinalPersonality") - 1) % longitudinal_personality_max
    else:
      # 차량 gap 정보 있을 때: 그 값으로 매핑
      personality = int(np.clip(pcm_cruise_gap - 1, 0, longitudinal_personality_max))

    self.params.put_int_nonblocking("LongitudinalPersonality", personality)
    return personality
