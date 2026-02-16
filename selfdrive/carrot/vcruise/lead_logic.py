# /data/openpilot/selfdrive/carrot/vcruise/lead_logic.py

from dataclasses import dataclass

@dataclass
class LeadState:
  d_rel: float = 0.0   # m
  v_rel: float = 0.0   # m/s


class LeadSafetyStop:
  def __init__(self, decel_rate: float = 1.5):
    self.decel_rate = float(decel_rate)

  def check_safe_stop(self, v_ego_ms: float, lead: LeadState, safe_distance: float = 4.0):
    decel = max(0.1, self.decel_rate)

    d_stop_ego = (v_ego_ms ** 2) / (2.0 * decel)
    d_stop_rel = (lead.v_rel ** 2) / (2.0 * decel)
    d_final = float(lead.d_rel) - d_stop_ego - d_stop_rel

    return (d_final >= safe_distance), d_final
