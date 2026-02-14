from dataclasses import dataclass

@dataclass
class VCruiseOutput:
  soft_hold_active: int = 0
  activate_cruise: int = 0
  carrot_cruise: int = 0

  paddle_decel_active: bool = False
  v_cruise_cluster_kph: float = 0.0
  v_cruise_display_kph: float = 0.0 
  log: str = ""