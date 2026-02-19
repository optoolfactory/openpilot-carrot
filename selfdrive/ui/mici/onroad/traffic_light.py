import time
import pyray as rl
from openpilot.selfdrive.ui.mici.onroad import SIDE_PANEL_WIDTH
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.widgets import Widget


class TrafficLight(Widget):
  def __init__(self):
    super().__init__()
    self._radius = 24
    self._green_start_time = None
    self._visible = False
    self._current_state = 0

  def _update_state(self):
    state = ui_state.sm["longitudinalPlan"].trafficState

    if state == 1:
      # Red  항상 표시
      self._visible = True
      self._current_state = 1
      self._green_start_time = None

    elif state == 2:
      # Green  최대 10초 표시
      if self._current_state != 2:
        self._green_start_time = time.monotonic()

      self._current_state = 2

      if self._green_start_time and (time.monotonic() - self._green_start_time <= 10.0):
        self._visible = True
      else:
        self._visible = False

    else:
      self._visible = False
      self._current_state = 0
      self._green_start_time = None

  def is_visible(self):
    return self._visible

  def _render(self, _):
    if not self._visible:
      return

    content_rect = rl.Rectangle(
      self.rect.x + self.rect.width - SIDE_PANEL_WIDTH,
      self.rect.y,
      SIDE_PANEL_WIDTH,
      self.rect.height,
    )

    center_x = content_rect.x + content_rect.width - self._radius
    center_y = self.rect.y + self._radius  # confidence 최상단 위치와 동일

    if self._current_state == 1:
      color = rl.Color(255, 0, 0, 255)
    else:
      color = rl.Color(0, 255, 0, 255)

    rl.draw_circle(int(center_x), int(center_y), self._radius, color)
