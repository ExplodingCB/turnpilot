"""TurnPilot's nav view: the onroad page a swipe right from the camera view shows, for seeing the route and what TurnPilot is
thinking. The map on the left (the route north up, the car's arrow at its GPS fix and heading, the next maneuver and the destination),
the maneuver, what TurnPilot is doing and its next moves, and the time and distance left on the right.

The camera view is untouched: this is another page of the main scroller (mici/layouts/main.py), which owns the swipe. It only
renders while on screen, since the scroller skips pages out of view, so it costs nothing while the camera view shows. On screen,
the map image is rebuilt only when the route or its frame changes (ui/lib/route_map.py) and uploaded once per rebuild; each frame
draws one texture, a few markers and labels. The text is ui/lib/turnpilot_nav.py's.
"""
import time

import numpy as np
import pyray as rl

from openpilot.selfdrive.navigation.helpers import latest_gps_source
from openpilot.selfdrive.navigation.renderer import ARROW
from openpilot.selfdrive.turnpilot.control_text import TAKE_CONTROL
from openpilot.selfdrive.ui.lib.route_map import CORNER_RADIUS_PX, EDGE_PX, RouteMap, rotated
from openpilot.selfdrive.ui.lib.turnpilot_nav import (INFO, MAX_NOTES, PLAN, WARN, calibration_text, maneuver_lines, notes,
                                                      remaining_text, route_status, state_lines)
from openpilot.selfdrive.ui.mici.onroad.navigation import draw_maneuver_arrow
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import FONT_SCALE, FontWeight, TextAlignment, TextAlignmentVertical, gui_app
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.system.ui.widgets import Widget
from openpilot.system.ui.widgets.label import UnifiedLabel, gui_label

MAP_SIZE = 240  # the screen's height: a square map on the left
COLUMN_GAP = 16
RIGHT_MARGIN = 8
MIN_BEARING_SPEED = 1.  # m/s: slower, GPS bearing is noise, so the arrow keeps its last heading
LINE_SPACING = 0.9  # UnifiedLabel's: each line after the first adds this much of a line's height
TITLE_SIZES = (30, 27, 24)  # TurnPilot's state: the first that fits on one line, else the last, wrapped
REMAINING_Y = 206  # the time and distance left, along the bottom

MAP_BG = rl.Color(7, 13, 18, 255)  # the camera view's nav panel
TEXT = rl.Color(255, 255, 255, int(255 * 0.9))  # alert text
SUBTEXT = rl.Color(255, 255, 255, int(255 * 0.65))  # alert subtext
ACCENT = rl.Color(155, 230, 185, 255)  # the nav panel's arrow
ORANGE = rl.Color(255, 115, 0, 255)  # a userPrompt alert's
NOTE_COLORS = {WARN: ORANGE, PLAN: ACCENT, INFO: SUBTEXT}
LOST = rl.Color(140, 140, 140, 255)  # the car's arrow at its last fix, GPS lost
CAR_SCALE = 1.3  # mapsd's ego arrow, a little bigger on a coarser map


def draw_car(x: float, y: float, bearing: float, color: rl.Color):
  """mapsd's ego arrow, turned to the GPS bearing, over a black outline: the concave arrow as two triangles."""
  tip, right, notch, left = (rl.Vector2(x + px, y + py) for px, py in rotated(ARROW * CAR_SCALE, bearing))
  for a, b in ((tip, right), (right, notch), (notch, left), (left, tip)):
    rl.draw_line_ex(a, b, 4, rl.BLACK)
  for a, b, c in ((tip, notch, right), (tip, left, notch)):
    # raylib culls clockwise triangles; turning keeps the winding, but draw whichever way it came out
    if (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x) > 0:
      b, c = c, b
    rl.draw_triangle(a, b, c, color)


def draw_target(x: float, y: float, radius: float):
  """The destination: a white ring."""
  rl.draw_circle_v(rl.Vector2(x, y), radius + 2, rl.BLACK)
  rl.draw_circle_v(rl.Vector2(x, y), radius, rl.WHITE)
  rl.draw_circle_v(rl.Vector2(x, y), radius * 0.45, MAP_BG)


class NavView(Widget):
  def __init__(self):
    super().__init__()
    self._map = RouteMap(MAP_SIZE, MAP_SIZE)
    self._texture: rl.Texture | None = None
    self._uploaded = -1  # the RouteMap version in the texture
    self._route_frame = -1  # navRoute's recv_frame last read
    self._gps_key: tuple[str, int] | None = None
    self._car: tuple[float, float] | None = None  # (longitude, latitude) of the last fix, kept through a GPS gap
    self._bearing = 0.
    self._gps_ok = False
    self._title = UnifiedLabel('', 30, FontWeight.DISPLAY, TEXT)
    self._detail = UnifiedLabel('', 22, FontWeight.MEDIUM, SUBTEXT)
    self._notes = [UnifiedLabel('', 22, FontWeight.MEDIUM, SUBTEXT) for _ in range(MAX_NOTES)]

  def _update_state(self):
    sm = ui_state.sm
    now = time.monotonic()
    if sm.recv_frame['navRoute'] != self._route_frame:
      # navd resends its route every 10 s; RouteMap ignores a repeat. Read by frame, not updated: the scroller doesn't render (or
      # update) this page while the camera view shows, so the frame a new route arrived on may have passed
      self._route_frame = sm.recv_frame['navRoute']
      self._map.set_route(np.array([(c.longitude, c.latitude) for c in sm['navRoute'].coordinates], dtype=np.float64))
    source = latest_gps_source(sm)
    self._gps_ok = source is not None
    if source is not None and (source, sm.recv_frame[source]) != self._gps_key:
      self._gps_key = (source, sm.recv_frame[source])
      gps = sm[source]
      self._car = (gps.longitude, gps.latitude)
      if gps.speed > MIN_BEARING_SPEED:
        self._bearing = gps.bearingDeg
      self._map.update(self._car, now)
    elif self._map.frame is None:
      self._map.update(None, now)  # a route and no fix yet: frame the route alone

  def _render(self, rect: rl.Rectangle):
    rl.draw_rectangle_rec(rect, rl.BLACK)
    self._draw_map(rl.Rectangle(rect.x, rect.y, MAP_SIZE, MAP_SIZE))
    left = MAP_SIZE + COLUMN_GAP
    self._draw_column(rl.Rectangle(rect.x + left, rect.y, rect.width - left - RIGHT_MARGIN, rect.height))

  def _draw_map(self, rect: rl.Rectangle):
    rl.draw_rectangle_rounded(rect, 2 * CORNER_RADIUS_PX / rect.height, 12, MAP_BG)
    cx, cy = rect.x + rect.width / 2, rect.y + rect.height / 2
    if self._map.image is None or self._map.frame is None:
      # no route to draw: the car's heading if there's a fix, which says the GPS works before a destination is set
      if self._car is not None and self._gps_ok:
        draw_car(cx, cy, self._bearing, rl.WHITE)
      else:
        gui_label(rect, 'No GPS fix', 26, SUBTEXT, FontWeight.MEDIUM, alignment=TextAlignment.CENTER)
      return
    if self._uploaded != self._map.version:
      if self._texture is None:
        self._texture = rl.load_texture_from_image(rl.Image(None, MAP_SIZE, MAP_SIZE, 1, rl.PixelFormat.PIXELFORMAT_UNCOMPRESSED_R8G8B8A8))
      rl.update_texture(self._texture, rl.ffi.cast("void *", rl.ffi.from_buffer(self._map.image)))
      self._uploaded = self._map.version
    rl.draw_texture_ex(self._texture, rl.Vector2(rect.x, rect.y), 0., 1., rl.WHITE)  # mid-swipe, in step with the markers

    frame, rm = self._map.frame, self._map

    def at(meters):
      """A point on the map, None off it (the route behind the car once zoomed in)."""
      px = frame.to_px(meters)[0]
      return rl.Vector2(rect.x + px[0], rect.y + px[1]) if frame.inside(px, EDGE_PX) else None

    if (start := at(rm.meters[0])) is not None:
      rl.draw_circle_v(start, 5, SUBTEXT)
    if (end := at(rm.meters[-1])) is not None:
      draw_target(end.x, end.y, 7)
    instruction = ui_state.sm['navInstruction']
    if self._routing and instruction.maneuverType != 'arrive' and np.isfinite(rm.progress):
      # the next maneuver, its distance on from the car's place along the route
      if (turn := at(rm.point_at(rm.progress + max(0., instruction.maneuverDistance)))) is not None:
        rl.draw_circle_v(turn, 7, rl.BLACK)
        rl.draw_circle_v(turn, 5, rl.WHITE)
    if self._car is not None:
      px = frame.to_px(rm.car_meters(self._car))[0]
      draw_car(rect.x + px[0], rect.y + px[1], self._bearing, rl.WHITE if self._gps_ok else LOST)

  @property
  def _routing(self) -> bool:
    return ui_state.sm.valid['navInstruction'] and ui_state.sm.alive['navInstruction']

  def _draw_column(self, rect: rl.Rectangle):
    sm, metric = ui_state.sm, ui_state.is_metric
    x, y, w = rect.x, rect.y, rect.width
    top = TextAlignmentVertical.TOP
    instruction = sm['navInstruction']

    # the maneuver, or why there's none
    if self._routing:
      distance, road = maneuver_lines(instruction, metric)
      if instruction.maneuverType == 'arrive':
        draw_target(x + 22, y + 32, 13)
      else:
        draw_maneuver_arrow(x + 22, y + 32, instruction.maneuverModifier, ACCENT)
      gui_label(rl.Rectangle(x + 52, y + 8, w - 52, 46), distance, 40, TEXT, FontWeight.DISPLAY, alignment_vertical=top)
      gui_label(rl.Rectangle(x, y + 54, w, 28), road, 23, SUBTEXT, FontWeight.MEDIUM, alignment_vertical=top)
    else:
      title, detail = route_status(ui_state.nav_destination, ui_state.turnpilot_map_status)
      gui_label(rl.Rectangle(x, y + 12, w, 40), title, 32, TEXT, FontWeight.DISPLAY, alignment_vertical=top)
      if detail:
        gui_label(rl.Rectangle(x, y + 52, w, 28), detail, 24, SUBTEXT, FontWeight.MEDIUM, alignment_vertical=top)

    # what TurnPilot is doing or thinking, then its next moves, stacked down to the time left. The title takes the largest size it
    # fits on one line at, else wraps; the reason and the notes wrap to two lines (the camera view's status line wraps too, at this
    # width beside its nav panel); what doesn't fit after them is dropped, least important last
    plan = sm['turnpilotPlan'] if sm.alive['turnpilotPlan'] and sm.valid['turnpilotPlan'] else None
    title, detail = state_lines(plan, ui_state.engaged)
    alarm = plan is None or str(plan.state) in ('disabled', 'unavailable') or TAKE_CONTROL in detail
    font = gui_app.font(FontWeight.DISPLAY)
    self._title.set_font_size(next((s for s in TITLE_SIZES if measure_text_cached(font, title, s).x <= w), TITLE_SIZES[-1]))
    cal = sm['extrinsicsCalibration']
    calibration = calibration_text(str(cal.calStatus), cal.calPerc) if sm.seen['extrinsicsCalibration'] else ''
    lines = [(self._title, title, ORANGE if alarm else TEXT), (self._detail, detail, SUBTEXT)]
    lines += [(label, text, NOTE_COLORS[kind]) for label, (text, kind) in
              zip(self._notes, notes(plan, gps_ok=self._gps_ok, routing=self._routing, calibration=calibration, metric=metric), strict=False)]
    line_y, bottom = y + 84, y + (REMAINING_Y - 2 if self._routing else rect.height - 4)
    for label, text, color in lines:
      if not text:
        continue
      label.set_text(text)
      label.set_text_color(color)
      one = label.font_size * FONT_SCALE
      height = min(label.get_content_height(int(w)), one * (1 + LINE_SPACING), bottom - line_y)
      if height + 1 < one:
        break
      label.render(rl.Rectangle(x, line_y, w, height + 1))
      line_y += height + 2

    if self._routing:
      gui_label(rl.Rectangle(x, y + REMAINING_Y, w, 32), remaining_text(instruction, metric), 28, TEXT, FontWeight.DISPLAY,
                alignment_vertical=top)

  def close(self):
    if self._texture is not None:
      rl.unload_texture(self._texture)
      self._texture = None
