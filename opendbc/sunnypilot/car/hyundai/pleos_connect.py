# Kia Pleos Connect (PV5): the camera's HUD, MDPS and LFA_ALT frames are replaced by openpilot.
# Kept separate from sunnypilot's CCNC handling, which targets torque steering cars.
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.hyundai.values import HyundaiFlags
from opendbc.sunnypilot.car.hyundai.values import HyundaiFlagsSP

LANE_CHANGE_SPEED_MIN = 8.9408
LANELINE_DEPARTURE = 4
LKA_ICON_OFF, LKA_ICON_STANDBY, LKA_ICON_ACTIVE = 0, 3, 4
LKA_ICON_ON = (LKA_ICON_STANDBY, LKA_ICON_ACTIVE)
LANELINE_HIDDEN, LANELINE_SOLID, LANELINE_DASHED, LANELINE_GREEN_DASHED = 1, 2, 8, 10
LANE_QUALITY_VALID = (2, 3)
LANE_FILL = 1  # LANE_LEFT/LANE_RIGHT/LANE_HIGHLIGHT: lane filled green
LANE_HIGHLIGHT_DISTANCE_MAX = 204.7  # m, ego lane fill length without a lead vehicle

# Lane change display. LANELINE_*_POSITION places each line by its distance from the car, 15 per half lane
LANE_POSITION_PER_M = 15 / 1.7
LANE_WIDTH = 30
LANE_CHANGE_NEAR = 5  # crossed line turns green and the far line hides once the car is this close to it
LANE_CHANGE_REVEAL_FRAMES = 10  # 0.5 s of 20 Hz HUD frames the new lane's far line stays hidden after crossing
LANE_CHANGE_RELABEL_JUMP = 10  # the camera relabels its lines after the car crosses, jumping the crossed side outwards

# LANELINE_CURVATURE steps (-15..15, negative is a right curve) from FR_CMR_03_50ms lane curvature,
# upper |curvature| bound of steps 0..14 in 1/km, fit to the stock camera's display
CURVATURE_STEP_BP = (0.173, 0.477, 0.731, 0.958, 1.194, 1.445, 1.683, 1.958, 2.198, 2.395, 2.62, 2.83, 3.0, 3.29, 3.75)
CURVATURE_STEP_MAX = len(CURVATURE_STEP_BP)
CURVATURE_HYSTERESIS = 0.08  # 1/km, matches the stock step change rate


def lane_curvature(msg_1b5) -> float | None:
  # 1/m, positive curves right. None without a valid lane line
  valid = [msg_1b5[f"Info_{side}LnCvtrVal"] for side in ("Lft", "Rt") if msg_1b5[f"Info_{side}LnQualSta"] in LANE_QUALITY_VALID]
  return sum(valid) / len(valid) if valid else None


def curvature_step(curvature: float, last_step: int = 0) -> int:
  magnitude = abs(curvature) * 1e3
  step = sum(magnitude >= bp for bp in CURVATURE_STEP_BP)
  step = -step if curvature > 0 else step

  # hold the last step while the curvature stays within its band widened by the hysteresis
  last = abs(last_step)
  lower = CURVATURE_STEP_BP[last - 1] - CURVATURE_HYSTERESIS if last > 0 else 0.0
  upper = CURVATURE_STEP_BP[last] + CURVATURE_HYSTERESIS if last < CURVATURE_STEP_MAX else float('inf')
  same_direction = last_step == 0 or (last_step < 0) == (curvature > 0)
  if step != last_step and same_direction and lower <= magnitude < upper:
    return last_step
  return step


def curvature_to_can(step: int) -> int:
  # signed 5 bit with an offset of 15, negative steps are stored as 15 + |step|
  return (step if step >= 0 else 15 - step) + 15


def lane_line_offset(msg_1b5, side: str) -> float | None:
  # distance from the car to a lane line in lane positions, positive while the line is on its own side.
  # the camera reports left lines negative and right lines positive, None without a valid line
  if msg_1b5[f"Info_{side}LnQualSta"] not in LANE_QUALITY_VALID:
    return None
  offset = msg_1b5[f"Info_{side}LnPosVal"]
  return (-offset if side == "Lft" else offset) * LANE_POSITION_PER_M


class LaneChangeDisplay:
  # Stock-like lane change: the target lane fills green with the crossed line dashed. Close to the crossed line it turns green
  # and the far line hides, so the camera relabeling its lines at the crossing is never drawn. After crossing the crossed line
  # stays green on the other side, the new lane takes the fill and its far line appears once it settled.
  def __init__(self):
    self.reset(None)

  def reset(self, direction: str | None) -> None:
    self.direction = direction
    self.crossed = False
    self.position: float | None = None  # distance to the crossed line, before and after crossing, in lane positions
    self.frames_since_crossed = 0

  def update(self, direction: str | None, msg_1b5) -> dict | None:
    # direction is "left", "right" or None, called once per new HUD frame
    if direction != self.direction:
      self.reset(direction)
    if direction is None:
      return None

    near_side, far_side = ("Lft", "Rt") if direction == "left" else ("Rt", "Lft")
    near, far = lane_line_offset(msg_1b5, near_side), lane_line_offset(msg_1b5, far_side)

    if not self.crossed:
      if near is not None and near <= 0:
        self._cross(-near)  # the crossed line is now past the car's center
      elif near is not None and self.position is not None and near > self.position + LANE_CHANGE_RELABEL_JUMP:
        self._cross(far if far is not None else 0.)  # camera already relabeled: the crossed line is the far side line
      elif near is not None:
        # a lane change only moves towards the line, filter out camera noise
        self.position = near if self.position is None else min(self.position, near)
    else:
      self.frames_since_crossed += 1
      past = -near if near is not None and near <= 0 else far
      if past is not None:
        self.position = max(self.position, past)

    return self._fields(direction)

  def _cross(self, past: float) -> None:
    self.crossed = True
    self.frames_since_crossed = 0
    self.position = max(past, 0.)

  def _fields(self, direction: str) -> dict:
    near_lane, far_lane = ("LEFT", "RIGHT") if direction == "left" else ("RIGHT", "LEFT")
    position = LANE_WIDTH // 2 if self.position is None else int(round(min(max(self.position, 0), LANE_WIDTH)))

    if not self.crossed:
      close = position <= LANE_CHANGE_NEAR
      return {
        f"LANE_{near_lane}": LANE_FILL,
        f"LANE_{far_lane}": 0,
        "LANE_HIGHLIGHT": 0,
        f"LANELINE_{near_lane}": LANELINE_GREEN_DASHED if close else LANELINE_DASHED,
        f"LANELINE_{far_lane}": LANELINE_HIDDEN if close else LANELINE_SOLID,
        f"LANELINE_{near_lane}_POSITION": position,
        # while hidden, the far line waits under the car to take over the crossed line after crossing
        f"LANELINE_{far_lane}_POSITION": position if close else LANE_WIDTH - position,
      }

    # after crossing the crossed line is on the far side of the car, the near side shows the new lane's outer line
    return {
      "LANE_LEFT": 0,
      "LANE_RIGHT": 0,
      "LANE_HIGHLIGHT": LANE_FILL,
      f"LANELINE_{far_lane}": LANELINE_GREEN_DASHED,
      f"LANELINE_{near_lane}": LANELINE_HIDDEN if self.frames_since_crossed < LANE_CHANGE_REVEAL_FRAMES else LANELINE_SOLID,
      f"LANELINE_{far_lane}_POSITION": position,
      f"LANELINE_{near_lane}_POSITION": LANE_WIDTH - position,
    }


def line_type(camera_laneline):
  # the camera only reports dashed/solid while its own LFA is active, assume solid otherwise
  return camera_laneline if camera_laneline in (LANELINE_SOLID, LANELINE_DASHED) else LANELINE_SOLID


def create_pleos_connect_cluster(packer, CAN, openpilotLongitudinalControl, enabled, hud, leftBlinker, rightBlinker, msg_161, msg_162, msg_1b5,
                       is_metric, out, main_cruise_enabled, lfa_icon, ldw_enabled, curvature_step=0, lane_change=None):
  # lane_change: LaneChangeDisplay fields while an openpilot lane change is shown
  # Never modify the cached camera frames when constructing replacement HUD messages.
  msg_161 = msg_161.copy()
  msg_162 = msg_162.copy()

  for f in {"FAULT_LSS", "FAULT_HDA", "FAULT_DAS", "FAULT_LFA", "FAULT_DAW", "FAULT_ESS"}:
    msg_162[f] = 0
  if msg_161["ALERTS_2"] == 5:
    msg_161.update({"ALERTS_2": 0, "SOUNDS_2": 0})
  if msg_161["ALERTS_3"] == 17:
    msg_161["ALERTS_3"] = 0
  if msg_161["ALERTS_5"] in (2, 5):
    msg_161["ALERTS_5"] = 0
  if msg_161["SOUNDS_4"] == 2 and msg_161["LFA_ICON"] in (3, 0,):
    msg_161["SOUNDS_4"] = 0

  anyBlinker = leftBlinker or rightBlinker
  lane_changing = bool(lfa_icon) and lane_change is not None
  # LDW follows openpilot's lane departure warning toggle: its icon and the camera's warnings like stock
  left_depart = ldw_enabled and (hud.leftLaneDepart or msg_161["LANELINE_LEFT"] == LANELINE_DEPARTURE)
  right_depart = ldw_enabled and (hud.rightLaneDepart or msg_161["LANELINE_RIGHT"] == LANELINE_DEPARTURE)
  # the ego lane fill reaches up to the lead vehicle
  lead_distance = msg_1b5["Longitudinal_Distance"]

  msg_161.update({
    "DAW_ICON": 0,
    "LKA_ICON": (msg_161["LKA_ICON"] if msg_161["LKA_ICON"] in LKA_ICON_ON else LKA_ICON_STANDBY) if ldw_enabled else LKA_ICON_OFF,
    "LFA_ICON": 2 if lfa_icon else 0,
    "CENTERLINE": 1 if lfa_icon else 0,
    "LANELINE_CURVATURE": curvature_to_can(curvature_step if lfa_icon and not lane_changing else 0),
    "LANELINE_LEFT": (LANELINE_DEPARTURE if left_depart else 0 if not lfa_icon else LANELINE_HIDDEN if not hud.leftLaneVisible else
                      line_type(msg_161["LANELINE_LEFT"])),
    "LANELINE_RIGHT": (LANELINE_DEPARTURE if right_depart else 0 if not lfa_icon else LANELINE_HIDDEN if not hud.rightLaneVisible else
                       line_type(msg_161["LANELINE_RIGHT"])),
    "LCA_LEFT_ICON": (0 if not lfa_icon or out.vEgo < LANE_CHANGE_SPEED_MIN else 1 if out.leftBlindspot else 2 if anyBlinker else 4),
    "LCA_RIGHT_ICON": (0 if not lfa_icon or out.vEgo < LANE_CHANGE_SPEED_MIN else 1 if out.rightBlindspot else 2 if anyBlinker else 4),
    # chevrons only while the lane change can go ahead, like stock with a vehicle in the blind spot
    "LCA_LEFT_ARROW": 2 if leftBlinker and not out.leftBlindspot else 0,
    "LCA_RIGHT_ARROW": 2 if rightBlinker and not out.rightBlindspot else 0,
    "LANE_HIGHLIGHT_DISTANCE": lead_distance if lead_distance > 0 else LANE_HIGHLIGHT_DISTANCE_MAX,
  })

  if lane_changing:
    msg_161.update(lane_change)

  if ldw_enabled and (hud.leftLaneDepart or hud.rightLaneDepart):
    msg_162["VIBRATE"] = 1

  if openpilotLongitudinalControl:
    if msg_161["ALERTS_3"] in (1, 2, 3, 4, 7, 8, 9, 10):
      msg_161["ALERTS_3"] = 0
    if msg_161["ALERTS_5"] == 4:
      msg_161["ALERTS_5"] = 0
    if msg_161["SOUNDS_3"] == 5:
      msg_161["SOUNDS_3"] = 0

    set_speed = 255
    if main_cruise_enabled:
      set_speed = round(out.vCruiseCluster * (1 if is_metric else CV.KPH_TO_MPH))
      if set_speed > (145 if is_metric else 90):
        set_speed = 40 if is_metric else 25

    msg_161.update({
      "SETSPEED": 3 if enabled else 1,
      "SETSPEED_HUD": 0 if not main_cruise_enabled else 2 if enabled else 1,
      "SETSPEED_SPEED": set_speed,
      "DISTANCE": hud.leadDistanceBars,
      "DISTANCE_SPACING": 0 if not main_cruise_enabled else 1 if enabled else 3,
      "DISTANCE_LEAD": 0 if not main_cruise_enabled else 2 if enabled and hud.leadVisible else 1 if hud.leadVisible else 0,
      "DISTANCE_CAR": 0 if not main_cruise_enabled else 2 if enabled else 1,
      "SLA_ICON": 0,
      "NAV_ICON": 0,
      "TARGET": 0,
    })

    msg_162["LEAD"] = 0 if not main_cruise_enabled else 2 if enabled else 1
    msg_162["LEAD_DISTANCE"] = msg_1b5["Longitudinal_Distance"]

  return [packer.make_can_msg(msg, CAN.ECAN, data) for msg, data in [("CCNC_0x161", msg_161), ("CCNC_0x162", msg_162)]]


def create_pleos_connect_mdps(packer, CAN, mdps, cam_lfa_alt):
  # The camera expects the MDPS to acknowledge its own angle control request, and latches
  # FAULT_LFA/FAULT_LSS/FAULT_DAS after ~0.5s without it. Its LFA_ALT never reaches the MDPS,
  # so mirror the request back the same way the MDPS does on a stock car (~30ms later).
  values = mdps.copy()
  values["MDPS_ADASAciActvSta_Lv2"] = cam_lfa_alt["ADAS_ActvACILvl2Sta"]
  return packer.make_can_msg("MDPS", CAN.CAM, values)


class PleosConnectCarController:
  def __init__(self, CP, CP_SP):
    self.pleos_connect = bool(CP.flags & HyundaiFlags.PLEOS_CONNECT)
    self.ldw_enabled = bool(CP_SP.flags & HyundaiFlagsSP.LDW_ENABLED)
    self.last_counters: dict[int, int] = {}
    self.curvature_step = 0
    self.lane_change_display = LaneChangeDisplay()
    self.lane_change = None

  def _new_frame(self, addr: int, msg) -> bool:
    # Forward once per source frame so the receiver sees the source's counter sequence
    counter = msg.get("COUNTER")
    last = self.last_counters.get(addr)
    self.last_counters[addr] = counter
    return last is not None and counter != last

  def update(self, CS, CC):
    can_sends = []
    if not self.pleos_connect:
      return can_sends

    if self._new_frame(0xEA, CS.mdps):
      can_sends.append(create_pleos_connect_mdps(self.packer, self.CAN, CS.mdps, CS.cam_lfa_alt))

    new_161, new_162 = self._new_frame(0x161, CS.msg_161), self._new_frame(0x162, CS.msg_162)
    if new_161:
      if (curvature := lane_curvature(CS.msg_1b5)) is not None:
        self.curvature_step = curvature_step(curvature, self.curvature_step)
      # controlsd only sets the blinkers during an openpilot lane change, not while a vehicle blocks the target lane
      direction = None
      if self.lfa_icon and CC.leftBlinker != CC.rightBlinker:
        left = CC.leftBlinker
        if not (CS.out.leftBlindspot if left else CS.out.rightBlindspot):
          direction = "left" if left else "right"
      self.lane_change = self.lane_change_display.update(direction, CS.msg_1b5)

    if new_161 or new_162:
      msg_161, msg_162 = create_pleos_connect_cluster(self.packer, self.CAN, self.CP.openpilotLongitudinalControl, CC.enabled,
                                                      CC.hudControl, CC.leftBlinker, CC.rightBlinker, CS.msg_161, CS.msg_162,
                                                      CS.msg_1b5, CS.is_metric, CS.out, CS.main_cruise_enabled, self.lfa_icon,
                                                      self.ldw_enabled, self.curvature_step, self.lane_change)
      can_sends += [msg for msg, new in ((msg_161, new_161), (msg_162, new_162)) if new]

    return can_sends
