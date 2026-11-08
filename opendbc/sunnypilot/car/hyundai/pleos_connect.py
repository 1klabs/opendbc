# Kia Pleos Connect (PV5): the camera's HUD, MDPS and LFA_ALT frames are replaced by openpilot.
# Kept separate from sunnypilot's CCNC handling, which targets torque steering cars.
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.hyundai.values import HyundaiFlags
from opendbc.sunnypilot.car.hyundai.values import HyundaiFlagsSP

LANE_CHANGE_SPEED_MIN = 8.9408
LANELINE_DEPARTURE = 4
LKA_ICON_OFF, LKA_ICON_STANDBY, LKA_ICON_ACTIVE = 0, 3, 4
LKA_ICON_ON = (LKA_ICON_STANDBY, LKA_ICON_ACTIVE)
LANELINE_SOLID, LANELINE_DASHED = 2, 8
LANE_QUALITY_VALID = (2, 3)

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


def lane_positions(msg_1b5):
  leftlaneraw, rightlaneraw = msg_1b5["Info_LftLnPosVal"], msg_1b5["Info_RtLnPosVal"]

  scale_per_m = 15 / 1.7
  leftlane = abs(int(round(15 + (leftlaneraw - 1.7) * scale_per_m)))
  rightlane = abs(int(round(15 + (rightlaneraw - 1.7) * scale_per_m)))

  if msg_1b5["Info_LftLnQualSta"] not in (2, 3):
    leftlane = 0
  if msg_1b5["Info_RtLnQualSta"] not in (2, 3):
    rightlane = 0

  if leftlaneraw == -2.0248375:
    leftlane = 30 - rightlane
  if rightlaneraw == 2.0248375:
    rightlane = 30 - leftlane

  if leftlaneraw == rightlaneraw == 0:
    leftlane = rightlane = 15
  elif leftlaneraw == 0:
    leftlane = 30 - rightlane
  elif rightlaneraw == 0:
    rightlane = 30 - leftlane

  total = leftlane + rightlane
  if total == 0:
    leftlane = rightlane = 15
  else:
    leftlane = round((leftlane / total) * 30)
    rightlane = 30 - leftlane
  return leftlane, rightlane


def line_type(camera_laneline):
  # the camera only reports dashed/solid while its own LFA is active, assume solid otherwise
  return camera_laneline if camera_laneline in (LANELINE_SOLID, LANELINE_DASHED) else LANELINE_SOLID


def create_pleos_connect_cluster(packer, CAN, openpilotLongitudinalControl, enabled, hud, leftBlinker, rightBlinker, msg_161, msg_162, msg_1b5,
                       is_metric, out, main_cruise_enabled, lfa_icon, ldw_enabled, curvature_step=0):
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
  # LDW follows openpilot's lane departure warning toggle: its icon and the camera's warnings like stock
  left_depart = ldw_enabled and (hud.leftLaneDepart or msg_161["LANELINE_LEFT"] == LANELINE_DEPARTURE)
  right_depart = ldw_enabled and (hud.rightLaneDepart or msg_161["LANELINE_RIGHT"] == LANELINE_DEPARTURE)

  msg_161.update({
    "DAW_ICON": 0,
    "LKA_ICON": (msg_161["LKA_ICON"] if msg_161["LKA_ICON"] in LKA_ICON_ON else LKA_ICON_STANDBY) if ldw_enabled else LKA_ICON_OFF,
    "LFA_ICON": 2 if lfa_icon else 0,
    "CENTERLINE": 1 if lfa_icon else 0,
    "LANELINE_CURVATURE": curvature_to_can(curvature_step if lfa_icon and not anyBlinker else 0),
    "LANELINE_LEFT": (LANELINE_DEPARTURE if left_depart else 0 if not lfa_icon else 1 if not hud.leftLaneVisible else 6 if anyBlinker else
                      line_type(msg_161["LANELINE_LEFT"])),
    "LANELINE_RIGHT": (LANELINE_DEPARTURE if right_depart else 0 if not lfa_icon else 1 if not hud.rightLaneVisible else 6 if anyBlinker else
                       line_type(msg_161["LANELINE_RIGHT"])),
    "LCA_LEFT_ICON": (0 if not lfa_icon or out.vEgo < LANE_CHANGE_SPEED_MIN else 1 if out.leftBlindspot else 2 if anyBlinker else 4),
    "LCA_RIGHT_ICON": (0 if not lfa_icon or out.vEgo < LANE_CHANGE_SPEED_MIN else 1 if out.rightBlindspot else 2 if anyBlinker else 4),
    "LCA_LEFT_ARROW": 2 if leftBlinker else 0,
    "LCA_RIGHT_ARROW": 2 if rightBlinker else 0,
  })

  if lfa_icon and (leftBlinker or rightBlinker):
    leftlane, rightlane = lane_positions(msg_1b5)
    msg_161["LANELINE_LEFT_POSITION"] = leftlane
    msg_161["LANELINE_RIGHT_POSITION"] = rightlane

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
    if new_161 and (curvature := lane_curvature(CS.msg_1b5)) is not None:
      self.curvature_step = curvature_step(curvature, self.curvature_step)

    if new_161 or new_162:
      msg_161, msg_162 = create_pleos_connect_cluster(self.packer, self.CAN, self.CP.openpilotLongitudinalControl, CC.enabled,
                                                      CC.hudControl, CC.leftBlinker, CC.rightBlinker, CS.msg_161, CS.msg_162,
                                                      CS.msg_1b5, CS.is_metric, CS.out, CS.main_cruise_enabled, self.lfa_icon,
                                                      self.ldw_enabled, self.curvature_step)
      can_sends += [msg for msg, new in ((msg_161, new_161), (msg_162, new_162)) if new]

    return can_sends
