# PV5-specific cluster encoding, kept separate from upstream CCNC behavior.
from opendbc.car.common.conversions import Conversions as CV

LANE_CHANGE_SPEED_MIN = 8.9408
LANELINE_CURVATURE = {i: (31 if i == -1 else 13 - abs(i + 15)) if i < 0 else 15 + i for i in range(-15, 16)}


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


def create_pv5_cluster(packer, CAN, openpilotLongitudinalControl, enabled, hud, leftBlinker, rightBlinker, msg_161, msg_162, msg_1b5,
                       is_metric, out, main_cruise_enabled, lfa_icon):
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

  msg_161.update({
    "DAW_ICON": 0,
    "LKA_ICON": 0,
    "LFA_ICON": 2 if lfa_icon else 0,
    "CENTERLINE": 1 if lfa_icon else 0,
    "LANELINE_CURVATURE": LANELINE_CURVATURE.get(max(-15, min(int(out.steeringAngleDeg / 4.5), 15)), 14) if lfa_icon and not anyBlinker else 15,
    "LANELINE_LEFT": (0 if not lfa_icon else 1 if not hud.leftLaneVisible else 4 if hud.leftLaneDepart else 6 if anyBlinker else 2),
    "LANELINE_RIGHT": (0 if not lfa_icon else 1 if not hud.rightLaneVisible else 4 if hud.rightLaneDepart else 6 if anyBlinker else 2),
    "LCA_LEFT_ICON": (0 if not lfa_icon or out.vEgo < LANE_CHANGE_SPEED_MIN else 1 if out.leftBlindspot else 2 if anyBlinker else 4),
    "LCA_RIGHT_ICON": (0 if not lfa_icon or out.vEgo < LANE_CHANGE_SPEED_MIN else 1 if out.rightBlindspot else 2 if anyBlinker else 4),
    "LCA_LEFT_ARROW": 2 if leftBlinker else 0,
    "LCA_RIGHT_ARROW": 2 if rightBlinker else 0,
  })

  if lfa_icon and (leftBlinker or rightBlinker):
    leftlane, rightlane = lane_positions(msg_1b5)
    msg_161["LANELINE_LEFT_POSITION"] = leftlane
    msg_161["LANELINE_RIGHT_POSITION"] = rightlane

  if hud.leftLaneDepart or hud.rightLaneDepart:
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
