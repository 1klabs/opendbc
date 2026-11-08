import itertools
import unittest

from opendbc.can import CANPacker, CANParser
from opendbc.car import gen_empty_fingerprint, structs
from opendbc.car.hyundai.interface import CarInterface
from opendbc.car.hyundai.hyundaicanfd import CanBus
from opendbc.car.hyundai.values import CAR, CarControllerParams, HyundaiFlags
from opendbc.car.vehicle_model import VehicleModel, calc_slip_factor
from opendbc.sunnypilot.car.hyundai.pleos_connect import CURVATURE_STEP_BP, LANE_CHANGE_NEAR, LANE_CHANGE_REVEAL_FRAMES, LaneChangeDisplay, \
  create_pleos_connect_cluster, create_pleos_connect_mdps, curvature_step, curvature_to_can, lane_curvature


def lane_msg(left, right, quality=3, lead=0.):
  # FR_CMR_03_50ms with lane line offsets in m, the camera reports the left line negative
  return {"Info_LftLnPosVal": left, "Info_RtLnPosVal": right, "Info_LftLnQualSta": quality, "Info_RtLnQualSta": quality,
          "Longitudinal_Distance": lead}


def lane_change_drive(direction, relabel_at=13):
  # car moving 0.17 m per frame towards the target lane from the lane center. After crossing the camera keeps the crossed line
  # on the old side with a flipped sign for a few frames before relabeling it, plus one frame of noise backwards
  sign = 1 if direction == "left" else -1
  msgs = []
  for k in range(26):
    lateral = 0.17 * k + (0.1 if k == 5 else 0.)  # noise: one frame looks closer than the next
    if k == 6:
      lateral = 0.17 * 4
    if k < relabel_at:  # crossed line still labeled on the old side
      near, far = 1.7 - lateral, 1.7 + lateral
    else:  # relabeled: crossed line on the far side, new outer line on the near side
      near, far = 3.4 - (lateral - 1.7), lateral - 1.7
    left, right = (-near, far) if sign == 1 else (-far, near)
    msgs.append(lane_msg(left, right))
  return msgs
from opendbc.sunnypilot.car.hyundai.values import HyundaiFlagsSP, HyundaiSafetyFlagsSP
from opendbc.sunnypilot.car.interfaces import setup_interfaces


class TestPleosConnectIntegration(unittest.TestCase):
  def test_canfd_torque_limits_unchanged(self):
    for car in CAR:
      if car.config.flags & HyundaiFlags.CANFD:
        cp = CarInterface.get_params(car, gen_empty_fingerprint(), [], False, False, False)
        with self.subTest(car=car):
          self.assertEqual(CarControllerParams(cp).STEER_MAX, 270)

  def test_bsm_does_not_enable_angle_control(self):
    for car in (CAR.KIA_PV5, CAR.KIA_EV6):
      for bsm in (False, True):
        fingerprint = gen_empty_fingerprint()
        if bsm:
          fingerprint[0][0x1BA] = 32
        cp = CarInterface.get_params(car, fingerprint, [], False, False, False)
        self.assertEqual(bool(cp.flags & HyundaiFlags.HAS_BSM), bsm)
        self.assertEqual(cp.steerControlType == structs.CarParams.SteerControlType.angle, car == CAR.KIA_PV5)

  def test_mass_normalized_model_matches_safety(self):
    cp = CarInterface.get_params(CAR.KIA_PV5, gen_empty_fingerprint(), [], False, False, False)
    self.assertAlmostEqual(calc_slip_factor(VehicleModel(cp)), -0.0008615808609487565, places=9)

  def test_one_cluster_source(self):
    for car in (CAR.KIA_PV5, CAR.KIA_EV6, CAR.HYUNDAI_SONATA_HEV_2024):
      fingerprint = gen_empty_fingerprint()
      cp = CarInterface.get_params(car, fingerprint, [], False, False, False)
      cp_sp = CarInterface.get_params_sp(cp, car, fingerprint, [], False, False, False)
      ci = CarInterface(cp, cp_sp)
      for frame in range(10):
        ci.update([])
        _, sends = ci.apply(structs.CarControl().as_reader(), structs.CarControlSP(), frame * 10_000_000)
        ids = [(addr, bus) for addr, _, bus in sends]
        self.assertEqual(len(ids), len(set(ids)))
        self.assertEqual((0xCB, 0) in ids, car == CAR.KIA_PV5)
        if car == CAR.KIA_PV5:
          self.assertNotIn((0x12A, 0), ids)
          self.assertNotIn((0x1E0, 0), ids)

  def test_cluster_does_not_modify_camera_state(self):
    cp = CarInterface.get_params(CAR.KIA_PV5, gen_empty_fingerprint(), [], False, False, False)
    can = CanBus(cp)
    parser = CANParser("hyundai_canfd_generated", [], can.CAM)
    msg_161 = dict(parser.vl["CCNC_0x161"])
    msg_162 = dict(parser.vl["CCNC_0x162"])
    msg_1b5 = dict(parser.vl["FR_CMR_03_50ms"])
    original = (msg_161.copy(), msg_162.copy(), msg_1b5.copy())
    cc = structs.CarControl()
    out = structs.CarState()
    packer = CANPacker("hyundai_canfd_generated")
    for long, enabled, blinker, metric in itertools.product((False, True), repeat=4):
      sends = create_pleos_connect_cluster(packer, can, long, enabled, cc.hudControl, blinker, False,
                                           msg_161, msg_162, msg_1b5, metric, out, enabled, enabled, blinker)
      self.assertEqual([addr for addr, _, _ in sends], [0x161, 0x162])
      self.assertEqual((msg_161, msg_162, msg_1b5), original)

  def test_lane_change_display(self):
    for direction, relabel_at in itertools.product(("left", "right"), (13, 10)):
      near, far = ("LEFT", "RIGHT") if direction == "left" else ("RIGHT", "LEFT")
      display = LaneChangeDisplay()
      frames = [display.update(direction, msg) for msg in lane_change_drive(direction, relabel_at)]
      crossing = next(i for i, f in enumerate(frames) if f["LANE_HIGHLIGHT"])
      pre, post = frames[:crossing], frames[crossing:]
      with self.subTest(direction=direction, relabel_at=relabel_at):
        self.assertEqual(crossing, 10)  # center on the line, or relabeled first when frames skip the zero crossing
        # starts centered with the target lane filled and the line to cross dashed
        self.assertEqual((pre[0][f"LANE_{near}"], pre[0][f"LANELINE_{near}"], pre[0][f"LANELINE_{far}"]), (1, 8, 2))
        self.assertEqual((pre[0][f"LANELINE_{near}_POSITION"], pre[0][f"LANELINE_{far}_POSITION"]), (15, 15))
        # the car only moves towards the crossed line, then away from it, through camera noise, sign flips and relabeling
        near_pos = [f[f"LANELINE_{near}_POSITION"] for f in pre]
        crossed_pos = [f[f"LANELINE_{far}_POSITION"] for f in post]
        self.assertEqual(near_pos, sorted(near_pos, reverse=True))
        self.assertEqual(crossed_pos, sorted(crossed_pos))
        self.assertLess(crossed_pos[0], LANE_CHANGE_NEAR)
        for f in pre:
          close = f[f"LANELINE_{near}_POSITION"] <= LANE_CHANGE_NEAR
          # close to the crossed line it turns green and the far line hides, waiting under the car
          self.assertEqual(f[f"LANELINE_{near}"], 10 if close else 8)
          self.assertEqual(f[f"LANELINE_{far}"], 1 if close else 2)
          self.assertEqual(f[f"LANELINE_{far}_POSITION"], f[f"LANELINE_{near}_POSITION"] if close else 30 - f[f"LANELINE_{near}_POSITION"])
        for i, f in enumerate(post):
          # the crossed line stays green on the other side, the new lane takes the fill and shows its outer line once settled
          self.assertEqual((f[f"LANE_{near}"], f["LANE_HIGHLIGHT"], f[f"LANELINE_{far}"]), (0, 1, 10))
          self.assertEqual(f[f"LANELINE_{near}"], 1 if i < LANE_CHANGE_REVEAL_FRAMES else 2)
          self.assertEqual(f[f"LANELINE_{near}_POSITION"], 30 - f[f"LANELINE_{far}_POSITION"])
      # stopping the lane change resets the display
      self.assertIsNone(display.update(None, lane_msg(-1.7, 1.7)))
      self.assertEqual(display.update(direction, lane_msg(-1.7, 1.7))["LANE_HIGHLIGHT"], 0)

  def test_lane_change_display_without_lanes(self):
    display = LaneChangeDisplay()
    fields = display.update("left", lane_msg(0., 0., quality=0))
    self.assertEqual((fields["LANELINE_LEFT_POSITION"], fields["LANELINE_RIGHT_POSITION"], fields["LANE_LEFT"]), (15, 15, 1))

  def test_not_ccnc(self):
    # PV5 carries 0x161/0x162 like CCNC cars, but must not take sunnypilot's torque steering CCNC path
    fingerprint = gen_empty_fingerprint()
    for bus in (0, 2):
      fingerprint[bus].update({0x161: 32, 0x162: 32})
    cp = CarInterface.get_params(CAR.KIA_PV5, fingerprint, [], True, False, False)
    cp_sp = CarInterface.get_params_sp(cp, CAR.KIA_PV5, fingerprint, [], True, False, False)
    self.assertFalse(cp_sp.flags & HyundaiFlagsSP.CCNC)
    self.assertFalse(cp_sp.safetyParam & HyundaiSafetyFlagsSP.CCNC)
    self.assertFalse(cp.alphaLongitudinalAvailable)
    self.assertFalse(cp.openpilotLongitudinalControl)
    self.assertTrue(cp.pcmCruise)

  def test_mdps_mirrors_camera_request(self):
    cp = CarInterface.get_params(CAR.KIA_PV5, gen_empty_fingerprint(), [], False, False, False)
    can = CanBus(cp)
    packer = CANPacker("hyundai_canfd_generated")
    parser = CANParser("hyundai_canfd_generated", [], can.CAM)
    parser.vl["MDPS"]
    mdps = {"COUNTER": 7, "MDPS_ADASAciActvSta_Lv2": 2, "MDPS_StrTqSnsrVal": 25, "MDPS_EstStrAnglVal": 1.7, "MDPS_LkaToiUnblSta": 0}
    for request in (0, 1, 2):
      addr, dat, bus = create_pleos_connect_mdps(packer, can, mdps, {"ADAS_ActvACILvl2Sta": request})
      self.assertEqual((addr, bus), (0xEA, can.CAM))
      parser.update((request + 1, [(addr, dat, bus)]))
      out = parser.vl["MDPS"]
      self.assertEqual(out["MDPS_ADASAciActvSta_Lv2"], request)
      self.assertEqual((out["COUNTER"], out["MDPS_StrTqSnsrVal"], out["MDPS_LkaToiUnblSta"]), (7, 25, 0))
      self.assertAlmostEqual(out["MDPS_EstStrAnglVal"], 1.7, places=3)

  def test_forwards_once_per_source_frame(self):
    fingerprint = gen_empty_fingerprint()
    cp = CarInterface.get_params(CAR.KIA_PV5, fingerprint, [], False, False, False)
    cp_sp = CarInterface.get_params_sp(cp, CAR.KIA_PV5, fingerprint, [], False, False, False)
    can = CanBus(cp)
    ci = CarInterface(cp, cp_sp)
    packer = CANPacker("hyundai_canfd_generated")

    sent = {0xEA: [], 0x161: [], 0x162: []}
    mdps_counter = hud_counter = 0
    for frame in range(100):
      t = frame * 10_000_000
      # MDPS drops a frame every 10, HUD is sent at 20 Hz
      if frame % 10 != 9:
        mdps_counter = (mdps_counter + 1) % 256
      if frame % 5 == 0:
        hud_counter = (hud_counter + 1) % 256
      msgs = [
        packer.make_can_msg("MDPS", can.ECAN, {"COUNTER": mdps_counter, "MDPS_ADASAciActvSta_Lv2": 1}),
        packer.make_can_msg("LFA_ALT", can.CAM, {"ADAS_ActvACILvl2Sta": 2}),
        packer.make_can_msg("CCNC_0x161", can.CAM, {"COUNTER": hud_counter}),
        packer.make_can_msg("CCNC_0x162", can.CAM, {"COUNTER": hud_counter}),
      ]
      ci.update([(t, msgs)])
      _, sends = ci.apply(structs.CarControl().as_reader(), structs.CarControlSP(), t)
      for addr, dat, bus in sends:
        if addr in sent and bus == (can.CAM if addr == 0xEA else can.ECAN):
          sent[addr].append(packer.dbc.addr_to_msg[addr].sigs["COUNTER"].get_raw_value(dat))

    # one frame per source counter, without duplicates or gaps. the first seen frame may only seed the counter
    for addr, last in ((0xEA, mdps_counter), (0x161, hud_counter), (0x162, hud_counter)):
      self.assertLessEqual(sent[addr][0], 2)
      self.assertEqual(sent[addr], list(range(sent[addr][0], last + 1)))

  def test_ldw_toggle(self):
    fingerprint = gen_empty_fingerprint()
    cp = CarInterface.get_params(CAR.KIA_PV5, fingerprint, [], False, False, False)
    for value, expected in ((True, True), (False, False), (None, False)):
      cp_sp = CarInterface.get_params_sp(cp, CAR.KIA_PV5, fingerprint, [], False, False, False)
      setup_interfaces(CarInterface, cp, cp_sp, [] if value is None else [{"IsLdwEnabled": value}])
      self.assertEqual(bool(cp_sp.flags & HyundaiFlagsSP.LDW_ENABLED), expected)

  def test_camera_ldws_passthrough(self):
    # stock LDWS: LKA icon standby/active, departing lane line flashes 4, steering wheel vibrates
    cp = CarInterface.get_params(CAR.KIA_PV5, gen_empty_fingerprint(), [], False, False, False)
    can = CanBus(cp)
    packer = CANPacker("hyundai_canfd_generated")
    parser = CANParser("hyundai_canfd_generated", [], can.ECAN)
    parser.vl["CCNC_0x161"]
    parser.vl["CCNC_0x162"]
    hud = structs.CarControl().hudControl
    msg_1b5 = lane_msg(0, 0, quality=0)
    for i, (ldw, lfa_icon, cam_lka_icon) in enumerate(itertools.product((False, True), (False, True), (0, 1, 3, 4))):
      msg_161 = {"LKA_ICON": cam_lka_icon, "LANELINE_LEFT": 8, "LANELINE_RIGHT": 4, "ALERTS_2": 0, "ALERTS_3": 0, "ALERTS_5": 0,
                 "SOUNDS_4": 0, "LFA_ICON": 2}
      sends = create_pleos_connect_cluster(packer, can, False, lfa_icon, hud, False, False, msg_161, {"VIBRATE": 1}, msg_1b5,
                                           True, structs.CarState(), False, lfa_icon, ldw)
      parser.update((i + 1, [(addr, dat, bus) for addr, dat, bus in sends]))
      with self.subTest(ldw=ldw, lfa_icon=lfa_icon, cam_lka_icon=cam_lka_icon):
        # icon follows the toggle, keeping the camera's standby/active state
        expected_icon = (cam_lka_icon if cam_lka_icon in (3, 4) else 3) if ldw else 0
        self.assertEqual(parser.vl["CCNC_0x161"]["LKA_ICON"], expected_icon)
        self.assertEqual(parser.vl["CCNC_0x161"]["LANELINE_RIGHT"] == 4, ldw)
        self.assertNotEqual(parser.vl["CCNC_0x161"]["LANELINE_LEFT"], 4)
        # camera vibration is never dropped, it also carries forward collision warnings
        self.assertEqual(parser.vl["CCNC_0x162"]["VIBRATE"], 1)

  def test_curvature_can_encoding(self):
    # stock camera encoding: raw = step for left curves, 15 + |step| for right curves
    cp = CarInterface.get_params(CAR.KIA_PV5, gen_empty_fingerprint(), [], False, False, False)
    can = CanBus(cp)
    packer = CANPacker("hyundai_canfd_generated")
    sig = packer.dbc.name_to_msg["CCNC_0x161"].sigs["LANELINE_CURVATURE"]
    for step in range(-15, 16):
      _, dat, _ = packer.make_can_msg("CCNC_0x161", can.ECAN, {"LANELINE_CURVATURE": curvature_to_can(step)})
      self.assertEqual(sig.get_raw_value(dat), step if step >= 0 else 15 - step)

  def test_curvature_step(self):
    self.assertEqual(curvature_step(0.0), 0)
    # each breakpoint starts the next step, positive curvature (right curve) gives negative steps.
    # starting from straight, the step only moves once past the breakpoint plus hysteresis
    for i, bp in enumerate(CURVATURE_STEP_BP):
      self.assertEqual(curvature_step(-(bp + 0.09) * 1e-3), i + 1)
      self.assertEqual(curvature_step((bp + 0.09) * 1e-3), -(i + 1))
    self.assertEqual(curvature_step(-(CURVATURE_STEP_BP[0] + 0.05) * 1e-3), 0)
    self.assertEqual(curvature_step(-1e-2), 15)
    # hysteresis holds the last step near a breakpoint, but not across a direction change
    bp = CURVATURE_STEP_BP[2] * 1e-3
    self.assertEqual(curvature_step(-(bp - 0.05e-3), 3), 3)
    self.assertEqual(curvature_step(-(bp - 0.2e-3), 3), 2)
    self.assertEqual(curvature_step(0.1e-3, 1), 0)

  def test_lane_curvature(self):
    msg = {"Info_LftLnCvtrVal": 1e-3, "Info_RtLnCvtrVal": 2e-3, "Info_LftLnQualSta": 3, "Info_RtLnQualSta": 3}
    self.assertAlmostEqual(lane_curvature(msg), 1.5e-3)
    self.assertAlmostEqual(lane_curvature({**msg, "Info_RtLnQualSta": 1}), 1e-3)
    self.assertIsNone(lane_curvature({**msg, "Info_LftLnQualSta": 0, "Info_RtLnQualSta": 1}))

  def test_cluster_line_type_and_curvature(self):
    cp = CarInterface.get_params(CAR.KIA_PV5, gen_empty_fingerprint(), [], False, False, False)
    can = CanBus(cp)
    packer = CANPacker("hyundai_canfd_generated")
    parser = CANParser("hyundai_canfd_generated", [], can.ECAN)
    parser.vl["CCNC_0x161"]
    hud = structs.CarControl().hudControl
    msg_1b5 = lane_msg(0, 0, quality=0)
    hud.leftLaneVisible = hud.rightLaneVisible = True  # always set by controlsd
    sig = packer.dbc.name_to_msg["CCNC_0x161"].sigs["LANELINE_CURVATURE"]
    lane_change = {"LANELINE_LEFT": 10, "LANELINE_RIGHT": 1, "LANE_LEFT": 1}
    cases = [
      # lfa_icon, blinker, camera (left, right), curvature step, lane change -> expected (left, right), raw curvature
      (True, False, (8, 2), -3, None, (8, 2), 18),         # camera dashed/solid kept, right curve
      (True, False, (0, 0), 4, None, (2, 2), 4),           # no camera line type: solid
      (True, True, (8, 8), 4, None, (8, 8), 4),            # blinker without a lane change (blocked): normal lines
      (True, True, (8, 8), 4, lane_change, (10, 1), 0),    # lane change display, straight
      (False, True, (8, 2), 4, lane_change, (0, 0), 0),    # not lateral active: hidden, straight, no lane change display
    ]
    for i, (lfa_icon, blinker, cam, step, lc, expected, raw_curv) in enumerate(cases):
      msg_161 = {"LKA_ICON": 3, "LANELINE_LEFT": cam[0], "LANELINE_RIGHT": cam[1], "ALERTS_2": 0, "ALERTS_3": 0, "ALERTS_5": 0,
                 "SOUNDS_4": 0, "LFA_ICON": 0}
      sends = create_pleos_connect_cluster(packer, can, False, lfa_icon, hud, blinker, False, msg_161, {}, msg_1b5,
                                           True, structs.CarState(), False, lfa_icon, True, step, lc)
      addr, dat, bus = sends[0]
      parser.update((i + 1, [(addr, dat, bus)]))
      with self.subTest(case=i):
        self.assertEqual((parser.vl["CCNC_0x161"]["LANELINE_LEFT"], parser.vl["CCNC_0x161"]["LANELINE_RIGHT"]), expected)
        self.assertEqual(sig.get_raw_value(dat), raw_curv)
        if lc is not None and lfa_icon:
          self.assertEqual(parser.vl["CCNC_0x161"]["LANE_LEFT"], 1)

  def test_cluster_chevrons_and_lane_fill_length(self):
    cp = CarInterface.get_params(CAR.KIA_PV5, gen_empty_fingerprint(), [], False, False, False)
    can = CanBus(cp)
    packer = CANPacker("hyundai_canfd_generated")
    parser = CANParser("hyundai_canfd_generated", [], can.ECAN)
    parser.vl["CCNC_0x161"]
    hud = structs.CarControl().hudControl
    msg_161 = {"LKA_ICON": 3, "LANELINE_LEFT": 0, "LANELINE_RIGHT": 0, "ALERTS_2": 0, "ALERTS_3": 0, "ALERTS_5": 0, "SOUNDS_4": 0, "LFA_ICON": 0}
    for i, (blindspot, lead) in enumerate(itertools.product((False, True), (0., 42.5))):
      out = structs.CarState()
      out.leftBlindspot = blindspot
      sends = create_pleos_connect_cluster(packer, can, False, True, hud, True, False, msg_161, {}, lane_msg(-1.7, 1.7, lead=lead),
                                           True, out, False, True, True)
      parser.update((i + 1, [sends[0]]))
      with self.subTest(blindspot=blindspot, lead=lead):
        # no chevrons into an occupied lane, like stock
        self.assertEqual(parser.vl["CCNC_0x161"]["LCA_LEFT_ARROW"], 0 if blindspot else 2)
        # the ego lane fill reaches the lead vehicle, or as far as possible without one
        self.assertAlmostEqual(parser.vl["CCNC_0x161"]["LANE_HIGHLIGHT_DISTANCE"], lead if lead else 204.7, places=1)
