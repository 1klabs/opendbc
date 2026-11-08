import itertools
import unittest

from opendbc.can import CANPacker, CANParser
from opendbc.car import gen_empty_fingerprint, structs
from opendbc.car.hyundai.interface import CarInterface
from opendbc.car.hyundai.hyundaicanfd import CanBus
from opendbc.car.hyundai.values import CAR, CarControllerParams, HyundaiFlags
from opendbc.car.vehicle_model import VehicleModel, calc_slip_factor
from opendbc.sunnypilot.car.hyundai.pleos_connect import create_pleos_connect_cluster, create_pleos_connect_mdps, lane_positions
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

  def test_missing_lane_positions(self):
    self.assertEqual(lane_positions({"Info_LftLnPosVal": 0, "Info_RtLnPosVal": 0,
                                     "Info_LftLnQualSta": 0, "Info_RtLnQualSta": 0}), (15, 15))

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
    msg_1b5 = {"Info_LftLnPosVal": 0, "Info_RtLnPosVal": 0, "Info_LftLnQualSta": 0, "Info_RtLnQualSta": 0}
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
