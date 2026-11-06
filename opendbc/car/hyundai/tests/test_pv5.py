import itertools
import unittest

from opendbc.can import CANPacker, CANParser
from opendbc.car import gen_empty_fingerprint, structs
from opendbc.car.hyundai.interface import CarInterface
from opendbc.car.hyundai.hyundaicanfd import CanBus
from opendbc.car.hyundai.values import CAR, CarControllerParams, HyundaiFlags
from opendbc.car.vehicle_model import VehicleModel, calc_slip_factor
from opendbc.sunnypilot.car.hyundai.pv5 import create_pv5_cluster, lane_positions


class TestPv5Integration(unittest.TestCase):
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
      sends = create_pv5_cluster(packer, can, long, enabled, cc.hudControl, blinker, False,
                                 msg_161, msg_162, msg_1b5, metric, out, enabled, enabled)
      self.assertEqual([addr for addr, _, _ in sends], [0x161, 0x162])
      self.assertEqual((msg_161, msg_162, msg_1b5), original)

  def test_missing_lane_positions(self):
    self.assertEqual(lane_positions({"Info_LftLnPosVal": 0, "Info_RtLnPosVal": 0,
                                     "Info_LftLnQualSta": 0, "Info_RtLnQualSta": 0}), (15, 15))
