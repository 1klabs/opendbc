#!/usr/bin/env python3
from opendbc.testing import parameterized_class
import unittest

from opendbc.car.hyundai.values import HyundaiSafetyFlags
from opendbc.sunnypilot.car.hyundai.values import HyundaiSafetyFlagsSP
from opendbc.car.structs import CarParams
import opendbc.safety.tests.common as common
from opendbc.safety.tests.common import CANPackerSafety
from opendbc.safety.tests.libsafety import libsafety_py
from opendbc.safety.tests.hyundai_common import HyundaiButtonBase, HyundaiLongitudinalBase

# All combinations of radar/camera-SCC and gas/hybrid/EV cars
ALL_GAS_EV_HYBRID_COMBOS = [
  # Radar SCC
  {"GAS_MSG": ("ACCELERATOR_BRAKE_ALT", "ACCELERATOR_PEDAL_PRESSED"), "SCC_BUS": 0, "EXTRA_SAFETY_PARAM": 0},
  {"GAS_MSG": ("ACCELERATOR", "ACCELERATOR_PEDAL"), "SCC_BUS": 0, "EXTRA_SAFETY_PARAM": HyundaiSafetyFlags.EV_GAS},
  {"GAS_MSG": ("ACCELERATOR_ALT", "ACCELERATOR_PEDAL"), "SCC_BUS": 0, "EXTRA_SAFETY_PARAM": HyundaiSafetyFlags.HYBRID_GAS},
  # Camera SCC
  {"GAS_MSG": ("ACCELERATOR_BRAKE_ALT", "ACCELERATOR_PEDAL_PRESSED"), "SCC_BUS": 2, "EXTRA_SAFETY_PARAM": HyundaiSafetyFlags.CAMERA_SCC},
  {"GAS_MSG": ("ACCELERATOR", "ACCELERATOR_PEDAL"), "SCC_BUS": 2, "EXTRA_SAFETY_PARAM": HyundaiSafetyFlags.EV_GAS | HyundaiSafetyFlags.CAMERA_SCC},
  {"GAS_MSG": ("ACCELERATOR_ALT", "ACCELERATOR_PEDAL"), "SCC_BUS": 2, "EXTRA_SAFETY_PARAM": HyundaiSafetyFlags.HYBRID_GAS | HyundaiSafetyFlags.CAMERA_SCC},
]

class TestHyundaiCanfdBase(HyundaiButtonBase, common.CarSafetyTest, common.DriverTorqueSteeringSafetyTest, common.SteerRequestCutSafetyTest):

  DBC = "hyundai_canfd_generated"
  SAFETY_MODEL = CarParams.SafetyModel.hyundaiCanfd

  TX_MSGS = [[0x50, 0], [0x1CF, 1], [0x2A4, 0]]
  STANDSTILL_THRESHOLD = 12  # 0.375 kph
  FWD_BLACKLISTED_ADDRS = {2: [0x50, 0x2a4]}

  MAX_RATE_UP = 2
  MAX_RATE_DOWN = 3
  MAX_TORQUE_LOOKUP = [0], [270]

  MAX_RT_DELTA = 112

  DRIVER_TORQUE_ALLOWANCE = 250
  DRIVER_TORQUE_FACTOR = 2

  # Safety around steering req bit
  MIN_VALID_STEERING_FRAMES = 89
  MAX_INVALID_STEERING_FRAMES = 2

  PT_BUS = 0
  SCC_BUS = 2
  STEER_BUS = 0
  STEER_MSG = ""
  GAS_MSG = ("", "")
  BUTTONS_TX_BUS = 1

  def _torque_driver_msg(self, torque):
    values = {"MDPS_StrTqSnsrVal": torque}
    return self.packer.make_can_msg_safety("MDPS", self.PT_BUS, values)

  def _torque_cmd_msg(self, torque, steer_req=1):
    values = {"StrTqReqVal": torque, "ActToiSta": steer_req}
    return self.packer.make_can_msg_safety(self.STEER_MSG, self.STEER_BUS, values)

  def _speed_msg(self, speed):
    values = {f"WHL_Spd{pos}Val": speed * 0.03125 for pos in ["FL", "FR", "RL", "RR"]}
    return self.packer.make_can_msg_safety("WHEEL_SPEEDS", self.PT_BUS, values)

  def _user_brake_msg(self, brake):
    values = {"DriverBraking": brake}
    return self.packer.make_can_msg_safety("TCS", self.PT_BUS, values)

  def _user_gas_msg(self, gas):
    values = {self.GAS_MSG[1]: gas}
    return self.packer.make_can_msg_safety(self.GAS_MSG[0], self.PT_BUS, values)

  def _pcm_status_msg(self, enable):
    values = {"ACCMode": 1 if enable else 0}
    return self.packer.make_can_msg_safety("SCC_CONTROL", self.SCC_BUS, values)

  def _button_msg(self, buttons, main_button=0, bus=None):
    if bus is None:
      bus = self.PT_BUS
    values = {
      "CRUISE_BUTTONS": buttons,
      "ADAPTIVE_CRUISE_MAIN_BTN": main_button,
    }
    return self.packer.make_can_msg_safety("CRUISE_BUTTONS", bus, values)

  def _acc_state_msg(self, enable):
    values = {"MainMode_ACC": enable}
    return self.packer.make_can_msg_safety("SCC_CONTROL", self.SCC_BUS, values)

  def _lkas_button_msg(self, enabled):
    values = {"LDA_BTN": enabled}
    return self.packer.make_can_msg_safety("CRUISE_BUTTONS", self.PT_BUS, values)

  def _main_cruise_button_msg(self, enabled):
    return self._button_msg(0, enabled)
class TestHyundaiCanfdLFASteeringBase(TestHyundaiCanfdBase):

  TX_MSGS = [[0x12A, 0], [0x1A0, 1], [0x1CF, 0], [0x1E0, 0]]
  RELAY_MALFUNCTION_ADDRS = {0: (0x12A, 0x1E0)}  # LFA, LFAHDA_CLUSTER
  FWD_BLACKLISTED_ADDRS = {2: [0x12A, 0x1E0]}

  STEER_MSG = "LFA"
  BUTTONS_TX_BUS = 2
  EXTRA_SAFETY_PARAM: int

  @classmethod
  def setUpClass(cls):
    super().setUpClass()
    if cls.__name__ in ("TestHyundaiCanfdLFASteering", "TestHyundaiCanfdLFASteeringAltButtons"):
      cls.packer = None
      raise unittest.SkipTest

  @property
  def SAFETY_PARAM(self):
    return self.EXTRA_SAFETY_PARAM
@parameterized_class(ALL_GAS_EV_HYBRID_COMBOS)
class TestHyundaiCanfdLFASteering(TestHyundaiCanfdLFASteeringBase):
  pass
class TestHyundaiCanfdLFASteeringAltButtonsBase(TestHyundaiCanfdLFASteeringBase):

  EXTRA_SAFETY_PARAM: int

  @property
  def SAFETY_PARAM(self):
    return HyundaiSafetyFlags.CANFD_ALT_BUTTONS | self.EXTRA_SAFETY_PARAM

  def _button_msg(self, buttons, main_button=0, bus=1):
    values = {
      "CRUISE_BUTTONS": buttons,
      "ADAPTIVE_CRUISE_MAIN_BTN": main_button,
    }
    return self.packer.make_can_msg_safety("CRUISE_BUTTONS_ALT", self.PT_BUS, values)

  def _lkas_button_msg(self, enabled):
    values = {"LDA_BTN": enabled}
    return self.packer.make_can_msg_safety("CRUISE_BUTTONS_ALT", self.PT_BUS, values)

  def _acc_cancel_msg(self, cancel, accel=0):
    values = {"ACCMode": 4 if cancel else 0, "aReqRaw": accel, "aReqValue": accel}
    return self.packer.make_can_msg_safety("SCC_CONTROL", self.PT_BUS, values)

  def test_button_sends(self):
    """
      No button send allowed with alt buttons.
    """
    for enabled in (True, False):
      for btn in range(8):
        self.safety.set_controls_allowed(enabled)
        self.assertFalse(self._tx(self._button_msg(btn)))

  def test_acc_cancel(self):
    # FIXME: the CANFD_ALT_BUTTONS cars are the only ones that use SCC_CONTROL to cancel, why can't we use buttons?
    for enabled in (True, False):
      self.safety.set_controls_allowed(enabled)
      self.assertTrue(self._tx(self._acc_cancel_msg(True)))
      self.assertFalse(self._tx(self._acc_cancel_msg(True, accel=1)))
      self.assertFalse(self._tx(self._acc_cancel_msg(False)))


class TestHyundaiCanfdPleosConnectButtons(unittest.TestCase):
  def setUp(self):
    self.packer = CANPackerSafety("hyundai_canfd_generated")
    self.safety = libsafety_py.libsafety
    self.safety.set_current_safety_param_sp(0)
    safety_param = HyundaiSafetyFlags.EV_GAS | HyundaiSafetyFlags.CAMERA_SCC | HyundaiSafetyFlags.CCNC | \
                   HyundaiSafetyFlags.CANFD_ALT_BUTTONS | HyundaiSafetyFlags.PLEOS_CONNECT_PV5
    self.safety.set_safety_hooks(CarParams.SafetyModel.hyundaiCanfd, safety_param)
    self.safety.init_tests()

  def _button_msg(self, **values):
    return self.packer.make_can_msg_safety("PLEOS_CONNECT_BUTTONS", 0, values)

  def _accelerator_msg(self, counter):
    return self.packer.make_can_msg_safety("ACCELERATOR", 0, {"COUNTER": counter})

  def test_accelerator_counter_increments_by_two(self):
    for counter in range(2, 22, 2):
      self.assertTrue(self.safety.safety_rx_hook(self._accelerator_msg(counter)))

    for i, counter in enumerate(range(23, 28)):
      self.assertEqual(i < 4, self.safety.safety_rx_hook(self._accelerator_msg(counter)))

  def test_lfa_button_enables_mads_lateral(self):
    self.safety.set_mads_params(True, False, False)

    for pressed, lateral_allowed in ((False, False), (True, True), (False, True)):
      self.assertTrue(self.safety.safety_rx_hook(self._button_msg(LDA_BTN=pressed)))
      self.assertEqual(pressed, self.safety.get_mads_button_press())
      self.assertEqual(lateral_allowed, self.safety.get_controls_allowed_lateral())

  def test_cruise_buttons_message_is_accepted(self):
    for button in range(4):
      self.assertTrue(self.safety.safety_rx_hook(self._button_msg(CRUISE_BUTTONS=button)))

    self.assertTrue(self.safety.safety_rx_hook(self._button_msg(PAUSE_RESUME_BTN=1)))
    self.assertTrue(self.safety.safety_rx_hook(self._button_msg(ADAPTIVE_CRUISE_MAIN_BTN=1)))


class TestHyundaiCanfdPv5AngleSteering(unittest.TestCase):
  def setUp(self):
    self.packer = CANPackerSafety("hyundai_canfd_generated")
    self.safety = libsafety_py.libsafety
    self.safety.set_current_safety_param_sp(0)
    self.safety_param = HyundaiSafetyFlags.EV_GAS | HyundaiSafetyFlags.CAMERA_SCC | HyundaiSafetyFlags.CCNC | \
                        HyundaiSafetyFlags.PLEOS_CONNECT_PV5 | HyundaiSafetyFlags.CANFD_ANGLE_STEERING
    self.safety.set_safety_hooks(CarParams.SafetyModel.hyundaiCanfd, self.safety_param)
    self.safety.init_tests()

    for _ in range(6):
      self.safety.safety_rx_hook(self._angle_meas_msg(0))
      self.safety.safety_rx_hook(self._speed_msg(10))

  def _angle_cmd_msg(self, angle, enabled, gain=0):
    values = {
      "ADAS_ActvACISta": 0,
      "ADAS_ActvACILvl2Sta": 2 if enabled else 1,
      "ADAS_StrAnglReqVal": angle,
      "ADAS_ACIAnglTqRedcGainVal": gain,
      "FCA_ESA_ActvSta": 0,
      "FCA_ESA_TqBstGainVal": 0,
    }
    return self.packer.make_can_msg_safety("LFA_ALT", 0, values)

  def _angle_meas_msg(self, angle):
    return self.packer.make_can_msg_safety("MDPS", 0, {"MDPS_EstStrAnglVal": angle})

  def _speed_msg(self, speed):
    values = {f"WHL_Spd{pos}Val": speed * 0.03125 for pos in ["FL", "FR", "RL", "RR"]}
    return self.packer.make_can_msg_safety("WHEEL_SPEEDS", 0, values)

  def test_angle_command_requires_controls_allowed(self):
    self.safety.set_controls_allowed(False)
    self.assertFalse(self.safety.safety_tx_hook(self._angle_cmd_msg(0, True)))

    self.safety.set_controls_allowed(True)
    self.assertTrue(self.safety.safety_tx_hook(self._angle_cmd_msg(0, True)))

  def test_controller_angle_limits_match_safety(self):
    from opendbc.car import gen_empty_fingerprint
    from opendbc.car.hyundai.interface import CarInterface
    from opendbc.car.hyundai.values import CAR, CarControllerParams
    from opendbc.car.lateral import apply_steer_angle_limits_vm
    from opendbc.car.vehicle_model import VehicleModel

    cp = CarInterface.get_params(CAR.KIA_PV5, gen_empty_fingerprint(), [], False, False, False)
    params = CarControllerParams(cp)
    vm = VehicleModel(cp)
    for speed in (0, 5, 15, 30, 40):
      for direction in (-1, 1):
        self.setUp()
        for _ in range(6):
          self.assertTrue(self.safety.safety_rx_hook(self._speed_msg(speed * 3.6 / 0.03125)))
        self.safety.set_controls_allowed(True)
        last = 0.0
        for frame in range(100):
          self.safety.set_timer(frame * 10_000)
          angle = apply_steer_angle_limits_vm(direction * 360, last, speed, last, True, params, vm)
          self.assertTrue(self.safety.safety_tx_hook(self._angle_cmd_msg(angle, True)), (speed, angle, last))
          self.assertTrue(self.safety.safety_rx_hook(self._angle_meas_msg(angle)))
          last = angle

  def test_brake_disables_angle_control(self):
    self.safety.set_controls_allowed(True)
    brake = self.packer.make_can_msg_safety("TCS", 0, {"DriverBraking": 1})
    self.assertTrue(self.safety.safety_rx_hook(brake))
    self.assertFalse(self.safety.safety_tx_hook(self._angle_cmd_msg(0, True)))

  def test_inactive_command_tracks_measured_angle(self):
    self.assertTrue(self.safety.safety_tx_hook(self._angle_cmd_msg(0, False)))
    self.assertFalse(self.safety.safety_tx_hook(self._angle_cmd_msg(10, False)))

  def test_torque_reduction_gain(self):
    self.safety.set_controls_allowed(True)
    self.assertTrue(self.safety.safety_tx_hook(self._angle_cmd_msg(0, True, gain=1.0)))
    self.assertFalse(self.safety.safety_tx_hook(self._angle_cmd_msg(0, True, gain=1.004)))
    self.assertFalse(self.safety.safety_tx_hook(self._angle_cmd_msg(0, False, gain=0.5)))

  def test_angle_command_maximum(self):
    self.safety.set_controls_allowed(True)
    self.safety.set_desired_angle_last(3600)
    self.assertTrue(self.safety.safety_tx_hook(self._angle_cmd_msg(360, True)))

    self.safety.set_desired_angle_last(3610)
    self.assertFalse(self.safety.safety_tx_hook(self._angle_cmd_msg(361, True)))

  def test_angle_message_blocked_without_angle_flag(self):
    self.safety.set_safety_hooks(CarParams.SafetyModel.hyundaiCanfd,
                                 self.safety_param & ~HyundaiSafetyFlags.CANFD_ANGLE_STEERING)
    self.safety.init_tests()
    self.assertFalse(self.safety.safety_tx_hook(self._angle_cmd_msg(0, False)))

  def test_stock_angle_message_is_blocked_from_forwarding(self):
    self.assertEqual(-1, self.safety.safety_fwd_hook(2, 0xCB))

  def test_torque_steering_hud_and_standard_buttons_are_not_whitelisted(self):
    torque_steering = self.packer.make_can_msg_safety("LFA", 0, {})
    standard_hud = self.packer.make_can_msg_safety("LFAHDA_CLUSTER", 0, {})
    standard_buttons = self.packer.make_can_msg_safety("CRUISE_BUTTONS", 2, {})

    self.assertFalse(self.safety.safety_tx_hook(torque_steering))
    self.assertFalse(self.safety.safety_tx_hook(standard_hud))
    self.assertFalse(self.safety.safety_tx_hook(standard_buttons))


@parameterized_class(ALL_GAS_EV_HYBRID_COMBOS)
class TestHyundaiCanfdLFASteeringAltButtons(TestHyundaiCanfdLFASteeringAltButtonsBase):
  pass
class TestHyundaiCanfdLKASteeringEV(TestHyundaiCanfdBase):

  SAFETY_PARAM = HyundaiSafetyFlags.CANFD_LKA_STEER_MSG | HyundaiSafetyFlags.EV_GAS

  TX_MSGS = [[0x50, 0], [0x1CF, 1], [0x2A4, 0]]
  RELAY_MALFUNCTION_ADDRS = {0: (0x50, 0x2a4)}  # LKAS, CAM_0x2A4
  FWD_BLACKLISTED_ADDRS = {2: [0x50, 0x2a4]}

  PT_BUS = 1
  SCC_BUS = 1
  STEER_MSG = "LKAS"
  GAS_MSG = ("ACCELERATOR", "ACCELERATOR_PEDAL")
# TODO: Handle ICE and HEV configurations once we see cars that use the new messages
class TestHyundaiCanfdLKASteeringAltEV(TestHyundaiCanfdBase):

  SAFETY_PARAM = HyundaiSafetyFlags.CANFD_LKA_STEER_MSG | HyundaiSafetyFlags.EV_GAS | HyundaiSafetyFlags.CANFD_LKA_STEER_MSG_ALT

  TX_MSGS = [[0x110, 0], [0x1CF, 1], [0x362, 0]]
  RELAY_MALFUNCTION_ADDRS = {0: (0x110, 0x362)}  # LKAS_ALT, CAM_0x362
  FWD_BLACKLISTED_ADDRS = {2: [0x110, 0x362]}

  PT_BUS = 1
  SCC_BUS = 1
  STEER_MSG = "LKAS_ALT"
  GAS_MSG = ("ACCELERATOR", "ACCELERATOR_PEDAL")
class TestHyundaiCanfdLKASteeringLongEV(HyundaiLongitudinalBase, TestHyundaiCanfdLKASteeringEV):

  SAFETY_PARAM = HyundaiSafetyFlags.CANFD_LKA_STEER_MSG | HyundaiSafetyFlags.LONG | HyundaiSafetyFlags.EV_GAS

  TX_MSGS = [[0x50, 0], [0x1CF, 1], [0x2A4, 0], [0x51, 0], [0x730, 1], [0x12a, 1], [0x160, 1],
             [0x1e0, 1], [0x1a0, 1], [0x1ea, 1], [0x200, 1], [0x345, 1], [0x1da, 1]]

  RELAY_MALFUNCTION_ADDRS = {0: (0x50, 0x2a4), 1: (0x1a0,)}  # LKAS, CAM_0x2A4, SCC_CONTROL

  DISABLED_ECU_UDS_MSG = (0x730, 1)
  DISABLED_ECU_ACTUATION_MSG = (0x1a0, 1)

  STEER_MSG = "LFA"
  GAS_MSG = ("ACCELERATOR", "ACCELERATOR_PEDAL")
  STEER_BUS = 1

  def _accel_msg(self, accel, aeb_req=False, aeb_decel=0):
    values = {
      "aReqRaw": accel,
      "aReqValue": accel,
    }
    return self.packer.make_can_msg_safety("SCC_CONTROL", self.PT_BUS, values)

  def _tx_acc_state_msg(self, enable):
    values = {"MainMode_ACC": enable}
    return self.packer.make_can_msg_safety("SCC_CONTROL", self.PT_BUS, values)
# Tests longitudinal for ICE, hybrid, EV cars with LFA steering
class TestHyundaiCanfdLFASteeringLongBase(HyundaiLongitudinalBase, TestHyundaiCanfdLFASteeringBase):

  FWD_BLACKLISTED_ADDRS = {2: [0x12a, 0x1e0, 0x1a0, 0x160]}

  RELAY_MALFUNCTION_ADDRS = {0: (0x12A, 0x1E0, 0x1a0, 0x160)}  # LFA, LFAHDA_CLUSTER, SCC_CONTROL, ADRV_0x160

  DISABLED_ECU_UDS_MSG = (0x7D0, 0)
  DISABLED_ECU_ACTUATION_MSG = (0x1a0, 0)

  @property
  def SAFETY_PARAM(self):
    return HyundaiSafetyFlags.LONG | self.EXTRA_SAFETY_PARAM

  def _accel_msg(self, accel, aeb_req=False, aeb_decel=0):
    values = {
      "aReqRaw": accel,
      "aReqValue": accel,
    }
    return self.packer.make_can_msg_safety("SCC_CONTROL", self.PT_BUS, values)

  def _tx_acc_state_msg(self, enable):
    values = {"MainMode_ACC": enable}
    return self.packer.make_can_msg_safety("SCC_CONTROL", self.PT_BUS, values)

  def test_tester_present_allowed(self, ecu_disable: bool = True):
    super().test_tester_present_allowed(ecu_disable=not self.EXTRA_SAFETY_PARAM & HyundaiSafetyFlags.CAMERA_SCC)
@parameterized_class(ALL_GAS_EV_HYBRID_COMBOS)
class TestHyundaiCanfdLFASteeringLong(TestHyundaiCanfdLFASteeringLongBase):
  @classmethod
  def setUpClass(cls):
    super().setUpClass()
    if cls.__name__ == "TestHyundaiCanfdLFASteeringLong":
      raise unittest.SkipTest
@parameterized_class(ALL_GAS_EV_HYBRID_COMBOS)
class TestHyundaiCanfdLFASteeringLongAltButtons(TestHyundaiCanfdLFASteeringLongBase, TestHyundaiCanfdLFASteeringAltButtonsBase):
  @classmethod
  def setUpClass(cls):
    super().setUpClass()
    if cls.__name__ == "TestHyundaiCanfdLFASteeringLongAltButtons":
      raise unittest.SkipTest

  @property
  def SAFETY_PARAM(self):
    return HyundaiSafetyFlags.LONG | HyundaiSafetyFlags.CANFD_ALT_BUTTONS | self.EXTRA_SAFETY_PARAM

  def test_acc_cancel(self):
    # Alt buttons does not use SCC_CONTROL to cancel if longitudinal
    pass


CAMERA_SCC_COMBOS = [c for c in ALL_GAS_EV_HYBRID_COMBOS if c["EXTRA_SAFETY_PARAM"] & HyundaiSafetyFlags.CAMERA_SCC]


@parameterized_class(CAMERA_SCC_COMBOS)
class TestHyundaiCanfdLFASteeringCCNC(TestHyundaiCanfdLFASteeringBase):

  TX_MSGS = [[0x12A, 0], [0x1E0, 0], [0x1CF, 2], [0x161, 0], [0x162, 0], [0xEA, 2]]
  RELAY_MALFUNCTION_ADDRS = {0: (0x12A, 0x1E0, 0x161, 0x162), 2: (0xEA,)}
  FWD_BLACKLISTED_ADDRS = {0: [0xEA], 2: [0x12A, 0x1E0, 0x161, 0x162]}

  @classmethod
  def setUpClass(cls):
    super().setUpClass()
    if cls.__name__ == "TestHyundaiCanfdLFASteeringCCNC":
      cls.packer = None
      raise unittest.SkipTest

  def setUp(self):
    libsafety_py.libsafety.set_current_safety_param_sp(HyundaiSafetyFlagsSP.CCNC)
    super().setUp()

  def tearDown(self):
    self.safety.set_current_safety_param_sp(0)

  def test_ccnc_tx_msgs(self):
    for msg, bus in (("CCNC_0x161", 0), ("CCNC_0x162", 0), ("MDPS", 2)):
      self.assertTrue(self._tx(self.packer.make_can_msg_safety(msg, bus, {})))


if __name__ == "__main__":
  unittest.main()
