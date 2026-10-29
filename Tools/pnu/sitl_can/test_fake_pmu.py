#!/usr/bin/env python3
'''
PNU : tests for the fake PMU's CAN framing, without a vcan interface.

Creating a vcan interface needs root, so these drive FakePMU.handle_frame() with
hand-built SocketCAN frames and capture what it would have transmitted.  What they
cannot cover is the socket itself - binding, filtering and the kernel's RTR delivery -
which is only provable by running against a real vcan; see README.md in this directory.

Run with pytest, or directly:  Tools/pnu/sitl_can/test_fake_pmu.py

AP_FLAKE8_CLEAN
'''

import os
import struct
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import pmu_frames as icd                                                        # noqa: E402
from fake_pmu import FakePMU, CAN_EFF_FLAG, CAN_RTR_FLAG, CAN_ERR_FLAG          # noqa: E402
from fake_pmu import CAN_FRAME_FMT, TakeoffWatcher, is_vehicle_heartbeat        # noqa: E402
from pmu_model import State                                                     # noqa: E402


def make_pmu():
    '''a FakePMU with no socket, recording what it would have sent'''
    pmu = FakePMU('test0')
    pmu.sent = []
    pmu.send = lambda can_id, payload: pmu.sent.append((can_id, payload))
    pmu.log = lambda msg: None
    pmu.model._log = lambda msg: None       # bound before pmu.log was replaced
    return pmu


def frame(can_id, data=b'', rtr=False):
    flags = CAN_EFF_FLAG | (CAN_RTR_FLAG if rtr else 0)
    return struct.pack(CAN_FRAME_FMT, can_id | flags, len(data), data.ljust(8, b'\x00'))


def command(can_id, value, dlc=4):
    return frame(can_id, (struct.pack('<I', value) + b'\x00' * 4)[:dlc])


# -- RTR request / data response ------------------------------------------------------

def test_every_status_rtr_is_answered_on_the_same_id():
    pmu = make_pmu()
    for can_id in icd.STATUS_IDS:
        pmu.handle_frame(frame(can_id, rtr=True))
    assert [can_id for can_id, _ in pmu.sent] == list(icd.STATUS_IDS)
    assert all(len(payload) == icd.STATUS_DLC for _, payload in pmu.sent)


def test_nothing_is_sent_unprompted():
    '''the PMU never transmits on its own - only an RTR produces a frame'''
    pmu = make_pmu()
    for _ in range(100):
        pmu.model.update(0.01)
    assert pmu.sent == []


def test_a_data_frame_on_a_status_id_is_not_a_request():
    pmu = make_pmu()
    pmu.handle_frame(frame(icd.ENGSTS, b'\x00' * 8))
    assert pmu.sent == []


def test_an_rtr_on_an_unknown_id_is_ignored():
    pmu = make_pmu()
    pmu.handle_frame(frame(0x100001F0, rtr=True))
    assert pmu.sent == []


def test_error_frames_are_dropped():
    pmu = make_pmu()
    raw = struct.pack(CAN_FRAME_FMT, icd.ENGSTS | CAN_EFF_FLAG | CAN_RTR_FLAG | CAN_ERR_FLAG, 0, b'\x00' * 8)
    pmu.handle_frame(raw)
    assert pmu.sent == []


# -- the silent trigger ---------------------------------------------------------------

def test_silent_stops_answering_but_still_counts_the_requests():
    pmu = make_pmu()
    pmu.set_silent(True)
    for can_id in icd.STATUS_IDS:
        pmu.handle_frame(frame(can_id, rtr=True))
    assert pmu.sent == []
    assert pmu._rtr_seen == len(icd.STATUS_IDS)
    assert pmu._rtr_answered == 0


def test_silence_is_reversible_and_the_model_keeps_running_through_it():
    pmu = make_pmu()
    pmu.do_command('start')
    pmu.set_silent(True)
    for _ in range(600):
        pmu.model.update(0.01)                  # 6 s with nobody listening
    pmu.set_silent(False)
    pmu.handle_frame(frame(icd.ENGSTS, rtr=True))
    assert len(pmu.sent) == 1
    # the engine carried on through the silence rather than freezing
    assert pmu.model.state in (State.WARMUP, State.GROUND_IDLE)
    rpm = struct.unpack_from('<H', pmu.sent[0][1], 4)[0]
    assert rpm > 1000


# -- commands -------------------------------------------------------------------------

def test_engonoff_start_and_stop_drive_the_model():
    pmu = make_pmu()
    pmu.handle_frame(command(icd.ENGONOFF, 1))
    assert pmu.model.state == State.CRANK
    pmu.handle_frame(command(icd.ENGONOFF, 0))
    assert pmu.model.state == State.STOPPING


def test_engchk_before_start_selects_the_choke_crank():
    pmu = make_pmu()
    pmu.handle_frame(command(icd.ENGCHK, 1))
    pmu.handle_frame(command(icd.ENGONOFF, 1))
    assert pmu.model.state == State.CHOKING


def test_commands_with_the_wrong_dlc_are_rejected():
    pmu = make_pmu()
    pmu.handle_frame(command(icd.ENGONOFF, 1, dlc=8))
    assert pmu.model.state == State.STOPPED
    assert pmu._cmd_seen == 0


def test_battery_command_is_echoed_into_aux1sts():
    pmu = make_pmu()
    pmu.handle_frame(command(icd.BATCTRL, 80))
    pmu.handle_frame(frame(icd.AUX1STS, rtr=True))
    payload = pmu.sent[0][1]
    assert struct.unpack_from('<h', payload, 0)[0] == 80        # ICMD
    assert struct.unpack_from('<h', payload, 6)[0] == 800       # GCMD, 80.0 %


def test_manual_mode_sets_psts_bit_zero():
    pmu = make_pmu()
    pmu.handle_frame(command(icd.ENGMANUAL, 1))
    pmu.handle_frame(frame(icd.AUX2STS, rtr=True))
    assert struct.unpack_from('<H', pmu.sent[0][1], 4)[0] & 1


def test_an_undefined_engonoff_value_is_ignored():
    pmu = make_pmu()
    pmu.handle_frame(command(icd.ENGONOFF, 7))
    assert pmu.model.state == State.STOPPED


# -- takeoff detection ----------------------------------------------------------------

class FakeMessage(object):
    '''enough of a pymavlink message for TakeoffWatcher.consume()'''

    def __init__(self, kind, srcsystem=1, **fields):
        self._kind = kind
        self._srcsystem = srcsystem
        self.__dict__.update(fields)

    def get_type(self):
        return self._kind

    def get_srcSystem(self):
        return self._srcsystem

    def get_srcComponent(self):
        return 1


def vehicle_heartbeat(armed, srcsystem=1):
    return FakeMessage('HEARTBEAT', srcsystem, type=2, autopilot=3,
                       base_mode=128 if armed else 0)


def test_streams_are_requested_because_copter_defaults_them_to_zero():
    '''REGRESSION, found in SITL 2026-10-29.  Every AP_MAV_DEFAULT_STREAM_RATE_* is 0 for
       Copter (GCS_MAVLink_Parameters.cpp:60-70), so a passive listener gets HEARTBEAT and
       nothing else and can never see a takeoff.  The watcher has to ask.'''
    watcher = make_watcher()
    hb = vehicle_heartbeat(False)
    assert watcher.request_target(hb) == (1, 1)
    assert watcher.request_target(hb) is None, 'must not re-ask on every heartbeat'
    # a ground station is not worth asking
    assert watcher.request_target(gcs_heartbeat()) is None
    # and once the vehicle has been identified there is nothing left to ask for
    watcher.consume(position(0.0))
    watcher._requested.clear()
    assert watcher.request_target(hb) is None


def gcs_heartbeat(srcsystem=255):
    # what Mission Planner and KGCS send: MAV_TYPE_GCS, MAV_AUTOPILOT_INVALID, no arm bit
    return FakeMessage('HEARTBEAT', srcsystem, type=6, autopilot=8, base_mode=0)


def position(alt_m, srcsystem=1):
    return FakeMessage('GLOBAL_POSITION_INT', srcsystem, relative_alt=int(alt_m * 1000))


def make_watcher(sysid=None):
    return TakeoffWatcher('none', make_pmu(), 1.0, sysid)


def locked_watcher():
    """a watcher that has identified the vehicle, the way a real stream would"""
    watcher = make_watcher()
    watcher.consume(position(0.0))
    assert watcher.sysid == 1
    return watcher


def test_a_gcs_heartbeat_is_not_a_vehicle():
    assert is_vehicle_heartbeat(vehicle_heartbeat(True))
    assert not is_vehicle_heartbeat(gcs_heartbeat())


def test_the_vehicle_is_identified_by_telemetry_not_by_heartbeat():
    '''REGRESSION, found in SITL 2026-10-29, introduced by the fix for the one below.
       Locking onto the first non-GCS heartbeat looked right and was not: a ground
       station that does not label itself MAV_TYPE_GCS - KGCS may well not - captures
       the lock, and then no position message ever matches and takeoff is never
       detected at all.  Only a vehicle sends GLOBAL_POSITION_INT.'''
    watcher = make_watcher()
    # a ground station that looks like neither a GCS nor a vehicle, arriving first
    watcher.consume(FakeMessage('HEARTBEAT', 42, type=0, autopilot=0, base_mode=0))
    assert watcher.sysid is None, 'a heartbeat must not capture the lock'
    # the real vehicle, identified by telemetry only a vehicle sends
    watcher.consume(position(20.0))
    assert watcher.sysid == 1
    watcher.consume(vehicle_heartbeat(True))
    assert watcher.airborne()


def test_gcs_heartbeats_do_not_flap_the_airborne_state():
    '''REGRESSION, found in SITL 2026-10-29.  With Mission Planner and KGCS also on the
       link, the armed flag was taken from whichever HEARTBEAT arrived last, so it
       alternated at the heartbeat rate.  Before takeoff that is invisible - in_air is
       False either way - but after takeoff the state flapped GROUND_IDLE <-> FLIGHT and
       ILD, PCL and the head temperatures went up and down with it.'''
    watcher = locked_watcher()
    watcher.consume(vehicle_heartbeat(True))
    watcher.consume(position(20.0))
    assert watcher.airborne()
    for _ in range(10):
        watcher.consume(gcs_heartbeat())
        assert watcher.airborne(), 'a GCS heartbeat cleared the armed flag'
        watcher.consume(gcs_heartbeat(srcsystem=1))     # one sharing the vehicle's id
        assert watcher.airborne(), 'a GCS on the vehicle system id cleared the armed flag'
        watcher.consume(vehicle_heartbeat(True))
        assert watcher.airborne()


def test_another_vehicles_telemetry_is_ignored_once_locked():
    watcher = locked_watcher()
    watcher.consume(vehicle_heartbeat(True))
    watcher.consume(position(20.0, srcsystem=7))
    assert not watcher.airborne()
    watcher.consume(position(20.0, srcsystem=1))
    assert watcher.airborne()


def test_nothing_is_detected_before_any_telemetry_arrives():
    watcher = make_watcher()
    watcher.consume(gcs_heartbeat())
    watcher.consume(vehicle_heartbeat(True))
    assert watcher.sysid is None
    assert not watcher.airborne()


def test_a_pinned_sysid_is_used_instead_of_identifying_one():
    watcher = make_watcher(sysid=3)
    watcher.consume(position(20.0, srcsystem=1))        # not ours
    watcher.consume(vehicle_heartbeat(True, srcsystem=1))
    assert not watcher.airborne()
    watcher.consume(position(20.0, srcsystem=3))
    watcher.consume(vehicle_heartbeat(True, srcsystem=3))
    assert watcher.airborne()


def test_hysteresis_holds_state_inside_the_band():
    watcher = locked_watcher()
    watcher.consume(vehicle_heartbeat(True))
    watcher.consume(position(2.0))
    assert watcher.airborne()
    watcher.in_air = True
    watcher.consume(position(0.7))          # inside the band: hold
    assert watcher.airborne()
    watcher.consume(position(0.4))          # below half the threshold: landed
    assert not watcher.airborne()


def test_disarming_lands_immediately():
    watcher = locked_watcher()
    watcher.consume(vehicle_heartbeat(True))
    watcher.consume(position(20.0))
    watcher.in_air = True
    watcher.consume(vehicle_heartbeat(False))
    assert not watcher.airborne()


def test_descending_under_rtl_is_still_airborne():
    """REGRESSION, found in SITL 2026-10-29.  RTL reports MAV_LANDED_STATE_LANDING all
       the way down, and testing for IN_AIR and TAKEOFF rather than for NOT ON_GROUND
       dropped the engine to the 5000 rpm ground idle while the aircraft was descending."""
    watcher = locked_watcher()
    watcher.consume(vehicle_heartbeat(True))
    watcher.consume(position(30.0))
    watcher.consume(FakeMessage('EXTENDED_SYS_STATE', 1, landed_state=2))   # IN_AIR
    assert watcher.airborne()
    watcher.in_air = True
    watcher.consume(FakeMessage('EXTENDED_SYS_STATE', 1, landed_state=4))   # LANDING
    assert watcher.airborne(), 'descending is not landed'
    watcher.consume(FakeMessage('EXTENDED_SYS_STATE', 1, landed_state=1))   # ON_GROUND
    assert not watcher.airborne()


def test_an_unknown_landed_state_keeps_flight_power():
    watcher = locked_watcher()
    watcher.consume(vehicle_heartbeat(True))
    watcher.consume(FakeMessage('EXTENDED_SYS_STATE', 1, landed_state=9))
    assert watcher.airborne(), 'only an explicit ON_GROUND may drop the engine to idle'


def test_extended_sys_state_wins_but_undefined_is_not_an_answer():
    watcher = locked_watcher()
    watcher.consume(vehicle_heartbeat(True))
    watcher.consume(position(20.0))
    watcher.consume(FakeMessage('EXTENDED_SYS_STATE', 1, landed_state=0))
    assert watcher.landed_state is None     # UNDEFINED is no data, not "on the ground"
    assert watcher.airborne()               # so the altitude fallback still applies
    watcher.consume(FakeMessage('EXTENDED_SYS_STATE', 1, landed_state=1))
    assert not watcher.airborne()           # ON_GROUND overrides the altitude
    watcher.consume(FakeMessage('EXTENDED_SYS_STATE', 1, landed_state=2))
    assert watcher.airborne()


def test_extended_sys_state_alone_identifies_the_vehicle():
    watcher = make_watcher()
    watcher.consume(FakeMessage('EXTENDED_SYS_STATE', 1, landed_state=2))
    assert watcher.sysid == 1
    assert watcher.airborne()               # no heartbeat and no position needed


if __name__ == '__main__':
    failures = 0
    for name, fn in sorted(globals().items()):
        if not name.startswith('test_') or not callable(fn):
            continue
        try:
            fn()
            print('PASS  %s' % name)
        except AssertionError as err:
            failures += 1
            print('FAIL  %s  %s' % (name, err))
    print('%s' % ('all passed' if failures == 0 else '%u FAILED' % failures))
    sys.exit(1 if failures else 0)
