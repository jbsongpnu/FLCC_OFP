#!/usr/bin/env python3
'''
PNU : tests for the fake PMU's engine model and wire format.

Two things are checked, and the second matters more than the first:

  1. the RPM ladder behaves as README.md's table says, including the cold-start abort
  2. every status frame decodes to the intended value AT THE OFFSETS AP_PMUCAN READS.
     The offsets below are transcribed from AP_PMUCAN::handle_frame() by hand rather
     than imported, so that a change on either side shows up here as a failure instead
     of agreeing with itself.

Run with pytest, or directly:  Tools/pnu/sitl_can/test_pmu_model.py

AP_FLAKE8_CLEAN
'''

import os
import struct
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import pmu_frames as icd                                                # noqa: E402
from pmu_model import PMUModel, State, TEMP_AMBIENT, ILD_FLIGHT         # noqa: E402
from pmu_model import RPM_CRANK, RPM_WARMUP, RPM_GROUND_IDLE, RPM_FLIGHT  # noqa: E402

DT = 0.005


def advance(model, seconds):
    for _ in range(int(round(seconds / DT))):
        model.update(DT)


def started(choke_closed=False):
    model = PMUModel()
    model.cmd_choke(1 if choke_closed else 0)
    model.cmd_engine_onoff(1)
    return model


def at_ground_idle():
    '''run the ladder up to the 5000 rpm ground idle'''
    model = started()
    advance(model, 60.0)
    assert model.state == State.GROUND_IDLE
    return model


# -- the ladder -----------------------------------------------------------------------

def test_choke_crank_reaches_1000_then_stops_without_running():
    model = started(choke_closed=True)
    advance(model, 1.0)
    assert model.state == State.CHOKING
    assert round(model.rpm) == RPM_CRANK
    assert not model.running
    advance(model, 2.0)                     # 2 s hold ends
    assert model.state == State.STOPPED
    assert not model.running


def test_crank_holds_two_seconds_then_ramps_to_warmup_in_one():
    model = started()
    advance(model, 0.5)
    assert round(model.rpm) == RPM_CRANK
    advance(model, 2.0)
    assert model.state == State.WARMUP      # the 2 s hold is over
    advance(model, 1.05)                    # 1000 -> 4200 is specified as 1.0 s
    assert round(model.rpm) == RPM_WARMUP


def test_heads_climb_at_five_degrees_per_second():
    model = started()
    advance(model, 2.5)                     # into WARMUP
    before = model.head_temp
    advance(model, 4.0)
    assert abs((model.head_temp - before) - 20.0) < 0.5


def test_cooling_device_trips_ground_idle_at_eighty():
    model = at_ground_idle()
    assert model.head_temp >= 80.0
    assert round(model.rpm) == RPM_GROUND_IDLE
    assert model.load_current == 6.3
    assert model.throttle_position == 13
    assert model.psts & (1 << 4)            # cooling fan


def test_flight_only_from_ground_idle():
    model = at_ground_idle()
    model.set_in_air(True)
    advance(model, 1.0)
    assert model.state == State.FLIGHT
    assert round(model.rpm) == RPM_FLIGHT
    assert model.load_current == ILD_FLIGHT
    assert model.battery_current == 0.0
    advance(model, 30.0)
    assert abs(model.head_temp - 180.0) < 0.1


def test_takeoff_below_ground_idle_aborts_and_latches():
    model = started()
    advance(model, 4.0)                     # WARMUP, well below 5000 rpm
    model.set_in_air(True)
    advance(model, 3.0)
    assert model.state == State.ABORT
    assert model.rpm == 0.0
    assert model.battery_current == -ILD_FLIGHT
    assert model.load_current == ILD_FLIGHT
    assert model.psts & (1 << 7)            # unintended stop
    # generated = ILD + IBAT, so a dead engine generates nothing
    assert abs(model.load_current + model.battery_current) < 1e-6
    # latched: time alone does not clear it
    advance(model, 30.0)
    assert model.state == State.ABORT
    model.cmd_engine_onoff(1)
    assert model.state == State.CRANK
    assert not model.abort_latched


def test_stop_winds_down_and_sets_the_stopping_bit():
    model = at_ground_idle()
    model.cmd_engine_onoff(0)
    assert model.state == State.STOPPING
    assert model.psts & (1 << 2)
    advance(model, 5.0)
    assert model.state == State.STOPPED
    assert model.rpm == 0.0
    advance(model, 30.0)
    assert abs(model.head_temp - TEMP_AMBIENT) < 0.1


def test_starting_bit_clears_once_warm_up_rpm_is_reached():
    model = started()
    advance(model, 1.0)
    assert model.psts & (1 << 1)
    advance(model, 3.2)
    assert not model.psts & (1 << 1)


def test_choke_bit_follows_the_command():
    model = PMUModel()
    assert not model.psts & (1 << 3)
    model.cmd_choke(1)
    assert model.psts & (1 << 3)
    model.cmd_choke(0)
    assert not model.psts & (1 << 3)


def test_fuel_low_is_never_set():
    model = at_ground_idle()
    assert not model.psts & (1 << 8)


# -- the wire -------------------------------------------------------------------------

def test_every_status_frame_is_eight_bytes():
    model = at_ground_idle()
    for can_id in icd.STATUS_IDS:
        assert len(icd.build(can_id, model)) == icd.STATUS_DLC


def test_batsts_decodes_at_the_offsets_ap_pmucan_reads():
    model = started()
    advance(model, 4.0)
    model.set_in_air(True)                  # abort: the only non-zero IBAT
    advance(model, 3.0)
    data = icd.build(icd.BATSTS, model)
    assert struct.unpack_from('<h', data, 2)[0] == 0            # Battery_Temp, not modelled
    assert struct.unpack_from('<h', data, 4)[0] == -875         # Battery_Current, -87.5 A
    assert struct.unpack_from('<h', data, 6)[0] == 1000         # GAGE, 100.0 %


def test_engsts_decodes_at_the_offsets_ap_pmucan_reads():
    model = at_ground_idle()
    model.set_in_air(True)
    advance(model, 40.0)
    data = icd.build(icd.ENGSTS, model)
    assert struct.unpack_from('<h', data, 0)[0] == 1800         # Engine_Head1_Temp, 180.0 degC
    assert struct.unpack_from('<h', data, 2)[0] == 1800         # Engine_Head2_Temp
    assert struct.unpack_from('<H', data, 4)[0] == RPM_FLIGHT   # Engine_RPM, raw rpm
    assert struct.unpack_from('<b', data, 6)[0] == 100          # Fuel_Quantity, raw percent
    assert struct.unpack_from('<b', data, 7)[0] == 50           # Throttle_Position_Report


def test_aux1sts_decodes_at_the_offsets_ap_pmucan_reads():
    model = at_ground_idle()
    model.cmd_battery(80)
    data = icd.build(icd.AUX1STS, model)
    assert struct.unpack_from('<h', data, 0)[0] == 80           # Current_Control_Command, echo
    assert struct.unpack_from('<h', data, 2)[0] == 63           # Load_Current, 6.3 A
    assert struct.unpack_from('<h', data, 4)[0] == 480          # System_Voltage, 48.0 V
    assert struct.unpack_from('<h', data, 6)[0] == 800          # Battery_Quantity_Command, 80.0 %


def test_aux2sts_decodes_at_the_offsets_ap_pmucan_reads():
    model = at_ground_idle()
    data = icd.build(icd.AUX2STS, model)
    # HOURCNT is UINT24 little-endian: AP_PMUCAN zeroes a uint32 and memcpys three bytes
    hourcnt = struct.unpack('<I', data[0:3] + b'\x00')[0]
    assert hourcnt == int(model.hour_count_s)
    assert hourcnt > 0                                          # it counted while running
    assert struct.unpack_from('<H', data, 4)[0] == model.psts   # PMU_Status
    assert struct.unpack_from('<h', data, 6)[0] == 0            # PMU_Temp, not modelled
    assert data[3] == 0                                         # the reserved byte


def test_versts_decodes_at_the_offsets_ap_pmucan_reads():
    data = icd.build(icd.VERSTS, PMUModel())
    assert struct.unpack_from('<I', data, 0)[0] == icd.VERSION_DATE_BCD
    assert data[4] == 0x00                                      # Version_Sub_Number, MN
    assert data[5] == 0x01                                      # Version_Main_Number, MJ


def test_scale_is_one_tenth_and_saturates_rather_than_wrapping():
    assert icd.scaled(48.0) == 480
    assert icd.scaled(-87.5) == -875
    assert icd.scaled(6.3) == 63
    assert icd.scaled(100000.0) == 32767
    assert icd.scaled(-100000.0) == -32768


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
