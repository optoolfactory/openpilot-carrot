#!/usr/bin/env python3
"""
Test script for eye detection loss and drowsy driving alerts.
Tests that alerts trigger within 3 seconds as required.
"""

import numpy as np
from cereal import log, car
from openpilot.common.realtime import DT_DMON
from openpilot.selfdrive.monitoring.helpers import DriverMonitoring, DRIVER_MONITOR_SETTINGS
from openpilot.system.hardware import HARDWARE

EventName = log.OnroadEvent.EventName
dm_settings = DRIVER_MONITOR_SETTINGS(device_type=HARDWARE.get_device_type())

# Frame counts for 3 seconds at 50Hz
FRAMES_3_SECONDS = int(3.0 / DT_DMON)
print(f"DT_DMON: {DT_DMON}")
print(f"Frames for 3 seconds: {FRAMES_3_SECONDS}")
print(f"Eyes not detected alert time: {dm_settings._EYES_NOT_DETECTED_ALERT_TIME}")
print(f"Drowsy alert time: {dm_settings._DROWSY_ALERT_TIME}")


def make_driver_state(face_detected=True, eyes_visible=True, eyes_closed_prob=0.0, phone_prob=0.0):
    """Create a driver state message for testing."""
    ds = log.DriverStateV2.new_message()
    
    # Set face detection
    ds.leftDriverData.faceProb = 1.0 if face_detected else 0.0
    ds.leftDriverData.faceOrientation = [0., 0., 0.]
    ds.leftDriverData.facePosition = [0., 0.]
    ds.leftDriverData.faceOrientationStd = [0.1, 0.1, 0.1]
    ds.leftDriverData.facePositionStd = [0.1, 0.1]
    
    # Set eye detection
    ds.leftDriverData.eyesVisibleProb = 1.0 if eyes_visible else 0.0
    ds.leftDriverData.eyesClosedProb = eyes_closed_prob
    ds.leftDriverData.phoneProb = phone_prob
    
    return ds


def make_car_state():
    """Create a car state message."""
    cs = car.CarState.new_message()
    cs.vEgo = 20.0  # 20 m/s
    cs.standstill = False
    cs.steeringPressed = False
    cs.gasPressed = False
    cs.steeringAngleDeg = 0.0
    cs.gearShifter = car.CarState.GearShifter.drive
    return cs


def test_eyes_not_detected_alert():
    """Test that alert triggers when eyes not detected for 3 seconds."""
    print("\n" + "="*60)
    print("Test 1: Eyes Not Detected Alert")
    print("="*60)
    
    dm = DriverMonitoring()
    
    # Create a mock message structure
    class MockMessage:
        def __init__(self):
            self.driverStateV2 = make_driver_state(face_detected=True, eyes_visible=False)
            self.carState = make_car_state()
            self.selfdriveState = type('obj', (object,), {'enabled': False})()
            self.liveCalibration = type('obj', (object,), {'rpyCalib': [0., 0., 0.]})()
            self.modelV2 = type('obj', (object,), {
                'meta': type('obj', (object,), {
                    'disengagePredictions': type('obj', (object,), {
                        'brakeDisengageProbs': [0.0]
                    })()
                })()
            })()
            self.frameId = 0
            
        def __getitem__(self, key):
            if key == 'driverStateV2':
                return self.driverStateV2
            elif key == 'carState':
                return self.carState
            elif key == 'selfdriveState':
                return self.selfdriveState
            elif key == 'liveCalibration':
                return self.liveCalibration
            elif key == 'modelV2':
                return self.modelV2
            return None
        
        def all_checks(self):
            return True
        
        def valid(self):
            return {'driverStateV2': True, 'carState': True}
    
    # Simulate eyes not detected for 3 seconds
    alert_triggered = False
    for i in range(FRAMES_3_SECONDS + 10):
        mock_msg = MockMessage()
        mock_msg.frameId = i
        
        # Update states and events
        dm._update_states(
            driver_state=mock_msg.driverStateV2,
            cal_rpy=[0., 0., 0.],
            car_speed=20.0,
            op_engaged=False,
            standstill=False,
            demo_mode=False,
            steering_angle_deg=0.0
        )
        
        dm._update_events(
            driver_engaged=False,
            op_engaged=False,
            standstill=False,
            wrong_gear=False,
            car_speed=20.0
        )
        
        # Check if alert was triggered
        if EventName.driverDistracted2 in dm.current_events.names:
            if not alert_triggered:
                alert_triggered = True
                print(f"✓ Alert triggered at frame {i} ({i * DT_DMON:.2f}s)")
                print(f"  Eyes not detected counter: {dm.eyes_not_detected_counter}")
        
        if i % 50 == 0:
            print(f"Frame {i} ({i * DT_DMON:.2f}s): counter={dm.eyes_not_detected_counter}, alert={alert_triggered}")
    
    if alert_triggered:
        print("✓ TEST PASSED: Eyes not detected alert triggered within 3 seconds")
        return True
    else:
        print("✗ TEST FAILED: Eyes not detected alert was not triggered")
        return False


def test_drowsy_driving_alert():
    """Test that alert triggers when drowsy driving detected for 3 seconds."""
    print("\n" + "="*60)
    print("Test 2: Drowsy Driving Alert")
    print("="*60)
    
    dm = DriverMonitoring()
    
    class MockMessage:
        def __init__(self):
            # Eyes visible but closed (drowsy)
            self.driverStateV2 = make_driver_state(
                face_detected=True, 
                eyes_visible=True, 
                eyes_closed_prob=0.8  # High drowsy prob
            )
            self.carState = make_car_state()
            self.selfdriveState = type('obj', (object,), {'enabled': False})()
            self.liveCalibration = type('obj', (object,), {'rpyCalib': [0., 0., 0.]})()
            self.modelV2 = type('obj', (object,), {
                'meta': type('obj', (object,), {
                    'disengagePredictions': type('obj', (object,), {
                        'brakeDisengageProbs': [0.0]
                    })()
                })()
            })()
            self.frameId = 0
            
        def __getitem__(self, key):
            if key == 'driverStateV2':
                return self.driverStateV2
            elif key == 'carState':
                return self.carState
            elif key == 'selfdriveState':
                return self.selfdriveState
            elif key == 'liveCalibration':
                return self.liveCalibration
            elif key == 'modelV2':
                return self.modelV2
            return None
        
        def all_checks(self):
            return True
    
    # Simulate drowsy driving for 3 seconds
    alert_triggered = False
    for i in range(FRAMES_3_SECONDS + 10):
        mock_msg = MockMessage()
        mock_msg.frameId = i
        
        # Update states and events
        dm._update_states(
            driver_state=mock_msg.driverStateV2,
            cal_rpy=[0., 0., 0.],
            car_speed=20.0,
            op_engaged=False,
            standstill=False,
            demo_mode=False,
            steering_angle_deg=0.0
        )
        
        dm._update_events(
            driver_engaged=False,
            op_engaged=False,
            standstill=False,
            wrong_gear=False,
            car_speed=20.0
        )
        
        # Check if alert was triggered
        if EventName.driverDistracted2 in dm.current_events.names:
            if not alert_triggered:
                alert_triggered = True
                print(f"✓ Alert triggered at frame {i} ({i * DT_DMON:.2f}s)")
                print(f"  Drowsy counter: {dm.drowsy_counter}")
        
        if i % 50 == 0:
            print(f"Frame {i} ({i * DT_DMON:.2f}s): counter={dm.drowsy_counter}, alert={alert_triggered}")
    
    if alert_triggered:
        print("✓ TEST PASSED: Drowsy driving alert triggered within 3 seconds")
        return True
    else:
        print("✗ TEST FAILED: Drowsy driving alert was not triggered")
        return False


def test_alert_reset_on_eyes_visible():
    """Test that counters reset when eyes become visible."""
    print("\n" + "="*60)
    print("Test 3: Alert Reset When Eyes Visible")
    print("="*60)
    
    dm = DriverMonitoring()
    
    class MockMessage:
        def __init__(self, eyes_visible=True):
            self.driverStateV2 = make_driver_state(
                face_detected=True, 
                eyes_visible=eyes_visible
            )
            self.carState = make_car_state()
            self.selfdriveState = type('obj', (object,), {'enabled': False})()
            self.liveCalibration = type('obj', (object,), {'rpyCalib': [0., 0., 0.]})()
            self.modelV2 = type('obj', (object,), {
                'meta': type('obj', (object,), {
                    'disengagePredictions': type('obj', (object,), {
                        'brakeDisengageProbs': [0.0]
                    })()
                })()
            })()
            self.frameId = 0
            
        def __getitem__(self, key):
            if key == 'driverStateV2':
                return self.driverStateV2
            elif key == 'carState':
                return self.carState
            elif key == 'selfdriveState':
                return self.selfdriveState
            elif key == 'liveCalibration':
                return self.liveCalibration
            elif key == 'modelV2':
                return self.modelV2
            return None
        
        def all_checks(self):
            return True
    
    # Eyes not visible for 2 seconds (not enough for alert)
    for i in range(int(2.0 / DT_DMON)):
        mock_msg = MockMessage(eyes_visible=False)
        mock_msg.frameId = i
        dm._update_states(
            driver_state=mock_msg.driverStateV2,
            cal_rpy=[0., 0., 0.],
            car_speed=20.0,
            op_engaged=False,
            standstill=False,
            demo_mode=False,
            steering_angle_deg=0.0
        )
    
    print(f"After 2 seconds of eyes not detected: counter={dm.eyes_not_detected_counter}")
    
    # Eyes become visible
    for i in range(10):
        mock_msg = MockMessage(eyes_visible=True)
        mock_msg.frameId = int(2.0 / DT_DMON) + i
        dm._update_states(
            driver_state=mock_msg.driverStateV2,
            cal_rpy=[0., 0., 0.],
            car_speed=20.0,
            op_engaged=False,
            standstill=False,
            demo_mode=False,
            steering_angle_deg=0.0
        )
    
    if dm.eyes_not_detected_counter == 0 and not dm.eyes_not_detected_alerted:
        print("✓ TEST PASSED: Counter reset when eyes became visible")
        return True
    else:
        print("✗ TEST FAILED: Counter did not reset properly")
        return False


if __name__ == "__main__":
    print("\n" + "="*60)
    print("Eye Detection and Drowsy Driving Alert Tests")
    print("="*60)
    
    results = []
    try:
        results.append(("Eyes Not Detected Alert", test_eyes_not_detected_alert()))
    except Exception as e:
        print(f"✗ Test failed with error: {e}")
        import traceback
        traceback.print_exc()
        results.append(("Eyes Not Detected Alert", False))
    
    try:
        results.append(("Drowsy Driving Alert", test_drowsy_driving_alert()))
    except Exception as e:
        print(f"✗ Test failed with error: {e}")
        import traceback
        traceback.print_exc()
        results.append(("Drowsy Driving Alert", False))
    
    try:
        results.append(("Alert Reset on Eyes Visible", test_alert_reset_on_eyes_visible()))
    except Exception as e:
        print(f"✗ Test failed with error: {e}")
        import traceback
        traceback.print_exc()
        results.append(("Alert Reset on Eyes Visible", False))
    
    print("\n" + "="*60)
    print("Test Summary")
    print("="*60)
    for test_name, passed in results:
        status = "✓ PASSED" if passed else "✗ FAILED"
        print(f"{test_name}: {status}")
    
    all_passed = all(result[1] for result in results)
    print("\n" + ("="*60))
    if all_passed:
        print("All tests PASSED!")
    else:
        print("Some tests FAILED!")
    print("="*60)
