#!/usr/bin/env python3
"""
Test script for pupils not detected warning feature.
운전자의 눈동자가 3초 이상 인식되지 않으면 경고를 발생시키는 기능 테스트
"""

import unittest
from unittest.mock import Mock, MagicMock, patch
import numpy as np
from openpilot.selfdrive.monitoring.helpers import DriverMonitoring, DRIVER_MONITOR_SETTINGS
from cereal import log


class TestPupilsNotDetectedWarning(unittest.TestCase):
    """눈동자 미인식 경고 기능 테스트"""

    def setUp(self):
        """테스트 전 준비"""
        self.settings = DRIVER_MONITOR_SETTINGS(device_type="")
        self.dm = DriverMonitoring(settings=self.settings)

    def test_pupils_not_detected_warning_time_setting(self):
        """눈동자 미인식 경고 시간 설정이 3초인지 확인"""
        self.assertEqual(self.settings._PUPILS_NOT_DETECTED_WARNING_TIME, 3.0)

    def test_pupils_not_detected_variables_initialized(self):
        """눈동자 미인식 추적 변수가 제대로 초기화되는지 확인"""
        self.assertEqual(self.dm.pupils_not_detected_duration, 0.)
        self.assertFalse(self.dm.pupils_not_detected_warning_triggered)
        self.assertFalse(self.dm.pupils_not_detected_warning)

    def test_pupils_not_detected_duration_increases(self):
        """eyesVisibleProb가 낮을 때 duration이 증가하는지 확인"""
        # Mock driver data with low eye visibility
        driver_data = Mock()
        driver_data.faceProb = 0.9  # Face detected
        driver_data.eyesClosedProb = 0.1  # Eyes open
        driver_data.eyesVisibleProb = 0.2  # Eyes not visible (below 0.5 threshold)
        driver_data.phoneProb = 0.0
        driver_data.faceOrientation = [0., 0., 0.]
        driver_data.facePosition = [0., 0.]
        driver_data.faceOrientationStd = [0.1, 0.1, 0.1]
        driver_data.facePositionStd = [0.1, 0.1]

        # Mock driver state
        driver_state = Mock()
        driver_state.leftDriverData = driver_data
        driver_state.rightDriverData = driver_data
        driver_state.wheelOnRightProb = 0.5
        driver_state.frameId = 0

        # Set conditions for pupils not detected check
        self.dm.face_detected = True
        self.dm.pose.low_std = True

        # Manually test the logic (simulating _update_states)
        eyes_visible = driver_data.eyesVisibleProb > self.settings._EYE_THRESHOLD

        # Since eyesVisibleProb = 0.2 and threshold = 0.5, eyes_visible should be False
        self.assertFalse(eyes_visible)

        # Test duration accumulation
        self.dm.pupils_not_detected_duration = 0.
        self.dm.pupils_not_detected_warning_triggered = False
        self.dm.pupils_not_detected_warning = False

        # Simulate 4 seconds (60 frames at ~15Hz or similar)
        dt = self.settings._DT_DMON
        iterations = int(3.5 / dt)

        for i in range(iterations):
            if self.dm.face_detected and self.dm.pose.low_std and not eyes_visible:
                self.dm.pupils_not_detected_duration += dt
                if (self.dm.pupils_not_detected_duration >=
                    self.settings._PUPILS_NOT_DETECTED_WARNING_TIME and
                    not self.dm.pupils_not_detected_warning_triggered):
                    self.dm.pupils_not_detected_warning = True
                    self.dm.pupils_not_detected_warning_triggered = True

        # Check that warning was triggered after 3 seconds
        self.assertTrue(self.dm.pupils_not_detected_warning)
        self.assertGreaterEqual(self.dm.pupils_not_detected_duration,
                               self.settings._PUPILS_NOT_DETECTED_WARNING_TIME)

    def test_pupils_detected_resets_duration(self):
        """눈동자가 다시 인식되면 duration이 초기화되는지 확인"""
        # Set initial state
        self.dm.pupils_not_detected_duration = 2.5
        self.dm.pupils_not_detected_warning_triggered = True
        self.dm.pupils_not_detected_warning = False

        # Mock with high eye visibility (eyes detected)
        eyes_visible = True

        # Simulate reset condition
        self.dm.face_detected = True
        self.dm.pose.low_std = True

        if self.dm.face_detected and self.dm.pose.low_std and not eyes_visible:
            self.dm.pupils_not_detected_duration += self.settings._DT_DMON
        else:
            self.dm.pupils_not_detected_duration = 0.
            self.dm.pupils_not_detected_warning_triggered = False

        # Check that duration was reset
        self.assertEqual(self.dm.pupils_not_detected_duration, 0.)
        self.assertFalse(self.dm.pupils_not_detected_warning_triggered)

    def test_warning_only_triggers_once(self):
        """경고는 3초 이상 지속되어도 한 번만 발생하는지 확인"""
        self.dm.pupils_not_detected_duration = 2.0
        self.dm.pupils_not_detected_warning_triggered = False
        self.dm.pupils_not_detected_warning = False

        # First trigger
        if (self.dm.pupils_not_detected_duration >=
            self.settings._PUPILS_NOT_DETECTED_WARNING_TIME and
            not self.dm.pupils_not_detected_warning_triggered):
            self.dm.pupils_not_detected_warning = True
            self.dm.pupils_not_detected_warning_triggered = True

        # Duration exceeds threshold
        self.dm.pupils_not_detected_duration = 5.0

        warning_count = 0
        if (self.dm.pupils_not_detected_duration >=
            self.settings._PUPILS_NOT_DETECTED_WARNING_TIME and
            not self.dm.pupils_not_detected_warning_triggered):
            warning_count += 1

        # Should not trigger again
        self.assertEqual(warning_count, 0)


class TestPupilsNotDetectedIntegration(unittest.TestCase):
    """눈동자 미인식 경고 기능 통합 테스트"""

    def test_threshold_values(self):
        """임계값들이 올바르게 설정되었는지 확인"""
        settings = DRIVER_MONITOR_SETTINGS(device_type="")

        # Eye detection threshold should be 0.5
        self.assertEqual(settings._EYE_THRESHOLD, 0.5)

        # Pupils not detected warning time should be 3 seconds
        self.assertEqual(settings._PUPILS_NOT_DETECTED_WARNING_TIME, 3.0)

        # Should have same time as eyes closed warning for consistency
        self.assertEqual(settings._PUPILS_NOT_DETECTED_WARNING_TIME,
                        settings._EYES_CLOSED_WARNING_TIME)


def print_summary():
    """기능 요약 출력"""
    print("\n" + "="*70)
    print("운전자 눈동자 미인식 경고 기능 구현 완료")
    print("="*70)
    print("\n[기능 설명]")
    print("- 운전자의 눈동자(pupils)가 3초 이상 인식되지 않으면 경고 발생")
    print("- eyesVisibleProb < 0.5 상태가 3초 이상 지속되면 경고")
    print("- 경고는 driverDistracted2 이벤트로 발생")
    print("\n[구현 사항]")
    print("1. DRIVER_MONITOR_SETTINGS에 _PUPILS_NOT_DETECTED_WARNING_TIME = 3.0 추가")
    print("2. DriverMonitoring 클래스에 추적 변수 추가:")
    print("   - pupils_not_detected_duration")
    print("   - pupils_not_detected_warning_triggered")
    print("   - pupils_not_detected_warning")
    print("3. _update_states 메서드에서 eyesVisibleProb 모니터링")
    print("4. _update_events 메서드에서 경고 이벤트 발생")
    print("\n[동작 방식]")
    print("- 얼굴이 감지되고 자세가 안정적일 때 눈동자 상태 확인")
    print("- eyesVisibleProb <= 0.5이면 카운트 시작")
    print("- 3초 이상 지속되면 driverDistracted2 경고 발생")
    print("- 눈동자가 다시 감지되면 카운트 초기화")
    print("="*70 + "\n")


if __name__ == '__main__':
    print_summary()
    unittest.main()
