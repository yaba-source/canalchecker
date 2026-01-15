"""
Unit Tests für DriveStateMachine
Testet P-Controller, State-Logik und Geschwindigkeiten
"""

import unittest
import math
import sys
import os

# Füge Parent-Directory zum Python Path hinzu
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from canalchecker_dev.logik.DriveLogic import DriveStateMachine, KP, ANGULAR_Z_MAX, ANGLE_TOLERABLE_ERR


class TestDriveStateMachine(unittest.TestCase):
    """Test Suite für DriveStateMachine"""

    def setUp(self):
        """Wird vor jedem Test ausgeführt"""
        self.sm = DriveStateMachine()

    def tearDown(self):
        """Wird nach jedem Test ausgeführt"""
        self.sm = None

    # ==================== Initialisierungs-Tests ====================

    def test_initial_state(self):
        """Teste initiale Statuswerte"""
        self.assertEqual(self.sm.state, 1)
        self.assertFalse(self.sm.drive_complete)
        self.assertEqual(self.sm.id, -1)
        self.assertEqual(self.sm.distance, 0)
        self.assertEqual(self.sm.angle, 0)

    def test_initial_speeds(self):
        """Teste initiale Geschwindigkeiten"""
        self.assertEqual(self.sm.linear_speed, 0.0)
        self.assertEqual(self.sm.angular_speed, 0.0)
        self.assertEqual(self.sm.max_linear_speed, 0.0)

    # ==================== P-Controller Tests ====================

    def test_p_controller_zero_error(self):
        """P-Controller: Null-Fehler"""
        result = self.sm.p_controller(0.0, 0.0)
        self.assertEqual(result, 0.0)
        self.assertEqual(self.sm.angular_speed, 0.0)

    def test_p_controller_positive_error(self):
        """P-Controller: Positiver Fehler ohne Sättigung"""
        target = math.radians(5)
        actual = math.radians(0)
        result = self.sm.p_controller(target, actual)

        expected = KP * (target - actual)
        self.assertAlmostEqual(result, expected)
        self.assertAlmostEqual(self.sm.angular_speed, expected)

    def test_p_controller_negative_error(self):
        """P-Controller: Negativer Fehler ohne Sättigung"""
        target = math.radians(0)
        actual = math.radians(5)
        result = self.sm.p_controller(target, actual)

        expected = KP * (target - actual)
        self.assertAlmostEqual(result, expected)
        self.assertAlmostEqual(self.sm.angular_speed, expected)

    def test_p_controller_saturation_positive(self):
        """P-Controller: Positive Sättigung"""
        target = math.radians(180)
        actual = 0.0
        result = self.sm.p_controller(target, actual)

        self.assertEqual(result, ANGULAR_Z_MAX)
        self.assertEqual(self.sm.angular_speed, ANGULAR_Z_MAX)

    def test_p_controller_saturation_negative(self):
        """P-Controller: Negative Sättigung"""
        target = math.radians(-180)
        actual = 0.0
        result = self.sm.p_controller(target, actual)

        self.assertEqual(result, -ANGULAR_Z_MAX)
        self.assertEqual(self.sm.angular_speed, -ANGULAR_Z_MAX)

    # ==================== execute(): Allgemeines Verhalten ====================

    def test_execute_no_max_speed_stops(self):
        """execute: max_linear_speed = 0 → Roboter steht"""
        self.sm.max_linear_speed = 0.0
        self.sm.linear_speed = 1.0
        self.sm.angular_speed = 1.0

        self.sm.execute()

        self.assertEqual(self.sm.linear_speed, 0.0)
        self.assertEqual(self.sm.angular_speed, 0.0)

    # ==================== State 1 Tests ====================

    def test_state1_far_no_marker(self):
        """State 1: Weit entfernt, kein Marker"""
        self.sm.state = 1
        self.sm.max_linear_speed = 0.3
        self.sm.distance = 1.0
        self.sm.id = -1
        self.sm.angle = 0.0

        self.sm.execute()

        self.assertEqual(self.sm.state, 1)
        self.assertEqual(self.sm.linear_speed, 0.3)
        self.assertEqual(self.sm.angular_speed, 0.0)

    def test_state1_reach_target_distance_with_marker(self):
        """State 1: Distanz < 0.5 und Marker vorhanden → Wechsel in State 2"""
        self.sm.state = 1
        self.sm.max_linear_speed = 0.3
        self.sm.distance = 0.3
        self.sm.id = 5
        self.sm.angle = 0.0

        self.sm.execute()

        self.assertEqual(self.sm.state, 2)
        self.assertEqual(self.sm.linear_speed, 0.0)
        self.assertEqual(self.sm.angular_speed, 0.0)

    def test_state1_angle_above_tolerable_error(self):
        """State 1: Winkel > Toleranz → P-Regler aktiv"""
        self.sm.state = 1
        self.sm.max_linear_speed = 0.3
        self.sm.distance = 1.0
        self.sm.id = 5
        self.sm.angle = 10.0  # Grad

        self.sm.execute()

        error = 0.0 - math.radians(self.sm.angle)
        expected = KP * error
        if expected > ANGULAR_Z_MAX:
            expected = ANGULAR_Z_MAX
        elif expected < -ANGULAR_Z_MAX:
            expected = -ANGULAR_Z_MAX

        self.assertEqual(self.sm.linear_speed, self.sm.max_linear_speed)
        self.assertAlmostEqual(self.sm.angular_speed, expected)

    def test_state1_angle_within_tolerable_error(self):
        """State 1: Winkel innerhalb Toleranz → angular_speed = 0"""
        self.sm.state = 1
        self.sm.max_linear_speed = 0.3
        self.sm.distance = 1.0
        self.sm.id = 5
        self.sm.angle = ANGLE_TOLERABLE_ERR - 0.1

        self.sm.angular_speed = 1.0
        self.sm.execute()

        self.assertEqual(self.sm.linear_speed, self.sm.max_linear_speed)
        self.assertEqual(self.sm.angular_speed, 0.0)

    def test_state1_angle_exactly_tolerable_error(self):
        """State 1: Winkel = Toleranz → sollte nicht geregelt werden (elif-Zweig greift nicht)"""
        self.sm.state = 1
        self.sm.max_linear_speed = 0.3
        self.sm.distance = 1.0
        self.sm.id = 5
        self.sm.angle = ANGLE_TOLERABLE_ERR

        # Erwartung: keine der beiden Bedingungen trifft zu → angular_speed bleibt unverändert
        self.sm.angular_speed = 0.5
        self.sm.execute()

        self.assertEqual(self.sm.linear_speed, self.sm.max_linear_speed)
        self.assertEqual(self.sm.angular_speed, 0.5)

    # ==================== State 2 Tests ====================

    def test_state2_drive_complete(self):
        """State 2: Fahrt abgeschlossen"""
        self.sm.state = 2
        self.sm.max_linear_speed = 0.3
        self.sm.linear_speed = 0.3

        self.sm.execute()

        self.assertTrue(self.sm.drive_complete)
        self.assertEqual(self.sm.linear_speed, 0.0)
        # angular_speed bleibt in execute() für state 2 unverändert, sollte aber zuvor schon 0 sein
        # Falls du dort explizit auf 0 setzen möchtest, müsstest du den Code anpassen.

    # ==================== Integrations- / Szenario-Tests ====================

    def test_simple_drive_sequence(self):
        """Integrations-Test: Einfache Fahrt bis Marker und Stop"""
        # Start: State 1, fährt auf Marker zu
        self.sm.state = 1
        self.sm.max_linear_speed = 0.3
        self.sm.id = 5

        # Mehrere Schritte mit abnehmender Distanz
        for dist in [2.0, 1.0, 0.6]:
            self.sm.distance = dist
            self.sm.angle = 0.0
            self.sm.execute()
            self.assertEqual(self.sm.state, 1)
            self.assertEqual(self.sm.linear_speed, self.sm.max_linear_speed)

        # Jetzt Zielbereich erreichen
        self.sm.distance = 0.3
        self.sm.execute()
        self.assertEqual(self.sm.state, 2)
        self.assertEqual(self.sm.linear_speed, 0.0)

        # Abschluss
        self.sm.execute()
        self.assertTrue(self.sm.drive_complete)


if __name__ == '__main__':
    unittest.main()
