"""
Unit Tests für FollowStateMachine
Testet Follow-Logik, Abstandsregelung und P-Controller
"""

import unittest
import math
from FollowLogic import FollowStateMachine, KP_ANGULAR


class TestFollowStateMachine(unittest.TestCase):
    """Test Suite für FollowStateMachine"""
    
    def setUp(self):
        """Wird vor jedem Test ausgeführt"""
        self.sm = FollowStateMachine()
    
    def tearDown(self):
        """Wird nach jedem Test ausgeführt"""
        self.sm = None
    
    # ==================== Initialisierungs-Tests ====================
    
    def test_initial_state(self):
        """Teste initiale Werte"""
        self.assertEqual(self.sm.state, 10)
        self.assertFalse(self.sm.follow_done)
        self.assertEqual(self.sm.id, -1)
        self.assertEqual(self.sm.id_to_follow, 69)
        self.assertTrue(self.sm.robot_found)
    
    def test_initial_speeds(self):
        """Teste initiale Geschwindigkeiten"""
        self.assertEqual(self.sm.linear_speed, 0.0)
        self.assertEqual(self.sm.angular_speed, 0.0)
    
    def test_target_distance_default(self):
        """Teste Standard-Zieldistanz"""
        self.assertEqual(self.sm.target_distance, 50.0)
    
    # ==================== State 10: Suche Tests ====================
    
    def test_state10_no_robot(self):
        """State 10: Kein Roboter erkannt"""
        self.sm.state = 10
        self.sm.id = -1
        
        self.sm.execute()
        
        self.assertEqual(self.sm.state, 10)
        self.assertFalse(self.sm.robot_found)
    
    def test_state10_wrong_marker(self):
        """State 10: Falscher Marker ignoriert"""
        self.sm.state = 10
        self.sm.id = 0  # Nicht 69
        
        self.sm.execute()
        
        self.assertEqual(self.sm.state, 10)
        self.assertFalse(self.sm.robot_found)
    
    def test_state10_correct_marker_found(self):
        """State 10: Marker 69 gefunden → Wechsel zu State 20"""
        self.sm.state = 10
        self.sm.id = 69
        self.sm.distance = 1.0
        
        self.sm.execute()
        
        self.assertEqual(self.sm.state, 20)
    
    # ==================== State 20: Follow Tests ====================
    
    def test_state20_follow_with_correct_marker(self):
        """State 20: Folgt Marker 69"""
        self.sm.state = 20
        self.sm.id = 69
        self.sm.distance = 1.0  # 100 cm
        self.sm.angle = 5.0
        self.sm.target_distance = 50.0  # Soll-Abstand 50 cm
        
        self.sm.execute()
        
        self.assertEqual(self.sm.state, 20)
        self.assertTrue(self.sm.robot_found)
        self.assertEqual(self.sm.marker_lost_counter, 0)
        self.assertNotEqual(self.sm.linear_speed, 0.0)
        self.assertNotEqual(self.sm.angular_speed, 0.0)
    
    def test_state20_distance_control(self):
        """State 20: Abstandsregelung funktioniert"""
        self.sm.state = 20
        self.sm.id = 69
        self.sm.angle = 0.0  # Perfekt ausgerichtet
        self.sm.target_distance = 50.0  # 50 cm Soll
        
        # Test: Zu weit entfernt → sollte vorwärts fahren
        self.sm.distance = 1.5  # 150 cm
        self.sm.execute()
        self.assertGreater(self.sm.linear_speed, 0, "Sollte vorwärts fahren")
        
        # Test: Zu nah → sollte rückwärts fahren
        self.sm.distance = 0.3  # 30 cm
        self.sm.execute()
        self.assertLess(self.sm.linear_speed, 0, "Sollte rückwärts fahren")
    
    def test_state20_distance_too_close_stops(self):
        """State 20: Stoppt bei zu geringer Distanz"""
        self.sm.state = 20
        self.sm.id = 69
        self.sm.target_distance = 50.0
        self.sm.distance = 0.4  # 40 cm, unter target_distance
        
        self.sm.execute()
        
        self.assertEqual(self.sm.linear_speed, 0.0, "Sollte stoppen")
    
    def test_state20_angular_correction_positive(self):
        """State 20: Winkelkorrektur bei positivem Winkel"""
        self.sm.state = 20
        self.sm.id = 69
        self.sm.distance = 1.0
        self.sm.angle = 10.0
        self.sm.target_distance = 50.0
        
        self.sm.execute()
        
        self.assertLess(self.sm.angular_speed, 0, "Sollte nach links drehen")
    
    def test_state20_angular_correction_negative(self):
        """State 20: Winkelkorrektur bei negativem Winkel"""
        self.sm.state = 20
        self.sm.id = 69
        self.sm.distance = 1.0
        self.sm.angle = -10.0
        self.sm.target_distance = 50.0
        
        self.sm.execute()
        
        self.assertGreater(self.sm.angular_speed, 0, "Sollte nach rechts drehen")
    
    def test_state20_marker_lost(self):
        """State 20: Marker 69 verloren"""
        self.sm.state = 20
        self.sm.id = -1  # Marker verloren
        self.sm.distance = 1.0
        
        self.sm.execute()
        
        self.assertEqual(self.sm.linear_speed, 0.0, "Sollte stoppen")
        self.assertEqual(self.sm.state, 20, "Bleibt in State 20")
    
    # ==================== State 30: Abschluss Tests ====================
    
    def test_state30_completion(self):
        """State 30: Follow abgeschlossen"""
        self.sm.state = 30
        
        self.sm.execute()
        
        self.assertTrue(self.sm.follow_done)
        self.assertEqual(self.sm.linear_speed, 0.0)
        self.assertEqual(self.sm.angular_speed, 0.0)
    
    # ==================== P-Controller Tests ====================
    
    def test_pcontroller_angular_zero_error(self):
        """Angular P-Controller: Null-Fehler"""
        result = self.sm.pcontroller_angular(0.0)
        self.assertEqual(result, 0.0)
    
    def test_pcontroller_angular_positive_error(self):
        """Angular P-Controller: Positive Fehler"""
        error = math.radians(5)
        result = self.sm.pcontroller_angular(error)
        
        expected = -KP_ANGULAR * error
        # Prüfe ob innerhalb der Grenzen
        self.assertLessEqual(abs(result), self.sm.align_angular_speed)
    
    def test_pcontroller_angular_saturation(self):
        """Angular P-Controller: Sättigung"""
        large_error = math.radians(50)
        result = self.sm.pcontroller_angular(large_error)
        
        self.assertEqual(abs(result), self.sm.align_angular_speed)
    
    def test_pcontroller_linear_zero_error(self):
        """Linear P-Controller: Null-Fehler"""
        result = self.sm.pcontroller_linear(0.0)
        self.assertEqual(result, 0.0)
    
    def test_pcontroller_linear_positive_error(self):
        """Linear P-Controller: Zu weit entfernt"""
        distance_error = 0.5  # 50 cm zu weit
        result = self.sm.pcontroller_linear(distance_error)
        
        expected = self.sm.kp_linear * distance_error
        self.assertAlmostEqual(result, expected, places=5)
        self.assertGreater(result, 0, "Sollte vorwärts fahren")
    
    def test_pcontroller_linear_negative_error(self):
        """Linear P-Controller: Zu nah"""
        distance_error = -0.3  # 30 cm zu nah
        result = self.sm.pcontroller_linear(distance_error)
        
        expected = self.sm.kp_linear * distance_error
        self.assertAlmostEqual(result, expected, places=5)
        self.assertLess(result, 0, "Sollte rückwärts fahren")
    
    def test_pcontroller_linear_saturation_positive(self):
        """Linear P-Controller: Positive Sättigung"""
        large_error = 5.0  # Sehr weit entfernt
        result = self.sm.pcontroller_linear(large_error)
        
        self.assertEqual(result, self.sm.max_linear_speed)
    
    def test_pcontroller_linear_saturation_negative(self):
        """Linear P-Controller: Negative Sättigung"""
        large_error = -5.0  # Sehr nah
        result = self.sm.pcontroller_linear(large_error)
        
        self.assertEqual(result, -self.sm.max_linear_speed)
    
    # ==================== Hilfsfunktionen Tests ====================
    
    def test_setSpeedPara(self):
        """Teste setSpeedPara Funktion"""
        self.sm.setSpeedPara(0.15, 0.25)
        
        self.assertEqual(self.sm.linear_speed, 0.15)
        self.assertEqual(self.sm.angular_speed, 0.25)
    
    def test_distance_to_robot_calculation(self):
        """Teste Umrechnung target_distance zu distance_to_robot"""
        self.sm.target_distance = 50.0  # 50 cm
        self.sm.execute()
        
        self.assertEqual(self.sm.distance_to_robot, 0.5, "50 cm = 0.5 m")
        
        self.sm.target_distance = 100.0  # 100 cm
        self.sm.execute()
        self.assertEqual(self.sm.distance_to_robot, 1.0, "100 cm = 1.0 m")
    
    # ==================== Integrations-Tests ====================
    
    def test_full_follow_sequence(self):
        """Integrations-Test: Komplette Follow-Sequenz"""
        # State 10: Suche
        self.assertEqual(self.sm.state, 10)
        self.sm.id = -1
        self.sm.execute()
        self.assertEqual(self.sm.state, 10)
        
        # Roboter gefunden
        self.sm.id = 69
        self.sm.distance = 1.0
        self.sm.execute()
        self.assertEqual(self.sm.state, 20)
        
        # Follow läuft
        self.sm.angle = 5.0
        self.sm.target_distance = 50.0
        for _ in range(5):
            self.sm.execute()
            self.assertEqual(self.sm.state, 20)
        
        # Zu State 30
        self.sm.state = 30
        self.sm.execute()
        self.assertTrue(self.sm.follow_done)
    
    def test_approach_target_distance(self):
        """Test: Annäherung an Ziel-Distanz"""
        self.sm.state = 20
        self.sm.id = 69
        self.sm.angle = 0.0
        self.sm.target_distance = 50.0  # 50 cm Ziel
        
        # Start bei 2m Entfernung
        distances = [2.0, 1.5, 1.0, 0.7, 0.55, 0.5]
        
        for dist in distances:
            self.sm.distance = dist
            self.sm.execute()
            
            if dist > 0.5:
                self.assertGreater(self.sm.linear_speed, 0, f"Bei {dist}m sollte vorwärts fahren")


class TestFollowStateMachineEdgeCases(unittest.TestCase):
    """Grenzfall-Tests für FollowStateMachine"""
    
    def setUp(self):
        self.sm = FollowStateMachine()
    
    def test_rapid_marker_changes(self):
        """Test bei schnellen Marker-Wechseln"""
        self.sm.state = 20
        self.sm.distance = 1.0
        self.sm.target_distance = 50.0
        
        # Wechsel zwischen 69 und -1
        for i in range(10):
            self.sm.id = 69 if i % 2 == 0 else -1
            self.sm.angle = 0.0
            self.sm.execute()
    
    def test_extreme_target_distances(self):
        """Test mit extremen Ziel-Distanzen"""
        self.sm.state = 20
        self.sm.id = 69
        self.sm.angle = 0.0
        self.sm.distance = 1.0
        
        # Sehr kleine Ziel-Distanz
        self.sm.target_distance = 20.0  # 20 cm
        self.sm.execute()
        self.assertNotEqual(self.sm.linear_speed, 0.0)
        
        # Sehr große Ziel-Distanz
        self.sm.target_distance = 100.0  # 100 cm
        self.sm.execute()
    
    def test_extreme_angles(self):
        """Test mit extremen Winkeln"""
        self.sm.state = 20
        self.sm.id = 69
        self.sm.distance = 1.0
        self.sm.target_distance = 50.0
        
        # 90° Fehler
        self.sm.angle = 90.0
        self.sm.execute()
        self.assertEqual(self.sm.angular_speed, -self.sm.align_angular_speed)
        
        # -90° Fehler
        self.sm.angle = -90.0
        self.sm.execute()
        self.assertEqual(self.sm.angular_speed, self.sm.align_angular_speed)


if __name__ == '__main__':
    unittest.main()
