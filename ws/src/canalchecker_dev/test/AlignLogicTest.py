"""
Unit Tests für AlignStateMachine.
Testet alle States, Übergänge und Grenzfälle.
"""

import unittest
import math
import sys
import os

# Füge Parent-Directory zum Python Path hinzu
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from canalchecker_dev.logik.AlignLogic import AlignStateMachine


class TestAlignStateMachine(unittest.TestCase):
    """Test Suite für AlignStateMachine."""
    
    def setUp(self):
        """Wird vor jedem Test ausgeführt."""
        self.sm = AlignStateMachine()
    
    def tearDown(self):
        """Wird nach jedem Test ausgeführt."""
        self.sm = None
    
    def test_initial_state(self):
        """Teste initiale State Machine Werte."""
        self.assertEqual(self.sm.state, 10)
        self.assertFalse(self.sm.align_done)
        self.assertEqual(self.sm.id, -1)
    
    def test_initialization_with_logger(self):
        """Teste Initialisierung mit Logger."""
        class MockLogger:
            def info(self, msg):
                pass
        
        sm = AlignStateMachine(logger=MockLogger())
        self.assertIsNotNone(sm.logger)
    
    def test_state10_search_rotation(self):
        """State 10: Roboter rotiert bei Suche."""
        self.sm.state = 10
        self.sm.id = -1
        
        self.sm.execute()
        
        self.assertEqual(self.sm.state, 10)
        self.assertEqual(self.sm.linear_speed, 0.0)
        self.assertEqual(self.sm.angular_speed, 0.2)
    
    def test_state10_wrong_marker(self):
        """State 10: Ignoriert falschen Marker."""
        self.sm.state = 10
        self.sm.id = 5
        self.sm.distance = 1.0
        
        self.sm.execute()
        
        self.assertEqual(self.sm.state, 10)
    
    def test_state10_marker_found_transition(self):
        """State 10: Übergang zu State 20 wenn Marker 0 gefunden."""
        self.sm.state = 10
        self.sm.id = 0
        self.sm.distance = 1.0
        
        self.sm.execute()
        
        self.assertEqual(self.sm.state, 20)
    
    def test_state10_marker_too_close(self):
        """State 10: Marker zu nah, bleibt in State 10."""
        self.sm.state = 10
        self.sm.id = 0
        self.sm.distance = 0.3
        
        self.sm.execute()
        
        self.assertEqual(self.sm.state, 10)
    
    def test_state10_marker_lost_counter_reset(self):
        """State 10: marker_lost_counter wird zurückgesetzt."""
        self.sm.state = 10
        self.sm.marker_lost_counter = 5
        
        self.sm.execute()
        
        self.assertEqual(self.sm.marker_lost_counter, 0)
    
    def test_state20_alignment_positive_angle(self):
        """State 20: Korrektur bei positivem Winkel."""
        self.sm.state = 20
        self.sm.id = 0
        self.sm.distance = 1.0
        self.sm.angle = 10.0
        
        self.sm.execute()
        
        self.assertEqual(self.sm.state, 20)
        self.assertLess(self.sm.angular_speed, 0)
    
    def test_state20_alignment_negative_angle(self):
        """State 20: Korrektur bei negativem Winkel."""
        self.sm.state = 20
        self.sm.id = 0
        self.sm.distance = 1.0
        self.sm.angle = -10.0
        
        self.sm.execute()
        
        self.assertGreater(self.sm.angular_speed, 0)
    
    def test_state20_alignment_complete(self):
        """State 20: Übergang zu State 30 bei Alignment."""
        self.sm.state = 20
        self.sm.id = 0
        self.sm.distance = 1.0
        self.sm.angle = 2.0
        
        self.sm.execute()
        
        self.assertEqual(self.sm.state, 30)
    
    def test_state20_alignment_at_tolerance_boundary(self):
        """State 20: Grenzwert-Test bei genau 3 Grad."""
        self.sm.state = 20
        self.sm.id = 0
        self.sm.distance = 1.0
        self.sm.angle = 3.0
        
        self.sm.execute()
        
        self.assertEqual(self.sm.state, 20)
        
        self.sm.angle = 2.9
        self.sm.execute()
        self.assertEqual(self.sm.state, 30)
    
    def test_state20_marker_lost_increments_counter(self):
        """State 20: Marker verloren erhöht Counter."""
        self.sm.state = 20
        self.sm.id = -1
        self.sm.distance = 1.0
        self.sm.marker_lost_counter = 0
        
        for i in range(3):
            self.sm.execute()
            self.assertEqual(self.sm.marker_lost_counter, i + 1)
    
    def test_state20_marker_lost_timeout(self):
        """State 20: Zurück zu State 10 nach 5 Frames."""
        self.sm.state = 20
        self.sm.id = -1
        self.sm.distance = 1.0
        self.sm.marker_lost_counter = 4
        
        self.sm.execute()
        self.assertEqual(self.sm.marker_lost_counter, 5)
        
        self.sm.execute()
        self.assertEqual(self.sm.state, 10)
    
    def test_state20_wrong_marker(self):
        """State 20: Falscher Marker erhöht Counter."""
        self.sm.state = 20
        self.sm.id = 5
        self.sm.distance = 1.0
        
        self.sm.execute()
        
        self.assertGreater(self.sm.marker_lost_counter, 0)
    
    def test_state30_completion(self):
        """State 30: Alignment abgeschlossen."""
        self.sm.state = 30
        
        self.sm.execute()
        
        self.assertTrue(self.sm.align_done)
        self.assertEqual(self.sm.linear_speed, 0.0)
        self.assertEqual(self.sm.angular_speed, 0.0)
    
    def test_pcontroller_zero_error(self):
        """P-Controller: Kein Fehler gibt 0 zurück."""
        result = self.sm.pcontroller(0.0)
        self.assertEqual(result, 0.0)
    
    def test_pcontroller_positive_error(self):
        """P-Controller: Positive Fehler."""
        error = math.radians(5)
        result = self.sm.pcontroller(error)
        
        expected = -self.sm.kp_angular * error
        self.assertAlmostEqual(result, expected, places=5)
    
    def test_pcontroller_negative_error(self):
        """P-Controller: Negative Fehler."""
        error = math.radians(-5)
        result = self.sm.pcontroller(error)
        
        expected = -self.sm.kp_angular * error
        self.assertAlmostEqual(result, expected, places=5)
    
    def test_pcontroller_saturation_positive(self):
        """P-Controller: Positive Sättigung."""
        large_error = math.radians(50)
        result = self.sm.pcontroller(large_error)
        
        self.assertEqual(result, -self.sm.align_angular_speed)
    
    def test_pcontroller_saturation_negative(self):
        """P-Controller: Negative Sättigung."""
        large_error = math.radians(-50)
        result = self.sm.pcontroller(large_error)
        
        self.assertEqual(result, self.sm.align_angular_speed)
    
    def test_setSpeedPara(self):
        """Teste setSpeedPara Funktion."""
        self.sm.setSpeedPara(0.5, 0.3)
        
        self.assertEqual(self.sm.linear_speed, 0.5)
        self.assertEqual(self.sm.angular_speed, 0.3)
    
    def test_full_alignment_sequence(self):
        """Integrations-Test: Komplette Sequenz."""
        self.assertEqual(self.sm.state, 10)
        self.sm.id = -1
        self.sm.execute()
        self.assertEqual(self.sm.state, 10)
        
        self.sm.id = 0
        self.sm.distance = 1.0
        self.sm.execute()
        self.assertEqual(self.sm.state, 20)
        
        self.sm.angle = 10.0
        for _ in range(5):
            self.sm.execute()
            self.assertEqual(self.sm.state, 20)
        
        self.sm.angle = 1.0
        self.sm.execute()
        self.assertEqual(self.sm.state, 30)
        
        self.sm.execute()
        self.assertTrue(self.sm.align_done)


class TestAlignStateMachineEdgeCases(unittest.TestCase):
    """Grenzfall-Tests für AlignStateMachine."""
    
    def setUp(self):
        """Setup für Tests."""
        self.sm = AlignStateMachine()
    
    def test_rapid_marker_id_changes(self):
        """Test bei schnellen Marker-ID-Wechseln."""
        self.sm.state = 20
        self.sm.distance = 1.0
        
        for i in range(10):
            self.sm.id = 0 if i % 2 == 0 else -1
            self.sm.execute()
        
        self.assertEqual(self.sm.state, 10)
    
    def test_distance_variations(self):
        """Test mit variierenden Distanzen."""
        self.sm.state = 10
        self.sm.id = 0
        
        self.sm.distance = 0.1
        self.sm.execute()
        self.assertEqual(self.sm.state, 10)
        
        self.sm.distance = 2.0
        self.sm.execute()
        self.assertEqual(self.sm.state, 20)
    
    def test_extreme_angles(self):
        """Test mit extremen Winkeln."""
        self.sm.state = 20
        self.sm.id = 0
        self.sm.distance = 1.0
        
        self.sm.angle = 180.0
        self.sm.execute()
        self.assertEqual(self.sm.angular_speed, -self.sm.align_angular_speed)
        
        self.sm.angle = -180.0
        self.sm.execute()
        self.assertEqual(self.sm.angular_speed, self.sm.align_angular_speed)


if __name__ == '__main__':
    unittest.main()
