import math

class AlignStateMachine:
    def __init__(self, logger=None):
        self.state = 10
        self.logger = logger
        self.align_done = False
        self.error_rad = None
        self.distance = 0.0
        self.id = -1
        self.angle = 0.0  
        self.id_to_turn = 0
        self.kp_angular = 0.3
        self.align_angular_speed = 0.05
        self.search_angular_speed = 0.2
        self.angle_tolerance = 6
        self.target_distance = 0.25
        self.linear_speed = 0.0   
        self.angular_speed = 0.0
        self.marker_lost_counter = 0
        self.distance_farMarker = 1.0
        
            
    def setpgain(self, kp_angular, max_angular_speed):
        """Setze Regelungsparameter"""
        self.kp_angular = kp_angular
        self.align_angular_speed = max_angular_speed  

    def pcontroller(self, error_rad):
        """P-Regler mit progressiver Geschwindigkeit"""
        factor = abs(error_rad) * 5
        control = self.kp_angular * error_rad * factor
        return control
    
    def setSpeedPara(self, linear, angular):
        self.linear_speed = linear
        self.angular_speed = angular
    
    def execute(self):
        match self.state:
            case 10:  
                self.linear_speed = 0.0
                self.marker_lost_counter = 0
                
                if self.id == self.id_to_turn and self.distance < self.distance_farMarker:  
                    self.state = 20
                    if self.logger:
                        self.logger.info(f"Marker ID 0 gefunden bei {self.distance:.2f}m, starte Alignment")
                else:
                    self.angular_speed = self.search_angular_speed  
            
            case 20: 
                self.linear_speed = 0.0
                self.angular_speed = self.align_angular_speed
                
                if self.id == self.id_to_turn: 
                    if self.logger:
                        self.logger.info(f"AlignStateMachine: Aligning to marker ID {self.id} at distance {self.distance:.2f}m and angle {self.angle:.2f}°")
                    angle_rad = math.radians(self.angle)  
                    self.angular_speed = self.pcontroller(angle_rad)
                    
                    if abs(self.angle) < self.angle_tolerance:
                        self.state = 30
                        if self.logger:
                            self.logger.info(f"Align fertig auf Marker 0 bei Distanz {self.distance:.2f}m und Winkel {self.angle:.2f}°")
                  
                else:  # Anderer Marker
                    self.state = 10
                    if self.logger:
                        self.logger.info(f"Falscher Marker {self.id}, zurück zur Suche")
            
            case 30:  
                self.align_done = True
                self.linear_speed = 0.0
                self.angular_speed = 0.0
                if self.logger:
                    self.logger.info(f"AlignStateMachine: Align fertig")
                    