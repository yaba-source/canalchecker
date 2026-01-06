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
        self.kp_angular = 1.1
        self.align_angular_speed = 0.05
        self.max_speed = 0.2
        self.search_angular_speed = self.max_speed
        self.angle_tolerance = 3
        self.linear_speed = 0.0   
        self.angular_speed = 0.0
        self.marker_lost_counter = 0
        self.distance_far_marker= 0.5
       
    
    def pcontroller(self, error_rad):
        """P-Regler mit progressiver Geschwindigkeit"""
        control = self.kp_angular * error_rad 
        self.max_speed = self.align_angular_speed
        if control > self.max_speed:
            control = self.max_speed
        elif control < -self.max_speed:
            control = -self.max_speed

        return -control
    
    def setSpeedPara(self, linear, angular):
        self.linear_speed = linear
        self.angular_speed = angular
    
    def execute(self):
        if self.max_speed == 0.0:
            self.linear_speed = 0.0
            self.angular_speed = 0.0
            return
        match self.state:
            case 10:  
                self.linear_speed = 0.0
                self.marker_lost_counter = 0
                
                if self.id == self.id_to_turn and self.distance > self.distance_far_marker:  
                    self.state = 20
                    if self.logger:
                        self.logger.info(f"Marker ID 0 gefunden bei {self.distance:.2f}m, starte Alignment")
                else:
                    self.angular_speed = self.search_angular_speed  
            
            case 20:
                if self.id == self.id_to_turn and self.distance > self.distance_far_marker:
                    self.linear_speed = 0.0
                    self.marker_lost_counter = 0

                    angle_rad = math.radians(self.angle)
                    self.angular_speed = self.pcontroller(angle_rad)

                    if abs(self.angle) < self.angle_tolerance:
                        self.state = 30
                        if self.logger:
                            self.logger.info(
                                f"Align fertig auf Marker 0 bei Distanz {self.distance:.2f}m und Winkel {self.angle:.2f}°"
                            )
                else:
                   
                    self.marker_lost_counter += 1
                    if self.marker_lost_counter > 5:
                        self.state = 10
                        if self.logger:
                            self.logger.info(f"Falscher/kein Marker {self.id}, zurück zur Suche")

            case 30:  
                self.align_done = True
                self.linear_speed = 0.0
                self.angular_speed = 0.0
                if self.logger:
                    self.logger.info(f"AlignStateMachine: Align fertig")
                    