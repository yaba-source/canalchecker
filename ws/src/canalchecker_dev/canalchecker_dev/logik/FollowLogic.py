import math

KP_ANGULAR = 2

class FollowStateMachine:
    def __init__(self, logger=None):
        self.state = 10
        self.logger = logger
        self.follow_done = False
        self.error_rad = None
        self.distance = 0.0
        self.id = -1
        self.angle = 0.0  
        self.id_to_follow = 69
        self.kp_angular = 1.1
        self.align_angular_speed = 0.05
        self.angle_tolerance = 3
        self.linear_speed = 0.0   
        self.angular_speed = 0.0
        self.marker_lost_counter = 0
       
        self.distance_to_robot = 0.5  
        self.target_distance = 50.0   
        self.max_speed = 0.2          
        
        self.distance_tolerance = 0.1
        self.kp_linear = 0.2
        self.max_linear_speed = 0.2
        self.robot_found = True

    
    def pcontroller_angular(self, error_rad):
        """P-Regler mit progressiver Geschwindigkeit"""
        control = KP_ANGULAR * error_rad 
        max_speed = self.align_angular_speed
        if control > max_speed:
            control = max_speed
        elif control < -max_speed:
            control = -max_speed

        return -control
    
    def pcontroller_linear(self, distance_error):
        """P-Regler für Abstandsregelung"""
        control = self.kp_linear * distance_error
        if control > self.max_linear_speed:
            control = self.max_linear_speed
        elif control < -self.max_linear_speed:
            control = -self.max_linear_speed
        return control
    
    def setSpeedPara(self, linear, angular):
        self.linear_speed = linear
        self.angular_speed = angular
    
    def execute(self):
        
        self.distance_to_robot = self.target_distance / 100.0
        
        if self.distance < self.distance_to_robot:
            self.linear_speed = 0.0
            return
        match self.state:
            case 10:  
                if self.logger:
                    self.logger.info("State 10: Suche nach Roboter")
                if self.id == self.id_to_follow:
                    self.state = 20
                    if self.logger:
                        self.logger.info(f"Marker ID {self.id_to_follow} gefunden bei {self.distance:.2f}m, starte Alignment")
                else:
                    self.robot_found = False
                    
            case 20:
                if self.logger:
                    self.logger.info("State 20: Folge Roboter")
                if self.id == self.id_to_follow:
                    self.robot_found = True
                    self.marker_lost_counter = 0

                    angle_rad = math.radians(self.angle)
                    self.angular_speed = self.pcontroller_angular(angle_rad)
                    
                    # Abstandsregelung mit P-Regler
                    distance_error = self.distance - self.distance_to_robot
                    self.linear_speed = self.pcontroller_linear(distance_error)

                else:
                    self.logger.info("Roboter verloren, suche erneut.")
                    self.linear_speed = 0.0
            case 30:
                if self.logger:
                    self.logger.info("State 30: Kein Roboter vorhanden, fahre mit Align fort")
                self.follow_done = True
                self.linear_speed = 0.0
                self.angular_speed = 0.0