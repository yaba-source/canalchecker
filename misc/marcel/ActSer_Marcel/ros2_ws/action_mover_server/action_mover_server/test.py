import math
import pytest
from types import SimpleNamespace
class navigate_to_goal :
    def __init__(self):
        self.distancedistance: float = 0.0
        self.angle_diff: float = 0.0
        self.position_tolerance: float = 0.3
        self.angle_tolerance: float = 0.3
        self.linear_speed: float = 0.4
        self.angular_speed: float = 0.5
        self.target = SimpleNamespace(x=None, y=None, theta=None)
        self.position = SimpleNamespace(x=None, y=None, theta=None)
        self.los= False
        self.in_angle = False
        self.Step = 0
        self.odom_recived = False
        self.target_recived = False
        def setTargetPara(x, y, th,self):
            self.position.x = x
            self.position.y = y
            self.position.theta = th

        def setPosPara(x, y, th,self):
            self.position.x = x
            self.position.y = y
            self.position.theta = th

        def setSpeedPara(lin_x, lin_y, ang_z,self):
            self.linear_x = lin_x
            self.angular_z = ang_z

        def calculate_distance(self):
            """Berechnet Distanz zum Ziel"""
            dx = self.target.x - self.position.x
            dy = self.target.y - self.position.y  
            return math.sqrt(dx**2 + dy**2)

        def calculate_target_angle(self):
            """Berechnet den Zielwinkel zum gewÃ¼nschten Punkt"""
            dx = self.target.x - self.position.x
            dy = self.target.y - self.position.y
            return math.atan2(dy, dx)

        def normalize_angle(angle,self):  
            """Normalisiert Winkel auf [-pi, pi]"""
            while self.angle > math.pi:
                self.angle -= 2 * math.pi
            while self.angle < -math.pi:
                self.angle += 2 * math.pi
            return angle
        
        def resetfinish():
            self.target.x=self.position.x
            self.target.y=self.position.y
            self.target.z=self.position.z

      

        def navigate_to_target(self): 
            
            
            if self.odom_recived == True:
                
                    
                    # Werte in jedem Durchlauf neu berechnen
                    self.distance = self.calculate_distance()
                    self.target_angle = self.calculate_target_angle()
                    self.angle_diff = self.normalize_angle(self.target_angle - self.position.theta)  # Korrigiert
                    
                    match self.Step:
                        
                           
                        case 10:
                            if self.distance > self.position_tolerance:
                                self.Step = 300
                            else:
                                self.Step = 60
                                
                        
                            
                        case 30:  # drehen
                            if abs(self.angle_diff) > self.angle_tolerance:
                                self.angular_z = math.copysign(self.angular_speed, self.angle_diff)
                                self.linear_x = 0.0  
                            else:
                                self.angular_z = 0.0
                                self.Step = 40
                                
                        case 40:#Fahren
                            if self.distance > self.position_tolerance:
                                self.linear_x = 0.4  
                                self.angular_z = 0.0
                            else:
                                Step = 60
                                
                        case 60: #Ziel 
                            print('Am Ziel')
                            self.linear_x = 0.0
                            self.angular_z = 0.0
                            
                            resetfinish()