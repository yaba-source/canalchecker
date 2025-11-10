import math
import pytest
from types import SimpleNamespace

class navigate_to_goal:
    def __init__(self,linear_speed=0.2,angular_speed=0.2):
        self.distance: float = 0.0  
        self.angle_diff: float = 0.0
        self.position_tolerance: float = 0.05
        self.angle_tolerance: float = 0.05
        self.angular_speed: float = angular_speed
        self.linear_speed:float=linear_speed
        self.target = SimpleNamespace(x=None, y=None, theta=None)
        self.position = SimpleNamespace(x=None, y=None, theta=None)

        self.finish = False  
        self.success = False
        self.in_angle = False
        self.odom_recived = False
        self.target_recived = False
        self.Step = 10  

        self.linear_x = 0.0
        self.linear_y = 0.0
        self.angular_z = 0.0
    
    def setTargetPara(self, x, y, th): 
        self.target.x = x
        self.target.y = y
        self.target.theta = th
        self.target_recived = True
    
    def setPosPara(self, x, y, th):  
        self.position.x = x
        self.position.y = y
        self.position.theta = th
        self.current_x = x              
        self.current_y = y               
        self.current_theta = th
        self.odom_recived = True

    def setSpeedPara(self, vx, vy, wz):
        pass
    
    
    def calculate_distance(self):
        """Berechnet Distanz zum Ziel"""
        dx = self.target.x - self.position.x
        dy = self.target.y - self.position.y  
        return math.sqrt(dx**2 + dy**2)
    
    def calculate_target_angle(self):
        """Berechnet den Zielwinkel zum gewünschten Punkt"""
        dx = self.target.x - self.position.x
        dy = self.target.y - self.position.y
        return math.atan2(dy, dx)
    
    def normalize_angle(self, angle): 
        """Normalisiert Winkel auf [-pi, pi]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def resetFinish(self):  
        self.finish = False
        self.success = False
        self.Step = 10
    
    def navigate_to_target(self): 
        
        if not (self.odom_recived and self.target_recived):
            print (f'waiting for data odom={self.odom_recived}, target={self.target_recived}')
            return  

        if self.position.x is None or self.target.x is None:
            print ('postion or target is none')
            return  
            # Werte in jedem Durchlauf neu berechnen
        self.distance = self.calculate_distance()
        self.target_angle = self.calculate_target_angle()
        self.angle_diff = self.normalize_angle(self.target_angle - self.position.theta)
            
        match self.Step:
            case 10:  # Initiale Prüfung
                if self.distance > self.position_tolerance:
                    self.Step = 30
                else:
                    self.Step = 60
                    print("near to target")

            case 30:  # Drehen
                if abs(self.angle_diff) > self.angle_tolerance:
                    self.angular_z = 0.3
                    self.linear_x = 0.0  
                else:
                    self.angular_z = 0.0
                    self.Step = 40
                
            case 40:  # Fahren
                if self.distance > self.position_tolerance:
                    
                    self.linear_x = self.linear_speed
                    self.angular_z = 1.0 * self.angle_diff  

                    
                    if self.angular_z > 0.4:
                        self.angular_z = 0.4
                    elif self.angular_z < -0.4:
                        self.angular_z = -0.4
                else:
                    self.linear_x = 0.0
                    self.angular_z = 0.0
                    self.Step=60

        
                
            case 60:  # Ziel erreicht
                    print('Am Ziel')
                    self.linear_x = 0.0
                    self.angular_z = 0.0
                    self.finish = True
                    self.success = True