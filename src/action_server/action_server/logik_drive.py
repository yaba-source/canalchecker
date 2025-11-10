import math
from types import SimpleNamespace


class drive_straight:
    def __init__(self,linear_speed=0.3,angular_speed=0.1):
        self.linear_speed = linear_speed
        self.angular_speed = angular_speed

        self.start_theta = None
        self.current_theta = 0.0

        self.linear_x = 0.0
        self.angular_z = 0.0
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0
        self.odom_recieved = False

    def setPosPara(self,x,y,th):  
        self.current_x = x
        self.current_y = y
        self.current_theta = th
        if self.start_theta is None:
            self.start_theta = th
        self.odom_received = True
        
    def navigate_straight(self):
        
        angle_error=self.current_theta-self.start_theta
        if angle_error>0.1:
            self.angular_z = self.angular_speed * angle_error
        if angle_error<-0.1:
            self.angular_z = self.angular_speed * angle_error


        

        self.linear_x = self.linear_speed