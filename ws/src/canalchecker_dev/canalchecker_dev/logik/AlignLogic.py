class AlignStateMachine:
    def __init__(self,logger=None):
        self.state =10
        self.error=None
        self.logger=logger
        self.align_done = False
        self.error_angle=None
        self.distance
        self.id
        self.angle
        self.id_to_turn=0
        self.kp_angular=1.2
        self.max_angular_speed=0.2
        self.angletolerance=0.05
        self.taregt_distance=0.25

    def setpgain(self,kp_angular,max_angular_speed):
        self.kp_angular=kp_angular
        self.max_angular_speed=max_angular_speed
        

    def pcontroller(self,error):
        control=self.kp_angular*error
        control=max(-self.max_angular_speed,min(self.max_angular_speed,control))
        return control

    
    def setSpeedPara(self,linear,angular):
        self.linear_speed=linear
        self.angular_speed=angular

    def execute(self):

        match self.state:
            case 10:# drehen bis marker auf der andern seite gefunden
                if self.id == self.id_to_turn and self.distance < self.taregt_distance:
                    self.state = 20
                    self.linear_speed=0
                    if self.logger:
                            self.logger.info(f"Marker {self.id} gefunden bei Distanz {self.distance:.2f}m")
                else:
                    self.angular_speed=self.max_angular_speed

                    
            case 20:#align on marker
                if self.id == self.id_to_turn and self.distance > self.taregt_distance :
                    self.angular_speed = self.pcontroller(self.angle)
                    if abs(self.angle) < self.angletolerance:
                        self.state = 30
                        if self.logger:
                            self.logger.info(f"Align auf Marker {self.id} fertig bei Distanz {self.distance:.2f}m und Winkel {self.angle:.2f}°")
                else:
                    self.state =10
                    if self.logger:
                            self.logger.info(f"Marker {self.id} verloren, suche neu")
                        
            case 30: #Align fertig
                self.align_done = True
                self.linear_speed=0
                self.angular_speed=0    
                if self.logger:
                    self.logger.info(f"AlignStateMachine: Align fertig")

