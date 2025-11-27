


class AlignStateMachine:
    def __init__(self,logger=None):
        self.state =10
        self.error=None
        self.logger=logger
        self.align_done = False
        self.distance
        self.id
        self.angle
        self.detected
        self.id_to_turn=0
        
    def setSpeedPara(self,linear,angular):
        self.linear_speed=linear
        self.angular_speed=angular

    def execute(self):

        match self.state:
            case 10:# drehen bis marker auf der andern seite gefunden
                if self.id == self.id_to_turn and self.distance < 0.30 and self.detected:
                    self.angular_speed=0.2
                    if self.angule<0.5:
                        self.angular_speed
            case 20:
                if self.id == self.id_to_turn and self.distance > 0.30 and self.detected:
                    self.state = 30
                else:
                    self.state = 20

           
                    
            

            case 30: #Align fertig

