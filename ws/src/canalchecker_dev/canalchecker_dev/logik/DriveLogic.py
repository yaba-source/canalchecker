import math

KP = 2                  # Reglerverstärkung
ANGULAR_Z_MAX = 0.2     # Maximale Winkelgeschwindigkeit
ANGLE_TOLERABLE_ERR = 3 # Tolerierbarer Winkelfehler (Unterhalb diesem wert keine Regelung)

class DriveStateMachine:
    """
    ## Description
    Basisklasse der Statemachine des Driveservers.
    Implementiert den P-Regler sowie das match-case der Statemachine.
    """
    def __init__(self):
        self.state = 1                          # Aktueller State der statemachine
        self.error = None                       # Wahrscheinlich YAGNI
        self.drive_complete = False             # Ziemlich selbsterklärend?
        self.id = -1                            # ID des erkannten ArUco markers
        self.distance = 0                       # Distanz in m
        self.angle = 0                          # Winkel in grad
        self.angular_speed = 0.0                  # Zu kommandierende Winkelgeschwindigkeit
        self.linear_speed=0.0  
        self.max_linear_speed= 0.0            


    def p_controller(self, target: float, actual: float):
        """
        ## Description
        Implementiert eine P-Regelung des Winkels.
        Berechnet zuerst einen Fehler, den unterschied zwischen dem Ziel (target) und tatsächlichem (actual) winkel.
        Sowohl 'target' als auch 'actual' werden in der einheit [rad] übergeben.
        Der Fehler wird mit einer verstärkung 'KP' multipliziert und mittels 'angular_cmd' auf '/cmd_vel' gepublisht.
        Sollte 'angular_cmd' größer als die Begrenzung sein, wird er auf diesen begrenzt ("Sättigung").

        :param target: Zielwert welcher erreicht werden soll
        :type target: float
        :param actual: Wert welcher eigentlich aktuell ausgelesen wird
        :type actual: float
        :return None:
        """

        error = target - actual

        self.angular_speed = (KP * error)

        if self.angular_speed > ANGULAR_Z_MAX:
            self.angular_speed = ANGULAR_Z_MAX
        elif self.angular_speed< -ANGULAR_Z_MAX:
            self.angular_speed = -ANGULAR_Z_MAX
        return self.angular_speed
    
    def execute(self):
        """
        ## Description
        Statemachine des DriveServers.
        Case 1: Überprüft ob die distanz unterhalb des zielwerts ist und wechselt dann in case 2. Implementiert außerdem die regelung.
        Case 2: Wird nur aufgerufen wenn die Fahrt fertig ist.
        
        :param None:
        :return None:
        """
        if self.max_linear_speed == 0.0:
            self.linear_speed = 0.0
            self.angular_speed = 0.0
            return
        match self.state:
            case 1:
            
                if self.distance < 0.5 and not self.id == -1:
                    self.state = 2
                    self.linear_speed =0.0
                    self.angular_speed=0.0
                else:
                    self.linear_speed=self.max_linear_speed
                if abs(self.angle) > ANGLE_TOLERABLE_ERR:
                    self.p_controller(0.0, math.radians(self.angle))
                elif abs(self.angle) < ANGLE_TOLERABLE_ERR:
                    self.angular_speed = 0.0
            case 2:
    
                self.drive_complete = True
                self.linear_speed = 0.0
