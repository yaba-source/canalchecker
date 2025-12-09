ANGULAR_Z_STEPSIZE = 0.01
ANGULAR_Z_MAX = 0.2

MAX_LINEAR_SPEED = 0.08

class DriveStateMachine:
    """
    Interne StateMachine des Driveservers.
    """
    def __init__(self):
        self.state = 1
        self.error = None
        self.drive_complete = False
        self.max_linear_speed = MAX_LINEAR_SPEED
        self.max_angular_speed = ANGULAR_Z_MAX
        self.id = -1
        self.distance = 0               # Distance in m
        self.angle = 0                  # Angle in degrees
        self.angle_tolerance = 3     # Tolerance in degrees
        self.angular_cmd = 0            # angular speed to be commanded

    def target_actual_comparison(self, target: float, actual: float):
        """
        Führt einen Soll / Ist vergleich zwischen zwei werten durch und modifiziert den zu kommandierenden winkel sowie geschwindigkeit entsprechend.
        
        :param target: Zielwert welcher erreicht werden soll
        :type target: float
        :param actual: Wert welcher eigentlich aktuell ausgelesen wird
        :type actual: float
        """

        if target > actual:
            self.angular_cmd += ANGULAR_Z_STEPSIZE
            if abs(self.angular_cmd) > ANGULAR_Z_MAX:
                self.angular_cmd = ANGULAR_Z_MAX
                #self.max_linear_speed = 0.04
        elif target < actual:
            self.angular_cmd -= ANGULAR_Z_STEPSIZE
            if abs(self.angular_cmd) > ANGULAR_Z_MAX:
                self.angular_cmd = -ANGULAR_Z_MAX
                #self.max_linear_speed = 0.04
        
        if abs(self.angular_cmd) < ANGULAR_Z_MAX:
            self.max_linear_speed = MAX_LINEAR_SPEED
        
        # Debug Statement
        print("angular_cmd: ",self.angular_cmd)
    
    def execute(self):
        match self.state:
            case 1:
                print('[DEBUG] StateMachine Case 1. Driving')
                if self.distance < 0.45:
                    self.state = 2
                if abs(self.angle) > self.angle_tolerance:
                    self.target_actual_comparison(0.0, self.angle)
            case 2:
                print('[DEBUG] StateMachine Case 2. Finished')
                self.drive_complete = True