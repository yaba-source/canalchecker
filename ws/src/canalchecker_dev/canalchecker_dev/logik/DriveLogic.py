ANGULAR_Z_STEPSIZE = 0.1
ANGULAR_Z_MAX = 0.2

class DriveStateMachine:
    """
    Interne StateMachine des Driveservers.
    """
    def __init__(self):
        self.state = 1
        self.error = None
        self.drive_complete = False
        self.max_linear_speed = 0.08
        self.max_angular_speed = 0.2
        self.id = -1
        self.distance = 0               # Distance in m
        self.angle = 0                  # Angle in degrees
        self.angle_tolerance = 3     # Tolerance in degrees
        self.angular_cmd = 0            # angular speed to be commanded

    def target_actual_comparison(self, target: float, actual: float):
        """
        Führt einen Soll / Ist vergleich zwischen zwei werten durch und gibt einen bool wert zurück.
        
        :param target: Zielwert welcher erreicht werden soll
        :type target: float
        :param actual: Wert welcher eigentlich aktuell ausgelesen wird
        :type actual: float
        """

        if target > actual:
            self.angular_cmd += ANGULAR_Z_STEPSIZE
            if abs(self.angular_cmd) > ANGULAR_Z_MAX:
                self.angular_cmd = ANGULAR_Z_MAX
        elif target < actual:
            self.angular_cmd -= ANGULAR_Z_STEPSIZE
            print("angular_cmd",self.angular_cmd)
            if abs(self.angular_cmd) > abs(ANGULAR_Z_MAX):
                self.angular_cmd = ANGULAR_Z_MAX
        else:
           # print('[ERROR] Error im Soll / Ist vergleich! Übergebene werte waren:\n Soll: ', target, 'Ist: ', actual)
            exit()
    
    def execute(self):
        match self.state:
            case 1:
                print('[DEBUG] StateMachine Case 1. Driving')
                if self.distance < 0.4 and not self.id == -1:
                    self.state = 2
                if abs(self.angle) > self.angle_tolerance:
                    self.target_actual_comparison(0.0, self.angle)
            case 2:
                print('[DEBUG] StateMachine Case 2. Finished')
                self.drive_complete = True
            case 3: 
                print("kein Aruco marker ")