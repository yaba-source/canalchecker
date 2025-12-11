KP = 1.1
ANGULAR_Z_MAX = 0.2
MAX_LINEAR_SPEED = 0.08
ANGLE_TOLERABLE_ERR = 3

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
        self.angular_cmd = 0.0          # angular speed to be commanded

    def target_actual_comparison(self, target: float, actual: float):
        """
        Führt einen Soll / Ist vergleich zwischen zwei werten durch und implementiert eine entsprechende P-Regelung. Regelt winkel sowie die Geschwindigkeit basierend auf diesem.
        
        :param target: Zielwert welcher erreicht werden soll
        :type target: float
        :param actual: Wert welcher eigentlich aktuell ausgelesen wird
        :type actual: float
        """

        error = target - actual

        self.angular_cmd = KP * error

        if self.angular_cmd > ANGULAR_Z_MAX:
            self.angular_cmd = ANGULAR_Z_MAX
        elif self.angular_cmd < - ANGULAR_Z_MAX:
            self.angular_cmd = -ANGULAR_Z_MAX
        
        if abs(error) < 0.1:
            self.max_linear_speed = MAX_LINEAR_SPEED
        elif abs(error) < 0.3:
            self.max_linear_speed = 0.5 * MAX_LINEAR_SPEED
        else:
            self.max_linear_speed = 0.2 * MAX_LINEAR_SPEED
        
        # DEBUG
        print("Error: ", float(error), "\nangular_cmd: ", float(self.angular_cmd), "\nLinear speed: ", float(self.max_linear_speed))
    
    def execute(self):
        match self.state:
            case 1:
                print('[DEBUG] StateMachine Case 1. Driving')
                if self.distance < 0.45 and not self.id == -1:
                    self.state = 2
                if abs(self.angle) > self.angle_tolerance:
                    self.target_actual_comparison(0.0, self.angle)
            case 2:
                print('[DEBUG] StateMachine Case 2. Finished')
                self.drive_complete = True