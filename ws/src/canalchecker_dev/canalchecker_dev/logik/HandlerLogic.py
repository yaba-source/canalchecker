"""
Describes the Logic in the which server is called
First call is align then Drive an if follow id is detected Follow 
"""


class stateMachine:
    def __init__(self):
        self.state =0
        self.error=None
        self.align_done = False
        self.drive_done = False
        self.follow_done = False
        

    def execute(self):
        """State Machine for the Workflow"""
        match self.state:
            case 10:
                self.align_state()
            case 20:
                self.drive_state()
            case 30:
                self.follow_state()
            case _:
                if self.logger:
                    self.logger.error(f"Unknown state: {self.state}")
                else:
                    print(f"Unknown state: {self.state}")

        
    def align_state(self):
        """State 10: Align Server"""
        if self.logger:
            self.logger.info("State 10: Calling Align Server")
        if self.align_done:
            if self.logger:
                self.logger.info("Align complet")
            self.state = 20
            self.align_done = False
    
    def drive_state(self):
        if self.drive_done:
            if self.aruco_detected and self.aruco_id == 69:
                if self.logger:
                    self.logger.info("Aruco ID 69 detected, moving to Follow")
                self.state = 30
            else:
                if self.logger:
                    self.logger.info("Drive completed, returning to Align")
                self.state = 10
            
            self.drive_done = False
            self.aruco_detected = False
    def follow_state(self):
        """State 30: Follow Server"""
        if self.logger:
            self.logger.info("State 30: Calling Follow Server")
        
        if self.follow_done:
            if self.logger:
                self.logger.info("Follow completed, returning to Align")
            self.state = 10
            self.follow_done = False

    def set_align_done(self):
        """Handler will set if align done"""
        self.align_done = True

    def set_drive_done(self):
        """Handler will set if drive done"""
        self.drive_done = True

    def set_follow_done(self):
        """Handler will set if follow done"""
        self.follow_done = True

    def set_aruco_detected(self, aruco_id):
        """Wird vom Handler aufgerufen wenn Aruco erkannt wurde"""
        self.aruco_detected = True
        self.aruco_id = aruco_id
    
