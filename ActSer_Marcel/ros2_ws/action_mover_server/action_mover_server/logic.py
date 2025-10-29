from types import SimpleNamespace

output = SimpleNamespace(
    linear=SimpleNamespace(x=0, y=0),
    angular=SimpleNamespace(z=0),
    distance_remaining = 0,
    finish = False,
    success = False
)

goal_coords = [None, None, None]

class RobotMovement:
    """
    Class containing all functions
    """

    def Output(self):
        return output
    
    def GoalParameters(self, x, y, theta) -> None:
        """
        This function gets called from the servers 'execute_callback_fnc'. It sets the 
        """

        goal_coords[0] = x
        goal_coords[1] = y
        goal_coords[2] = theta
    
    
    def ExecMovement(self):
        """
        Calls Robot movement logic. Called from the servers 'execute_callback_fnc'.
        """

        x = 1