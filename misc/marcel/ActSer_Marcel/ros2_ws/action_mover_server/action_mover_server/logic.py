from types import SimpleNamespace
import math

output = SimpleNamespace(
    linear=SimpleNamespace(x=0, y=0),
    angular=SimpleNamespace(z=0),
    distance_remaining = 0,
    finish = False,
    success = False
)

# Unless otherwise noted in the format: [x, y, theta]
goal_coords = [None, None, None]
position = [None, None, None]

# Speed in format: [Linear X, Linear Y, Angular Z]
speed = [None, None, None]

# Tolerances
ANGLE_TOLERANCE = math.radians(2)
TRANSLATION_TOLERANCE = 0.1

class RobotMovement:
    """
    Class containing all functions
    """

    def Output(self):
        return output
    

    def ResetMissionFlag(self):
        output.finish = False
        output.success = False


    def CurrentPos(self, x, y, theta) -> None:
        """
        Sets the Current position. Called from odom_callback_fnc.
        """

        position[0] = x
        position[1] = y
        position[2] = theta
    

    def SpeedParameter(self, linearx, lineary, angularz) -> None:
        """
        Sets the Speed Parameter. Called from odom_callback_fnc.
        """

        speed[0] = linearx
        speed[1] = lineary
        speed[2] = angularz
        
    
    def GoalParameters(self, x, y, theta) -> None:
        """
        This function gets called from the servers 'execute_callback_fnc'. It sets the 
        """

        goal_coords[0] = x
        goal_coords[1] = y
        goal_coords[2] = theta
    
    
    def PolarAngleDiff(self, norm=True):
        """
        Calculates the angle difference between current and target.
        """

        target_angle = math.atan2((goal_coords[1] - position[1]), (goal_coords[0] - position[0]))
        angle_diff = target_angle - position.theta
        if norm:
            if angle_diff > math.pi:
                angle_diff -= 2 * math.pi
            elif angle_diff < -math.pi:
                angle_diff += 2 * math.pi
        
        return angle_diff
    

    def PolarDistDiff(self):
        """
        Calculates the difference between current pos and target pos.
        """

        distance = math.sqrt(((goal_coords[0] - position[0]) ** 2) + ((goal_coords[1] - position[1]) ** 2)) 
        print("DEBUG", goal_coords[0], goal_coords[1], goal_coords[2])
        print("DEBUG", position[0], position[1], position[2])
        return distance


    def PolarGoalDistDiff(self):
        """
        """

        target_angle = goal_coords[2]
        angle_diff = target_angle - goal_coords[2]

        if angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        elif angle_diff < -math.pi:
            angle_diff += 2 * math.pi

        return angle_diff
    
    
    def ExecMovement(self):
        """
        Calls Robot movement logic. Called from the servers 'execute_callback_fnc'.
        """

        if goal_coords[0] or goal_coords[1] or goal_coords[2] == None:
            return
        if position[0] or position[0] or position[2] == None:
            return

        angle_diff = None
        distance_diff = None
        goal_angle_diff = None

        angle_diff = RobotMovement.PolarAngleDiff(self)
        distance_diff = RobotMovement.PolarDistDiff(self)
        goal_angle_diff = RobotMovement.PolarGoalDistDiff(self)
        output.distance_remaining = distance_diff

        if abs(angle_diff) >= ANGLE_TOLERANCE and distance_diff > TRANSLATION_TOLERANCE:
            
            if angle_diff > 0:
                output.angular.z = 0.4
            else:
                output.angular.z = -0.4
            return
        
        elif abs(angle_diff) < ANGLE_TOLERANCE:
            output.angular.z = 0
        
        if distance_diff > TRANSLATION_TOLERANCE:
            output.angular.z = 0
            output.linear.x = 0.1
            
            return
        
        if abs(goal_angle_diff) >= ANGLE_TOLERANCE:
            output.linear.x = 0
            output.linear.y = 0

            if goal_angle_diff > 0:
                output.angular.z = 0.4
            else: 
                output.angular.z = -0.4
            return
        
        elif abs(goal_angle_diff) < ANGLE_TOLERANCE:
            output.angular.z = 0
            output.finish = True
            output.success = True
            return