import time
from logic import DriveStateMachine

distance = 1 # Distanz zum ArUco Marker

class test():
    def __init__(self):
        self.cmd_linear_x = 0
        self.cmd_angular_z = 0
        self.aruco_id = 0
        self.aruco_angle = 0
        self.aruco_dist = distance

############# EXTERNALLY SUPPLIED DATA BEGIN #############
    def ExternalDistance(self):
        global distance
        print("Distance: ", distance)
        distance = distance - self.cmd_linear_x
        self.aruco_dist = distance
        if distance < 0.45:
            self.aruco_id = -1
        time.sleep(1)
############## EXTERNALLY SUPPLIED DATA END ##############

    def execute_callback_fnc(self): 
        #self.get_logger().info('Goal Received! Driving.')
        print("Goal Received. Driving")
        #cmd = Twist()
        self.cmd_linear_x = 0
        self.cmd_angular_z = 0
        #self.goal_handle = goal_handle
        self.goal_finished = False
        self.goal_result = None
        
        state = DriveStateMachine()
        while state.drive_complete == False:
            state.id = self.aruco_id
            state.angle = self.aruco_angle
            state.distance = self.aruco_dist
            if self.aruco_id == -1:
                # Wenn kein Aruco Marker erkannt wird, fahre nicht
                print("No ArUco found. Stopping drive.")
                self.stop_robot()
            else:
                print("ArUco found.")
                self.cmd_linear_x = state.max_linear_speed

            #if goal_handle.is_cancel_requested:
            #    goal_handle.canceled()
            #    self.stop_robot()
            #    self.get_logger().info('Goal cancelled. Robot Stopped.')
            #    return Drive.Result(reached=False)

            state.execute()
            if self.aruco_id == -1:
                self.cmd_angular_z = 0.0
                self.stop_robot()
            else:
                self.cmd_angular_z = float(state.angular_cmd)
                
            print("Publishing Linear: ", self.cmd_linear_x)
            print("Publishing Angular: ", self.cmd_angular_z)

            #feedback = Drive.Feedback()
            #feedback.dist_to_goal = state.distance
            #goal_handle.publish_feedback(feedback)
            self.ExternalDistance()
        
        self.stop_robot()

        if state.drive_complete:
            #goal_handle.succeed()
            #result = Drive.Result()
            #result.reached = True
            #self.get_logger().info('Drive Complete')
            #return result
            print("Drive complete!")
            exit()
        else:
            #result = Drive.Result()
            #result.reached = False
            #return result
            print("Drive failed")
            exit()
      
    def stop_robot(self):
        #cmd = Twist()
        self.cmd_linear_x = 0
        self.cmd_angular_z = 0
        print("Publishing Linear: ", self.cmd_linear_x)
        print("Publishing Angular: ", self.cmd_angular_z)


test().execute_callback_fnc()