import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from canalchecker_interface.action import Align, Follow, Drive
from canalchecker_interface.msg import ArucoDetection
from std_msgs.msg import Float32
import threading

class ActionServerHandler(Node):
    def __init__(self, target_distance=50.0):
        super().__init__('actionserverhandler')
        
        
        self.target_distance = target_distance
        self._target_distance_lock = threading.Lock()
        self.get_logger().info(f'Ziel-Distanz zum Roboter: {self.target_distance} cm')
        
        
        self._max_speed = 0.1  
        self._max_speed_lock = threading.Lock()
        

        self._aruco_id = -1
        self._aruco_lock = threading.Lock()
        self._target_aruco_id = 69  
        
        
        self._current_state = 0  
        self._follow_triggered = False
        
        
        self.sub_target_distance = self.create_subscription(
            Float32,
            '/target_distance',
            self.target_distance_callback,
            10
        )
        
        
        self.sub_max_speed = self.create_subscription(
            Float32,
            '/max_speed',
            self.max_speed_callback,
            10
        )
        
        
        self.sub_aruco = self.create_subscription(
            ArucoDetection,
            '/aruco_detections',
            self.aruco_callback,
            10
        )
        
        
        self.actionserver_align = ActionClient(self, Align, 'align')
        self.actionserver_drive = ActionClient(self, Drive, 'drive')
        self.actionserver_follow = ActionClient(self, Follow, 'follow')
        
        self.timer = self.create_timer(0.1, self.state_machine_loop)
        self.current_action = None
        self._goal_handle = None
        
        self.get_logger().info('Handler started ')

    def aruco_callback(self, msg: ArucoDetection):
        """Callback für ArUco Detection - prüft auf Trigger ID 69"""
        with self._aruco_lock:
            self._aruco_id = msg.aruco_id
        
        
        if msg.aruco_id == self._target_aruco_id and not self._follow_triggered:
            self.get_logger().warn(
                f"ARUCO ID {self._target_aruco_id} Wechsle zu Follow-Modus!"
            )
            self._follow_triggered = True
            self._trigger_follow_mode()
        else:
            self.get_logger().info(f"ArUco empfangen: ID={msg.aruco_id}")

    def _trigger_follow_mode(self):
        """Bricht aktuelle Action ab und startet Follow-Modus"""
        
        if self.current_action is not None:
            self.get_logger().warn(
                f"Breche {self.current_action} Action ab für Follow-Modus"
            )
            self._cancel_current_action()
    
        self._current_state = 30
        
        
        self.get_logger().info("Starte Follow-Modus SOFORT")
        goal_msg = Follow.Goal()
        goal_msg.target_distance = self.get_target_distance()
        goal_msg.max_speed = self.get_max_speed()
        self.send_goal('follow', goal_msg)

    def _cancel_current_action(self):
        """Versucht die aktuelle Action abzubrechen"""
        if self._goal_handle is not None:
            try:
                self._goal_handle.cancel_goal_async()
                self.get_logger().info("Cancel Request gesendet")
            except Exception as e:
                self.get_logger().error(f"Fehler beim Abbrechen: {e}")
        
        self.current_action = None

    def max_speed_callback(self, msg: Float32):
        """Callback für Geschwindigkeitsvorgabe - begrenzt auf 0.0 bis 0.2 m/s"""
        speed = msg.data
        
        
        if speed < 0.0:
            speed = 0.0
            self.get_logger().warn(f"Geschwindigkeit < 0! Setze auf 0.0 m/s")
        elif speed > 0.2:
            speed = 0.2
            self.get_logger().warn(f"Geschwindigkeit > 0.2! Begrenze auf 0.2 m/s")
        
        with self._max_speed_lock:
            self._max_speed = speed
        
        self.get_logger().info(f"Neue Max-Geschwindigkeit: {speed:.3f} m/s")

    def target_distance_callback(self, msg: Float32):
        """Callback für Ziel-Distanz in cm"""
        distance = msg.data
   
        if distance < 20.0:
            distance = 10.0
            self.get_logger().warn(f"Distanz < 10cm! Setze auf 10 cm")
        elif distance > 100.0:
            distance = 100.0
            self.get_logger().warn(f"Distanz > 500cm! Begrenze auf 500 cm")
        
        with self._target_distance_lock:
            self.target_distance = distance
        
        self.get_logger().info(f"Neue Ziel-Distanz: {distance:.1f} cm")

    def get_max_speed(self):
        """Gibt aktuelle maximale Geschwindigkeit zurück"""
        with self._max_speed_lock:
            return self._max_speed

    def get_target_distance(self):
        """Gibt aktuelle Ziel-Distanz zurück"""
        with self._target_distance_lock:
            return self.target_distance

    def get_aruco_id(self):
        """Gibt aktuelle Aruco ID zurück"""
        with self._aruco_lock:
            return self._aruco_id

    def state_machine_loop(self):
        """Timer Callback - Ablaufsteuerung"""
        
        if self._follow_triggered:
            return
        
        # Nur neue Actions starten wenn keine aktiv ist
        if self.current_action is not None:
            return
        
      
        if self._current_state == 0:  
            self.get_logger().info("State: IDLE -> Starte Align")
            self._current_state = 10
            self.send_align_goal()

    def send_align_goal(self):
        """Startet Align Action"""
        self.get_logger().info("Sende Align Goal")
        goal_msg = Align.Goal()
        goal_msg.max_speed = self.get_max_speed()
        self.send_goal('align', goal_msg)

    def send_drive_goal(self):
        """Startet Drive Action"""
        self.get_logger().info("Sende Drive Goal")
        goal_msg = Drive.Goal()
        goal_msg.max_speed = self.get_max_speed()
        self.send_goal('drive', goal_msg)

    def send_follow_goal(self):
        """Startet Follow Action"""
        self.get_logger().info("Sende Follow Goal")
        goal_msg = Follow.Goal()
        goal_msg.target_distance = self.get_target_distance()
        goal_msg.max_speed = self.get_max_speed()
        self.send_goal('follow', goal_msg)

    def send_goal(self, action_type, goal_msg):
        """Sendet Goal an Action Server"""
        clients = {
            'align': self.actionserver_align,
            'drive': self.actionserver_drive,
            'follow': self.actionserver_follow
        }
        
        client = clients.get(action_type)
        if not client:
            self.get_logger().error(f'Unknown action type: {action_type}')
            return
        
        self.current_action = action_type
        client.wait_for_server()
        
        self._send_goal_promise = client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_promise.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, promise):
        """Callback wenn Goal akzeptiert/abgelehnt"""
        goal_handler = promise.result()
        if not goal_handler.accepted:
            self.get_logger().info('Goal rejected :(')
            self.current_action = None
            return
        
        self.get_logger().info('Goal accepted :)')
        self._goal_handle = goal_handler  
        self._get_result_promise = goal_handler.get_result_async()
        self._get_result_promise.add_done_callback(self.get_result_callback)

    def get_result_callback(self, promise):
        """Callback wenn Server fertig ist - prüft Status und Result"""
        result_response = promise.result()
        result = result_response.result
        status = result_response.status
        
        # Status Codes: 4 = SUCCEEDED, 5 = CANCELED, 6 = ABORTED
        if status == 5:  # CANCELED
            self.get_logger().warn(f'{self.current_action} wurde abgebrochen')
            self.current_action = None
            return
        
        if status != 4:  # 4 = SUCCEEDED
            self.get_logger().error(
                f'{self.current_action} Server FEHLGESCHLAGEN (status={status})'
            )
            self.current_action = None
            return
        

        match self.current_action:
            case 'align':
                if result.reached:
                    self.get_logger().info('Align Server ERFOLGREICH')
                    self._current_state = 20  
                    self.send_drive_goal()
                else:
                    self.get_logger().warn('Align beendet aber reached=False')
            
            case 'drive':
                if result.reached:
                    self.get_logger().info('Drive Server ERFOLGREICH')
                    self._current_state=0
                else:
                    self.get_logger().warn('Drive beendet aber reached=False')
            
            case 'follow':
                if result.reached:
                    self.get_logger().info('Follow Server ERFOLGREICH - FERTIG!')
                    self._current_state=10
                else:
                    self.get_logger().warn('Follow beendet aber reached=False')
        
        self.current_action = None

    def feedback_callback(self, feedback_msg):
        """Callback für Feedback während der Ausführung"""
        feedback = feedback_msg.feedback
        
        match self.current_action:
            case 'align':
                if feedback.angle_to_goal:
                    self.get_logger().info(
                        f'Align Feedback: angle={feedback.angle_to_goal:.2f}°'
                    )
            
            case 'drive':
                self.get_logger().info(f'Drive Feedback: {feedback}')
            
            case 'follow':
                self.get_logger().info(f'Follow Feedback: {feedback}')

def main(args=None):
    rclpy.init(args=args)
    try:
        distance_input = input("Geben Sie den Soll-Abstand zum Roboter ein (in cm): ")
        try:
            target_distance = float(distance_input)
            print(f"Eingegebener Abstand: {target_distance} cm")
        except ValueError:
            print("Ungültige Eingabe! Verwende Standard: 50 cm")
            target_distance = 50.0
        print("=" * 50)
        
        handler = ActionServerHandler(target_distance=target_distance)
        handler.get_logger().info('Handler started - press Ctrl+C to exit')
        rclpy.spin(handler)
    
    except KeyboardInterrupt:
        handler.get_logger().info('Handler stopped')
    
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()