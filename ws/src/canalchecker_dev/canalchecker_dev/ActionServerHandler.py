import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from canalchecker_interface.action import Align, Follow, Drive
from canalchecker_interface.msg import ArucoDetection
import threading
import time



class ActionServerHandler(Node):
    def __init__(self):
        super().__init__('actionserverhandler')
        
        # ArUco Detection mit Lock
        self._aruco_id = -1
        self._aruco_lock = threading.Lock()
        self._target_aruco_id = 69  
        self._follow_triggered = False
        
        # Subscriber für ArUco Detection
        self.sub_aruco = self.create_subscription(
            ArucoDetection,
            '/aruco_detections',
            self.aruco_callback,
            10
        )
      
        # Action Clients
        self.actionserver_align = ActionClient(self, Align, 'align')
        self.actionserver_drive = ActionClient(self, Drive, 'drive')
        self.actionserver_follow = ActionClient(self, Follow, 'follow')
        
        # State Variablen
        self.current_action = None
        self._goal_handle = None
        self._get_result_promise = None
        self._send_goal_promise = None
        
        # self.get_logger().info('Handler gestartet - starte Align')

        # Warte auf Action Server
        # self.get_logger().info('Warte auf Align Action Server...')
        self.actionserver_align.wait_for_server()
        # self.get_logger().info('Align Action Server erreichbar')
        
        # self.get_logger().info('Warte auf Drive Action Server...')
        self.actionserver_drive.wait_for_server()
        # self.get_logger().info('Drive Action Server erreichbar')
        
        # self.get_logger().info('Warte auf Follow Action Server...')
        self.actionserver_follow.wait_for_server()
        # self.get_logger().info('Alle Action Server erreichbar - starte Handler')
        
        self.send_align_goal()


    def aruco_callback(self, msg: ArucoDetection):
        """Callback für ArUco Detection - prüft auf Trigger ID 69"""
        with self._aruco_lock:
            self._aruco_id = msg.aruco_id
        
        if msg.aruco_id == self._target_aruco_id and not self._follow_triggered:
            # self.get_logger().warn(
            #     f"ARUCO ID {self._target_aruco_id} erkannt! "
            #     f"current_action={self.current_action}"
            # )
            self._follow_triggered = True
            self._trigger_follow_mode()


    def _trigger_follow_mode(self):
        """Bricht aktuelle Action ab und startet Follow-Modus"""
        if self.current_action is not None and self.current_action != 'follow':
            # self.get_logger().warn(f"Breche {self.current_action} ab für Follow-Modus")
            
            # Setze current_action zurück um Race Condition zu vermeiden
            old_action = self.current_action
            self.current_action = None
            
            cancel_future = self._cancel_current_action()
            if cancel_future is not None:
                cancel_future.add_done_callback(self._on_cancel_done)
                return
        
        # Wenn keine Action aktiv oder bereits Follow
        self._start_follow()


    def _cancel_current_action(self):
        """Versucht die aktuelle Action abzubrechen"""
        if self._goal_handle is not None:
            try:
                cancel_future = self._goal_handle.cancel_goal_async()
                # self.get_logger().info("Cancel Request gesendet")
                return cancel_future
            except Exception as e:
                # self.get_logger().error(f"Fehler beim Abbrechen: {e}")
                return None
        else:
            # self.get_logger().warn("Kein Goal Handle vorhanden")
            return None


    def _on_cancel_done(self, future):
        """Callback nach Cancel - wartet bis vollständig abgeschlossen"""
        try:
            cancel_response = future.result()
            if len(cancel_response.goals_canceling) > 0:
                # self.get_logger().info("Cancel erfolgreich akzeptiert")
                pass
            else:
                # self.get_logger().warn("Cancel nicht akzeptiert")
                pass
        except Exception as e:
            # self.get_logger().error(f"Cancel Callback Fehler: {e}")
            pass
        
        # Warte bis Result-Callback abgeschlossen ist
        # self.get_logger().info("Warte auf Abschluss des abgebrochenen Goals...")
        time.sleep(1.0)  # 1 Sekunde Pause für sauberen Abschluss
        
        # self.get_logger().info("Starte Follow nach vollständigem Cancel")
        self._start_follow()


    def _start_follow(self):
        """Startet Follow-Modus mit Verzögerung und Prüfung"""
        # self.get_logger().info("Starte Follow-Modus JETZT")
        
        # Prüfe ob Server verfügbar
        if not self.actionserver_follow.wait_for_server(timeout_sec=3.0):
            # self.get_logger().error("Follow Action Server NICHT erreichbar!")
            self._follow_triggered = False
            return
        
        # self.get_logger().info("Follow Action Server ist bereit")
        goal_msg = Follow.Goal()
        
        # self.get_logger().info("Sende Follow Goal")
        self.send_goal('follow', goal_msg)


    def get_aruco_id(self):
        """Thread-safe Zugriff auf Aruco ID"""
        with self._aruco_lock:
            return self._aruco_id


    def send_align_goal(self):
        """Startet Align Action"""
        # self.get_logger().info("Sende Align Goal")
        goal_msg = Align.Goal()
        self.send_goal('align', goal_msg)


    def send_drive_goal(self):
        """Startet Drive Action"""
        # self.get_logger().info("Sende Drive Goal")
        goal_msg = Drive.Goal()
        self.send_goal('drive', goal_msg)


    def send_follow_goal(self):
        """Startet Follow Action"""
        # self.get_logger().info("Sende Follow Goal")
        goal_msg = Follow.Goal()
        self.send_goal('follow', goal_msg)


    def send_goal(self, action_type, goal_msg):
        """Sendet Goal an Action Server"""
        clients = {
            'align': self.actionserver_align,
            'drive': self.actionserver_drive,
            'follow': self.actionserver_follow
        }
        
        client = clients.get(action_type)
        if client is None:
            # self.get_logger().error(f'Unbekannter Action Type: {action_type}')
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
            # self.get_logger().warn(f'{self.current_action} Goal ABGELEHNT')
            self.current_action = None
            return
        
        # self.get_logger().info(f'{self.current_action} Goal AKZEPTIERT')
        self._goal_handle = goal_handler
        self._get_result_promise = goal_handler.get_result_async()
        self._get_result_promise.add_done_callback(self.get_result_callback)


    def get_result_callback(self, promise):
        """Callback wenn Action beendet ist"""
        result_response = promise.result()
        result = result_response.result
        status = result_response.status
        
        action_name = self.current_action if self.current_action else "UNKNOWN"
        
        # self.get_logger().info(f'Result für {action_name}: status={status}')
        
        # Status 5 = CANCELED
        if status == 5:
            # self.get_logger().warn(f'{action_name} wurde ABGEBROCHEN')
            # Wenn durch Follow-Trigger abgebrochen, nicht neu starten
            if self._follow_triggered:
                # self.get_logger().info('Abbruch durch Follow-Trigger - warte auf Follow')
                self.current_action = None
                return
            self.current_action = None
            return
        
        # Status 4 = SUCCEEDED
        if status != 4:
            # self.get_logger().error(f'{action_name} FEHLGESCHLAGEN (status={status})')
            self.current_action = None
            return
        
        match action_name:
            case 'align':
                if result.reached:
                    # self.get_logger().info('Align ERFOLGREICH → starte Drive')
                    self.current_action = None
                    self.send_drive_goal()
                else:
                    # self.get_logger().warn('Align beendet aber reached=False')
                    self.current_action = None
            
            case 'drive':
                if result.reached:
                    # self.get_logger().info('Drive ERFOLGREICH → starte Align')
                    self.current_action = None
                    self.send_align_goal()
                else:
                    # self.get_logger().warn('Drive beendet aber reached=False')
                    self.current_action = None
            
            case 'follow':
                # self.get_logger().info(f'Follow Result: reached={result.reached}')
                if result.reached:
                    # self.get_logger().info('Follow ERFOLGREICH → zurück zu Align')
                    pass
                else:
                    # self.get_logger().warn('Follow nicht reached → zurück zu Align')
                    pass
                
                self.current_action = None
                self._follow_triggered = False
                self.send_align_goal()
            
            case _:
                # self.get_logger().error(f'Unbekannte Action: {action_name}')
                self.current_action = None


    def feedback_callback(self, feedback_msg):
        """Callback für Feedback während Ausführung"""
        feedback = feedback_msg.feedback
        
        if self.current_action == 'align':
            # self.get_logger().info(f'Align Feedback: {feedback}')
            pass
        elif self.current_action == 'drive':
            # self.get_logger().info(f'Drive Feedback: {feedback}')
            pass
        elif self.current_action == 'follow':
            # self.get_logger().info(f'Follow Feedback: {feedback}')
            pass



def main(args=None):
    rclpy.init(args=args)
    
    from rclpy.executors import MultiThreadedExecutor
    
    handler = ActionServerHandler()
    # handler.get_logger().info('Handler läuft - Ctrl+C zum Beenden')
    
    # MultiThreadedExecutor für parallele Callback-Verarbeitung
    executor = MultiThreadedExecutor()
    executor.add_node(handler)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        # handler.get_logger().info('Handler gestoppt')
        pass
    finally:
        executor.shutdown()
        rclpy.shutdown()



if __name__ == '__main__':
    main()
