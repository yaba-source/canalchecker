from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='canalchecker_dev',
            executable='CameraNode'
        ),
        Node(
            package='canalchecker_dev',
            executable='DriveActionServer'
        ),
        Node(
            package='canalchecker_dev',
            executable='AlignActionServer'
        ),
        #Node(
        #    package='canalchecker_dev',
        #    executable='ActionServerHandler'
        #)
        Node(
            package='canalchecker_dev',
            executable='FollowActionServer'
        )
    ])