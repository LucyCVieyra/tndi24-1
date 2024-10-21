from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=["MicroXRCEAgent", "serial", "--dev", "/dev/serial0", "-b", "921600"],
            output="log"
        ),
        Node(
            package="tndi24",
            executable="px4_driver",
            output="screen"
        ),
        Node(
            package="v4l2_camera",
            executable="v4l2_camera_node",
            output="screen",
            remappings=[
                ("__ns","/camera")
            ],
            parameters=[{
                "video_device":"/dev/video0"
            }]
        )
    ])