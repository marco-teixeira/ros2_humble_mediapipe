from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess


#param
input_image_topic = "image_raw"

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2_mediapipe',
            namespace='',
            executable='mediapipe_hands',
            name='mediapipe_hands',
            remappings=[
                    ('ros2_mediapipe/input/image_raw', input_image_topic),
        ]
        )
    ])
