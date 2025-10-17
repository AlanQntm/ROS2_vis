from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='armor_vision',
            executable='armor_vision_node',
            name='armor_vision',
            output='screen',
            parameters=[{
                'device_id': 0,
                'video_path': '',
                'width': 1280,
                'height': 720,
                'fps': 60,
                'camera_info_path': 'config/camera_info.yaml',
                'bin.method': 'adaptive',
                'bin.block_size': 21,
                'bin.C': 5,
                'morph.ksize': 3,
                'ratio.min': 2.0,
                'ratio.max': 3.2,
                'area.min_px': 800,
                'rectangularity.min': 0.75,
                'candidate.max': 6,
                'track.max_age': 8,
                'track.iou_thresh': 0.2,
                'smooth.alpha': 0.25,
                'show_fps': True,
            }]
        )
    ])

