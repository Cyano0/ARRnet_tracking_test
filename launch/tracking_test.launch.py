# launch/tracking_test.launch.py
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Adjust these if you prefer absolute paths
    json_path  = '/home/ros/aoc_strawberry_scenario_ws/src/aoc_strawberry_scenario/arrnet_demo/ARRnet_tracking_test/dataset/labeled_data_output_images_edited.json'
    image_dir  = '/home/ros/aoc_strawberry_scenario_ws/src/aoc_strawberry_scenario/arrnet_demo/ARRnet_tracking_test/dataset/output_images'

    detector = Node(
        package='tracking_test',
        executable='mock_detection_publisher',  # swap to 'detector_node' for real model
        name='mock_detection_publisher',
        output='screen',
    )

    evaluator = Node(
        package='tracking_test',
        executable='evaluator',
        name='evaluator',
        output='screen',
    )

    visualiser = Node(
        package='tracking_test',
        executable='visualiser',
        name='visualiser',
        output='screen',
    )

    ground_truth = Node(
        package='tracking_test',
        executable='ground_truth_publisher',
        name='ground_truth_publisher',
        output='screen',
        parameters=[
            {'json_path':  json_path},
            {'image_dir':  image_dir},
            {'publish_rate': 5.0},
        ],
    )

    return LaunchDescription([
        detector,                                       # t=0
        TimerAction(period=1.0, actions=[evaluator]),   # t=1s
        TimerAction(period=2.0, actions=[visualiser]),  # t=2s
        TimerAction(period=3.0, actions=[ground_truth]) # t=3s
    ])

