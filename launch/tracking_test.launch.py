# launch/tracking_test.launch.py
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node

def generate_launch_description():
    detector = Node(
        package='tracking_test',
        executable='mock_detection_publisher',
        name='mock_detection_publisher',
        output='screen'
    )

    evaluator = Node(
        package='tracking_test',
        executable='evaluator',
        name='evaluator',
        output='screen'
    )

    ground_truth = Node(
        package='tracking_test',
        executable='ground_truth_publisher',
        name='ground_truth_publisher',
        output='screen',
        parameters=[
            {'json_path':  'src/aoc_strawberry_scenario/tracking_test/dataset/labeled_data_output_images_edited.json'},
            {'image_dir':  'src/aoc_strawberry_scenario/tracking_test/dataset/output_images'},
            {'publish_rate': 5.0}
        ]
    )

    # Launch order: detector → evaluator (after 1 s) → ground-truth (after 3 s total)
    return LaunchDescription([
        detector,
        TimerAction(period=1.0, actions=[evaluator]),
        TimerAction(period=3.0, actions=[ground_truth]),
    ])

