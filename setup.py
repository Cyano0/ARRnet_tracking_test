from setuptools import setup

package_name = 'tracking_test'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/tracking_test.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Detection and tracking test package using standard ROS2 messages',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ground_truth_publisher = tracking_test.ground_truth_publisher:main',
            'mock_detection_publisher = tracking_test.mock_detection_publisher:main',
            'evaluator = tracking_test.evaluator:main',
            'visualiser = tracking_test.visualiser:main',
            # optional real detector template:
            'detector_node = tracking_test.detector_node:main',
        ],
    },
)
