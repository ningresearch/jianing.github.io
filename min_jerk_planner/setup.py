from setuptools import find_packages, setup

package_name = 'min_jerk_planner'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/min_jerk_demo.launch.py']),
        ('share/' + package_name + '/config', ['config/joint_limits.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='User',
    maintainer_email='user@example.com',
    description='High-precision min-jerk trajectory planner for ROS 2 Humble',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'min_jerk_planner_node = min_jerk_planner.min_jerk_planner_node:main',
            'trajectory_executor = min_jerk_planner.trajectory_executor:main',
            'demo_client = min_jerk_planner.demo_client:main',
        ],
    },
)