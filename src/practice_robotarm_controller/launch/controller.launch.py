import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    robot_description = ParameterValue(
        Command(
            [
            "xacro ",
            os.path.join(
                get_package_share_directory('practice_robotarm_description'), 
                'urdf', 
                'practice_robotarm.urdf.xacro'
                ),
            ]
        ), 
        value_type=str
    )
    
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description,
                     'use_sim_time': True}],
    )

    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster", 
            "--controller-manager", 
            "/controller_manager"
        ],
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "arm_controller", 
            "--controller-manager", 
            "/controller_manager"
        ],
    )

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "gripper_controller", 
            "--controller-manager", 
            "/controller_manager"
        ],
    )


    return LaunchDescription(
        [
            robot_state_publisher_node,
            joint_state_broadcaster,
            arm_controller_spawner,
            gripper_controller_spawner
        ]
    )