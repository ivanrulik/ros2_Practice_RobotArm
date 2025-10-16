from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder
import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
    # Use sim time toggle (defaults to true for containers/sim)
    is_sim_arg = DeclareLaunchArgument("is_sim", default_value="true")

    is_sim = LaunchConfiguration("is_sim")

    moveit_config = (
        MoveItConfigsBuilder("practice_robotarm", package_name="practice_robotarm_moveit")
        .robot_description(file_path=os.path.join(get_package_share_directory("practice_robotarm_description"), "urdf", "practice_robotarm.urdf.xacro"))
        .robot_description_semantic(file_path="config/practice_robotarm.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .to_moveit_configs()
    )

    # Robot State Publisher to publish TF from the URDF
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{**moveit_config.robot_description, "use_sim_time": is_sim}],
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": is_sim, "publish_robot_description_semantic": True},
            {"robot_description_planning": {"cartesian_limits": {
                "max_trans_vel": 1.0,
                "max_trans_acc": 2.25,
                "max_trans_dec": 2.5,
                "max_rot_vel": 1.57
            }}}
        ],
        arguments=["--ros-args", "--log-level", "info"]
    )

    # Use an existing RViz config from the description package (no moveit.rviz present here)
    rviz_config = os.path.join(
        get_package_share_directory("practice_robotarm_description"),
        "rviz",
        "display.rviz",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        # Provide the full MoveIt parameter set so RViz MoveIt plugin initializes cleanly
        parameters=[{**moveit_config.to_dict(), "use_sim_time": is_sim}],
    )

    return LaunchDescription([
        is_sim_arg,
        robot_state_publisher_node,
        move_group_node,
        rviz_node
    ])