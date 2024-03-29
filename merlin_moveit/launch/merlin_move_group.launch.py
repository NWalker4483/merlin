import os

import yaml
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

from moveit_configs_utils import MoveItConfigsBuilder

def load_yaml(package_name: str, file_path: str):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as f:
            return yaml.safe_load(f)
    except EnvironmentError:
        return None


def load_file(package_name: str, file_path: str) -> str:
    package_path = get_package_share_directory(package_name)
    absolut_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolut_file_path, "r") as f:
            return f.read()
    except EnvironmentError:
        return None


def launch_setup(context, *args, **kwargs):

    # Configure robot_description
    robot_description = {"robot_description": LaunchConfiguration("robot_description")}

    # Robot semantics SRDF
    robot_description_semantic = {
        "robot_description_semantic": load_file(
            "merlin_moveit", "srdf/merlin.srdf"
        )
    }

    # Kinematics
    kinematics_yaml = load_yaml("merlin_moveit", "config/kinematics.yml")

    # Joint limits
    robot_description_planning = {
        "robot_description_planning": PathJoinSubstitution(
            [FindPackageShare("merlin_moveit"), "config/joint_limits.yml"]
        )
    }
    
    # Trajectory execution
    trajectory_execution = {
        "allow_trajectory_execution": True,
        "moveit_manage_controllers": True,
    }

    # Controllers
    controllers_yaml = load_yaml(
        LaunchConfiguration("moveit_controller_configurations_package").perform(
            context
        ),
        LaunchConfiguration("moveit_controller_configurations").perform(context),
    )

    moveit_controllers = {
        "moveit_simple_controller_manager": controllers_yaml,
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }

    # Planning scene
    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    # Time configuration
    use_sim_time = {"use_sim_time": LaunchConfiguration("sim")}

    moveit_config_package_name = "merlin_moveit"
    description_package_name = "merlin_description"
    description_xacro_file = "urdf/merlin/merlin.urdf.xacro"
    robot_description_semantic_file = "srdf/merlin.srdf"
    robot_description_kinematics_file = "config/kinematics.yml"
    moveit_controllers_file = "config/merlin_controllers.yml"
    joint_limits_file = "config/joint_limits.yml"
    pilz_cartesian_limits_file = "config/cartesian_limits.yml"

   
    config = MoveItConfigsBuilder(moveit_config_package_name,package_name=moveit_config_package_name)\
    .robot_description(
            file_path=get_package_share_directory(description_package_name)
            + "/"
            + description_xacro_file )\
    .robot_description_semantic(file_path=robot_description_semantic_file)\
    .robot_description_kinematics(file_path=robot_description_kinematics_file)\
    .trajectory_execution(file_path=moveit_controllers_file)\
    .joint_limits(file_path=joint_limits_file)\
    .pilz_cartesian_limits(file_path=pilz_cartesian_limits_file)\
    .to_moveit_configs()
    planning_plugin = {
        "planning_plugin": "pilz_industrial_motion_planner/CommandPlanner"
    }
    
    # Prepare move group node
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        arguments=["--ros-args"],
        parameters=[
            robot_description_planning,
            config.to_dict(),
            planning_plugin,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
            use_sim_time,
        ],
    )

    # RViz
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("merlin_description"), "config/config.rviz"]
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            use_sim_time,
        ],
        arguments=["-d", rviz_config],
    )

    return [move_group_node, rviz]


def generate_launch_description():

    # Launch arguments
    launch_args = []

    launch_args.append(
        DeclareLaunchArgument(
            name="robot_description", description="Robot description XML file."
        )
    )

    launch_args.append(
        DeclareLaunchArgument(
            name="moveit_controller_configurations_package",
            default_value="merlin_moveit",
            description="Package that contains MoveIt! controller configurations.",
        )
    )

    launch_args.append(
        DeclareLaunchArgument(
            name="moveit_controller_configurations",
            default_value="config/merlin_controllers.yml",
            description="Relative path to MoveIt! controller configurations YAML file. Note that the joints in the controllers must be named according to the robot_name.",
        )
    )

    launch_args.append(
        DeclareLaunchArgument(
            name="sim",
            default_value="true",
            description="Launch robot in simulation or on real setup.",
        )
    )

        
    launch_args.append(
        DeclareLaunchArgument(
            name="table",
            default_value="true",
            description="Launch robot in simulation or on real setup.",
        )
    )
    
    launch_args.append(
        DeclareLaunchArgument(
            name="tool",
            default_value="basic_drill",
            description="Launch robot in simulation or on real setup.",
        )
    )

    return LaunchDescription(launch_args + [OpaqueFunction(function=launch_setup)])
