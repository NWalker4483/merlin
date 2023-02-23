import os
import yaml
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return file.read()
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():
    moveit_config_package_name = "merlin_moveit"
    description_package_name = "merlin_description"
    description_xacro_file = "urdf/merlin/merlin.urdf.xacro"
    robot_description_semantic_file = "srdf/merlin.srdf"
    robot_description_kinematics_file = "config/kinematics.yml"
    moveit_controllers_file = "config/merlin_controllers.yml"
    joint_limits_file = "config/joint_limits.yml"
    pilz_cartesian_limits_file = "config/cartesian_limits.yml"

    moveit_config = (
        MoveItConfigsBuilder(moveit_config_package_name,package_name=moveit_config_package_name)\
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
    )

    # Get parameters for the Servo node
    servo_yaml = load_yaml("path_runner", "config/servoing.yml")
    servo_params = {"moveit_servo": servo_yaml}


    # Launch as much as possible in components
    container = launch_ros.actions.ComposableNodeContainer(
        name="moveit_servo_demo_container",
        namespace="/",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[
            # # Example of launching Servo as a node component
            # # Assuming ROS2 intraprocess communications works well, this is a more efficient way.
            # launch_ros.descriptions.ComposableNode(
            #     package="moveit_servo",
            #     plugin="moveit_servo::ServoNode",
            #     name="servo_server",
            #     parameters=[
            #         servo_params,
            #         moveit_config.robot_description,
            #         moveit_config.robot_description_semantic,
            #     ],
            # ),
            launch_ros.descriptions.ComposableNode(
                package="robot_state_publisher",
                plugin="robot_state_publisher::RobotStatePublisher",
                name="robot_state_publisher",
                parameters=[moveit_config.robot_description],
            ),
            # launch_ros.descriptions.ComposableNode(
            #     package="tf2_ros",
            #     plugin="tf2_ros::StaticTransformBroadcasterNode",
            #     name="static_tf2_broadcaster",
            #     parameters=[{"child_frame_id": "/merlin_link_0", "frame_id": "/panda_link0"}],
            # ),launch_ros.descriptions.ComposableNode(
            #     package="tf2_ros",
            #     plugin="tf2_ros::StaticTransformBroadcasterNode",
            #     name="static_tf2_broadcaster",
            #     parameters=[{"child_frame_id": "/panda_link0", "frame_id": "/world"}],
            # ),
            launch_ros.descriptions.ComposableNode(
                package="moveit_servo",
                plugin="moveit_servo::JoyToServoPub",
                name="controller_to_servo_node",
            ),
            launch_ros.descriptions.ComposableNode(
                package="joy",
                plugin="joy::Joy",
                name="joy_node",
            ),
        ],
        output="screen",
    )
    # # Launch a standalone Servo node.
    # # As opposed to a node component, this may be necessary (for example) if Servo is running on a different PC
    servo_node = launch_ros.actions.Node(
        package="moveit_servo",
        executable="servo_node_main",
        parameters=[
            servo_params,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
        output="screen",
    )

    return launch.LaunchDescription(
        [
            # rviz_node,
            # ros2_control_node,
            # joint_state_broadcaster_spawner,
            # panda_arm_controller_spawner,
            servo_node,
            container,
        ]
    )
