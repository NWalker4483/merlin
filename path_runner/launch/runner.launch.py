import os
from launch import LaunchDescription
from launch.launch_description_sources import load_python_launch_file_as_module
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
# from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # moveit_config = (
    #     MoveItConfigsBuilder(robot_name="lite6", package_name="moveit_resources_lite6_moveit_config")
    #     .trajectory_execution(file_path="config/moveit_controllers.yaml")
    #     .robot_description_semantic("config/lite6.srdf")
    #     .robot_description(file_path=get_package_share_directory("moveit_resources_lite6_description") 
    #         + "/urdf/lite6.urdf")
    #     .moveit_cpp(
    #         file_path=get_package_share_directory("lite6_moveit_demos")
    #         + "/config/jupyter_notebook_prototyping.yaml"
    #     )
    #     .to_moveit_configs()
    # )

    #rviz_config_file = (
    #    get_package_share_directory("moveit2_tutorials") + "/config/jupyter_notebook_prototyping.rviz"
    #)
    # rviz_node = Node(
    #     package="rviz2",
    #     executable="rviz2",
    #     name="rviz2",
    #     output="log",
    # #    arguments=["-d", rviz_config_file],
    #     parameters=[
    #         moveit_config.robot_description,
    #         moveit_config.robot_description_semantic,
    #     ],
    # )

    # static_tf = Node(
    #     package="tf2_ros",
    #     executable="static_transform_publisher",
    #     name="static_transform_publisher",
    #     output="log",
    #     arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "link_base"],
    # )

    # robot_state_publisher = Node(
    #     package="robot_state_publisher",
    #     executable="robot_state_publisher",
    #     name="robot_state_publisher",
    #     output="both",
    #     parameters=[moveit_config.robot_description],
    # )

    # joint_state_publisher = Node(
    #     package='joint_state_publisher',
    #     executable='joint_state_publisher',
    #     name='joint_state_publisher',
    #     output='screen',
    #     parameters=[{'source_list': ['ufactory/joint_states']}],
    # )

    # ros2_controllers_path = os.path.join(
    #     get_package_share_directory("moveit_resources_lite6_moveit_config"),
    #     "config",
    #     "ros2_controllers.yaml",
    # )

    # mod = load_python_launch_file_as_module(os.path.join(get_package_share_directory('xarm_api'), 'launch', 'lib', 'robot_api_lib.py'))
    # generate_robot_api_params = getattr(mod, 'generate_robot_api_params')
    # robot_params = generate_robot_api_params(
    #     os.path.join(get_package_share_directory('xarm_api'), 'config', 'xarm_params.yaml'),
    #     os.path.join(get_package_share_directory('xarm_api'), 'config', 'xarm_user_params.yaml'),
    #     ros_namespace='', node_name='ufactory_driver'
    # )
    # ros2_control_node = Node(
    #     package="controller_manager",
    #     executable="ros2_control_node",
    #     parameters=[
    #         moveit_config.robot_description, 
    #         ros2_controllers_path,
    #         robot_params],
    #     output="both",
    # )

    # load_controllers = []
    # for controller in [
    #     "lite6_traj_controller",
    # ]:
    #     load_controllers += [
    #         ExecuteProcess(
    #             cmd=["ros2 run controller_manager spawner {}".format(controller)],
    #             shell=True,
    #             output="screen",
    #         )
    #     ]

    # We can start a notebook from a launch file
    start_notebook = ExecuteProcess(cmd = ["python3 -m notebook --allow-root"], shell = True, output = "screen")
    return LaunchDescription(
        [
            start_notebook,
            # static_tf,
            # robot_state_publisher,
            # joint_state_publisher,
            # rviz_node,
            # ros2_control_node,
        ]
        # + load_controllers
        )