import os
import yaml
import xacro
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory

def get_package_file(package, file_path):
    """Get the location of a file installed in an ament package"""
    package_path = get_package_share_directory(package)
    absolute_file_path = os.path.join(package_path, file_path)
    return absolute_file_path

def load_file(file_path):
    """Load the contents of a file into a string"""
    try:
        with open(file_path, 'r') as file:
            return file.read()
    except EnvironmentError: # parent of IOError, OSError *and* WindowsError where available
        return None

def load_yaml(file_path):
    """Load a yaml file into a dictionary"""
    try:
        with open(file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError: # parent of IOError, OSError *and* WindowsError where available
        return None

def run_xacro(xacro_file):
    """Run xacro and output a file in the same directory with the same name, w/o a .xacro suffix"""
    urdf_file, ext = os.path.splitext(xacro_file)
    if ext != '.xacro':
        raise RuntimeError(f'Input file to xacro must have a .xacro extension, got {xacro_file}')
    os.system(f'xacro {xacro_file} -o {urdf_file}')
    return urdf_file


def generate_launch_description():
    xacro_file = get_package_file('merlin', 'urdf/merlin.xacro')
    urdf_file = run_xacro(xacro_file)
    srdf_file = get_package_file('merlin_moveit_config', 'config/merlin.srdf')
    kinematics_file = get_package_file('merlin_moveit_config', 'config/kinematics.yaml')
    ompl_config_file = get_package_file('merlin_moveit_config', 'config/ompl_planning.yaml')
    pilz_config_file = get_package_file('merlin_moveit_config', 'config/pilz_planning.yaml')
    pipeline_config_file = get_package_file('merlin_moveit_config', 'config/planning_pipelines_config.yaml')
    moveit_controllers_file = get_package_file('merlin_moveit_config', 'config/moveit_controllers.yaml')
    limits_config_file = get_package_file('merlin_moveit_config', 'config/joint_limits.yaml')

    robot_description = load_file(urdf_file)
    robot_description_semantic = load_file(srdf_file)
    kinematics_config = load_yaml(kinematics_file)
    pipeline_config_file = load_yaml(pipeline_config_file)
    ompl_config = load_yaml(ompl_config_file)
    pilz_config = load_yaml(pilz_config_file)
    limits_config = load_yaml(limits_config_file)

    moveit_controllers = {
        'moveit_simple_controller_manager' : load_yaml(moveit_controllers_file),
        'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager'
    }
    trajectory_execution = {
        'moveit_manage_controllers': False,
        # 'trajectory_execution.execution_duration_monitoring': False,
        'trajectory_execution.allowed_execution_duration_scaling': 1e10, # 1.2,
        'trajectory_execution.use_sim_time': 1e10, # 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 3, # 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.01
    }
    planning_scene_monitor_config = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True
    }

    # MoveIt node

    move_group_capabilities = {
        "capabilities": """pilz_industrial_motion_planner/MoveGroupSequenceAction \
            pilz_industrial_motion_planner/MoveGroupSequenceService"""
    }

    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            {
                'robot_description': robot_description,
                'use_sim_time':True,
                'robot_description_semantic': robot_description_semantic,
                'robot_description_planning': limits_config,
                'robot_description_kinematics': kinematics_config,
                'default_planning_pipeline': 'pilz',
                'planning_pipelines': ['pilz', 'ompl'],
                'ompl': ompl_config,
                'pilz': pilz_config
            },
            # pipeline_config,
            moveit_controllers,
            move_group_capabilities,
            trajectory_execution,
            planning_scene_monitor_config,
        ],
    )
 
    # Visualization (parameters needed for MoveIt display plugin)
    rviz = Node(
        name='rviz',
        package='rviz2',
        executable='rviz2',
        output='screen',
        parameters=[
            {
                'use_sim_time': True,
                'robot_description': robot_description,
                'robot_description_semantic': robot_description_semantic,
                'robot_description_kinematics': kinematics_config,
            }
        ],
    )

    return LaunchDescription([
        move_group_node,
        rviz,
        ] 
    )