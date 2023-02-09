import os 

from ament_index_python.packages import get_package_share_directory 


from launch import LaunchDescription 
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler 
from launch.event_handlers import (OnProcessStart, OnProcessExit) 
from launch.launch_description_sources import PythonLaunchDescriptionSource 

from launch_ros.actions import Node 

import xacro 

def generate_launch_description(): 
    gazebo = IncludeLaunchDescription( 
                PythonLaunchDescriptionSource([os.path.join( 
                    get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
            ) 
    package_path = os.path.join( get_package_share_directory('merlin')) 
    xacro_file = os.path.join(package_path, 'urdf', 'merlin.xacro') 
    doc = xacro.parse(open(xacro_file)) 
    xacro.process_doc(doc)

    node_robot_state_publisher = Node( 
        package = 'robot_state_publisher',
        executable='robot_state_publisher', 
        output='both', 
        parameters=[
        {'robot_description':doc.toxml(),
        "use_sim_time": True}
        ])

    load_joint_state_controller = ExecuteProcess(
        cmd = ['ros2', 'control', 'load_controller','--set-state','active','joint_state_broadcaster'],
        output='screen')

    load_arm_controller = ExecuteProcess(
        cmd = ['ros2', 'control', 'load_controller','--set-state','active','arm_controller'],
        output='screen')
    
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py', 
                        arguments=[ '-topic', '/robot_description', 
                        '-entity', 'merlin'],
                        output= 'screen') 
                        
    return LaunchDescription([
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[load_joint_state_controller])),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[load_arm_controller])),
        gazebo, 
        node_robot_state_publisher, 
        spawn_entity])
# Removing ros-humble-pilz-industrial-motion-planner (2.5.4-1jammy.20230124.083608) ...
# Removing ros-humble-moveit-ros-planning-interface (2.5.4-1jammy.20230124.081325) ...
# Removing ros-humble-moveit-ros-warehouse (2.5.4-1jammy.20230120.233218) ...
# Removing ros-humble-moveit-ros-move-group (2.5.4-1jammy.20230124.071138) ...
# Removing ros-humble-moveit-kinematics (2.5.4-1jammy.20230120.233211) ...
# Removing ros-humble-moveit-ros-occupancy-map-monitor (2.5.4-1jammy.20230120.230915) ...
# Removing ros-humble-moveit-core (2.5.4-1jammy.20230120.223848) ...
# Removing ros-humble-moveit-msgs (2.2.1-1jammy.20230112.160645) ...
# Removing ros-humble-moveit-common (2.5.4-1jammy.20230112.142325) ...