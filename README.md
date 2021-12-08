# A set of ROS Packages for simulating and controlling the *American Robot* Merlin MR6500 Robotic Arm (In Progress)
These packages contain a:
*  URDF Description
*  Arduino Hardware Interface
*  Gazebo Model
*  MoveIt Configuration
  
|||
|------|------|
|<img src="assets/sim.png" alt="Simulation Image" width="500"></img>|<img src="assets/weld.jpeg" alt="Welding Image" width="500"></img>|

go into the move it controller manager XML file and add the line
<arg name="execution_type" default="unused"/>

after the move it config has been generated change the controller name in Ross controllers. from arm controller to Merlin/arm controller

Read This: https://medium.com/@tahsincankose/custom-manipulator-simulation-in-gazebo-and-motion-planning-with-moveit-c017eef1ea90