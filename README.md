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

## TODD: 
* Test teensy 4.1 with ROS Control 
* Added Serial Connection Checking to the hardware interface
  * When the usb isnt connect it should throw a std ROS error
* Get Robot port from hardware.yaml file instead of hardcoding it 
* 
Read This: https://medium.com/@tahsincankose/custom-manipulator-simulation-in-gazebo-and-motion-planning-with-moveit-c017eef1ea90