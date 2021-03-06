#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <merlin_hardware_interface/merlin_hardware_interface.h>

#include <stdio.h>
#include <stdlib.h>

#include <fcntl.h>
#include <termios.h>

#include <cstddef> // for NULL
#include <ros/console.h>

union open_float
{
  float val = 0;
  char array[4];
};

using namespace std;
using namespace hardware_interface;
using joint_limits_interface::JointLimits;
using joint_limits_interface::PositionJointSoftLimitsHandle;
using joint_limits_interface::PositionJointSoftLimitsInterface;
using joint_limits_interface::SoftJointLimits;

namespace merlin_hardware_interface
{
  MerlinHardwareInterface::MerlinHardwareInterface(ros::NodeHandle &nh)
      : nh_(nh)
  {
    init();
    controller_manager_.reset(
        new controller_manager::ControllerManager(this, nh_));

    nh_.param("/merlin/hardware_interface/loop_hz", loop_hz_, 0.1);
    nh.param<std::string>("/merlin/hardware_interface/port", port_name, "/dev/ttyACM0");

    ros::Duration update_freq = ros::Duration(1.0 / loop_hz_);
    non_realtime_loop_ =
        nh_.createTimer(update_freq, &MerlinHardwareInterface::update, this);
  }

  MerlinHardwareInterface::~MerlinHardwareInterface() {}
  
  void MerlinHardwareInterface::init()
  {
    // Get joint names
    nh_.getParam("/merlin/hardware_interface/joints", joint_names_);
    num_joints_ = joint_names_.size();

    // Resize vectors
    joint_position_.resize(num_joints_);
    joint_velocity_.resize(num_joints_);
    joint_acceleration_.resize(num_joints_);
    joint_effort_.resize(num_joints_);

    joint_position_command_.resize(num_joints_);
    joint_velocity_command_.resize(num_joints_);
    joint_acceleration_command_.resize(num_joints_);

    // last_position_command_.resize(num_joints_);
    // last_velocity_command_.resize(num_joints_);

    // Initialize Controller
    for (int i = 0; i < num_joints_; ++i)
    {
      // Create joint state interface
      JointStateHandle jointStateHandle(joint_names_[i], &joint_position_[i],
                                        &joint_velocity_[i], &joint_effort_[i]);
      joint_state_interface_.registerHandle(jointStateHandle);

      // connect and register the joint position velocity interface
      PosVelJointHandle jointPosVelHandle(jointStateHandle,
                                          &joint_position_command_[i],
                                          &joint_velocity_command_[i]);
      posvelJointInterface.registerHandle(jointPosVelHandle);

       // connect and register the joint position velocity acceleration interface
      PosVelAccJointHandle jointPosVelAccHandle(jointStateHandle,
                                          &joint_position_command_[i],
                                          &joint_velocity_command_[i],
                                          &joint_acceleration_command_[i]);
      posvelaccJointInterface.registerHandle(jointPosVelAccHandle);

      // TODO: Setup enforce soft joint limits interface
      JointLimits limits;
      getJointLimits(joint_names_[i], nh_, limits);
    };

    registerInterface(&joint_state_interface_);
    registerInterface(&posvelJointInterface);
    registerInterface(&posvelaccJointInterface);
  }

  void MerlinHardwareInterface::update(const ros::TimerEvent &e)
  {
    elapsed_time_ = ros::Duration(e.current_real - e.last_real);
    read();
    controller_manager_->update(ros::Time::now(), elapsed_time_);
    write(elapsed_time_);
  }

  void MerlinHardwareInterface::read()
  {

    for (int i = 0; i < 6; i++)
    {
      int b = ::read(serial_port, &temp.array, 4);
      // Convert from degrees to radians
      joint_position_[i] = temp.val * (3.1415926 / 180.);
    }
  }

  void MerlinHardwareInterface::write(ros::Duration elapsed_time)
  {

    // positionJointSoftLimitsInterface.enforceLimits(elapsed_time);

    open_float temp;
    for (int stage = 0; stage < 6; stage++)
    {
      temp.val = (float)joint_position_command_[stage];
      // Convert from radians to degrees
      temp.val *= (180. / 3.1415926);
      ::write(serial_port, temp.array, 4);
    }
    for (int stage = 0; stage < 6; stage++)
    {
      temp.val = (float)joint_velocity_command_[stage];
      // Convert from radians to degrees
      temp.val *= (180. / 3.1415926);
      ::write(serial_port, temp.array, 4);
    }
  }
} // namespace merlin_hardware_interface
