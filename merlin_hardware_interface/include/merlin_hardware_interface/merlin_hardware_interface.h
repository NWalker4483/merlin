#ifndef ROS_CONTROL__Merlin_HARDWARE_INTERFACE_H
#define ROS_CONTROL__Merlin_HARDWARE_INTERFACE_H

#include <boost/scoped_ptr.hpp>
#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/posvel_command_interface.h>
#include <hardware_interface/posvelacc_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <iostream>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <merlin_hardware_interface/merlin_hardware.h>
#include <ros/ros.h>
// C library headers
#include <stdio.h>
#include <string.h>

// Linux headers
#include <errno.h>   // Error integer and strerror() function
#include <fcntl.h>   // Contains file controls like O_RDWR
#include <termios.h> // Contains POSIX terminal control definitions
namespace serial {
#include <unistd.h> // write(), read(), close()
}

using namespace hardware_interface;
using joint_limits_interface::JointLimits;
using joint_limits_interface::PositionJointSoftLimitsHandle;
using joint_limits_interface::PositionJointSoftLimitsInterface;
using joint_limits_interface::SoftJointLimits;

namespace merlin_hardware_interface {

class MerlinHardwareInterface
    : public merlin_hardware_interface::MerlinHardware {
public:
  MerlinHardwareInterface(ros::NodeHandle &nh);
  ~MerlinHardwareInterface();
  void init();
  void init_serial();
  void update(const ros::TimerEvent &e);
  void read();
  void write(ros::Duration elapsed_time);

protected:
  ros::NodeHandle nh_;
  int serial_port;
  ros::Timer non_realtime_loop_;
  ros::Duration control_period_;
  ros::Duration elapsed_time_;
  PositionJointInterface positionJointInterface;
  PosVelJointInterface posvelJointInterface;
  PositionJointSoftLimitsInterface positionJointSoftLimitsInterface;
  double loop_hz_;
  boost::shared_ptr<controller_manager::ControllerManager> controller_manager_;
  double p_error_, v_error_, e_error_;
};

} // namespace merlin_hardware_interface

#endif
