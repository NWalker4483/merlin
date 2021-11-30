#ifndef ROS_CONTROL__Merlin_HARDWARE_H
#define ROS_CONTROL__Merlin_HARDWARE_H

#include <boost/scoped_ptr.hpp>
#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <ros/ros.h>

namespace merlin_hardware_interface {
/// \brief Hardware interface for a robot
class MerlinHardware : public hardware_interface::RobotHW {
protected:
  // Interfaces
  hardware_interface::JointStateInterface joint_state_interface_;
  hardware_interface::PositionJointInterface position_joint_interface_;

  joint_limits_interface::PositionJointSaturationInterface
      position_joint_saturation_interface_;
  joint_limits_interface::PositionJointSoftLimitsInterface
      position_joint_limits_interface_;

  // Shared memory
  int num_joints_;
  int joint_mode_; // position, velocity, or effort
  std::vector<std::string> joint_names_;
  std::vector<int> joint_types_;
  std::vector<double> joint_position_;
  std::vector<double> joint_velocity_;

  std::vector<double> joint_position_command_;

  std::vector<double> joint_lower_limits_;
  std::vector<double> joint_upper_limits_;

}; // class

} // namespace merlin_hardware_interface

#endif
