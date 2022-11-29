#ifndef ROS_CONTROL__Merlin_HARDWARE_INTERFACE_H
#define ROS_CONTROL__Merlin_HARDWARE_INTERFACE_H

#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include <hardware_interface/System_Interface.hpp>
#include <rclcpp_lifecycle/state.hpp>
//#include "merlin_control.h"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using State = rclcpp_lifecycle::State;
namespace merlin_hardware_interface
{
  class MerlinHardwareInterface : public hardware_interface::SystemInterface
  {
  public:
   // Merlin driver 
    MerlinHardwareInterface();
    ~MerlinHardwareInterface();

    CallbackReturn read(const rclcpp::Time & time, const rclcpp::Duration & period);
    CallbackReturn write(const rclcpp::Time & time, const rclcpp::Duration & period);

    CallbackReturn on_init();

    CallbackReturn on_configure(const State &previous_state);
    CallbackReturn on_error(const State &previous_state);
    CallbackReturn on_cleanup(const State &previous_state);
    CallbackReturn on_shutdown(const State &previous_state);
    CallbackReturn on_activate(const State &previous_state);
    CallbackReturn on_deactivate(const State &previous_state);

    std::vector<hardware_interface::StateInterface> export_state_interfaces();
    std::vector<hardware_interface::CommandInterface> export_command_interfaces();
  };
} // namespace merlin_hardware_interface

#endif
