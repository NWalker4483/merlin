#include <merlin_hw/merlin_hwi.hpp>

namespace merlin_hardware_interface
{
  MerlinHardwareInterface::MerlinHardwareInterface()
  {

  }

  MerlinHardwareInterface::~MerlinHardwareInterface() {
    
  }

  CallbackReturn MerlinHardwareInterface::read(const rclcpp::Time & time, const rclcpp::Duration & period){
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn MerlinHardwareInterface::write(const rclcpp::Time & time, const rclcpp::Duration & period){
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn MerlinHardwareInterface::on_init(){
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn MerlinHardwareInterface::on_configure(const State &previous_state){
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn MerlinHardwareInterface::on_error(const State &previous_state){
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn MerlinHardwareInterface::on_cleanup(const State &previous_state){
    return CallbackReturn::SUCCESS;
  }
  
  CallbackReturn MerlinHardwareInterface::on_shutdown(const State &previous_state){
    return CallbackReturn::SUCCESS;
  }
  
  CallbackReturn MerlinHardwareInterface::on_activate(const State &previous_state){
    return CallbackReturn::SUCCESS;
  }
  
  CallbackReturn MerlinHardwareInterface::on_deactivate(const State &previous_state){
    return CallbackReturn::SUCCESS;
  }


} // namespace merlin_hardware_interface

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(merlin_hwi::MerlinHardwareInterface, hardware_interface::SystemInterface)