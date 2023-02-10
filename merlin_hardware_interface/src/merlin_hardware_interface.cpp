#include "merlin_hwi/merlin_hardware_interface.hpp"
#define JOINT_DOF 6
namespace merlin_hardware_interface {
hardware_interface::return_type
MerlinHardwareInterface::configure(const hardware_interface::HardwareInfo &info) {
  node_ = std::make_shared<rclcpp::Node>("fri_hardware_interface_node");

  auto ret = configure_default(info);
  if (ret != hardware_interface::return_type::OK) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to initialize SystemInterface.");
    return ret;
  }

  if (!init_lloyd_()) {
    return hardware_interface::return_type::ERROR;
  }

  if (!init_command_interfaces_()) {
    return hardware_interface::return_type::ERROR;
  }

  if (!init_state_interfaces_()) {
    return hardware_interface::return_type::ERROR;
  }

  if (!init_last_hw_states_()) {
    return hardware_interface::return_type::ERROR;
  }

  if (!verify_number_of_joints_()) {
    return hardware_interface::return_type::ERROR;
  }

  if (!verify_joint_command_interfaces_()) {
    return hardware_interface::return_type::ERROR;
  }

  if (!verify_joint_state_interfaces_()) {
    return hardware_interface::return_type::ERROR;
  }

  // read port_id and remote_host from lbr.ros2_control.xacro
  port_id_ = std::stoul(info_.hardware_parameters["port_id"]);
  info_.hardware_parameters["remote_host"] == "INADDR_ANY"
      ? remote_host_ = NULL
      : remote_host_ = info_.hardware_parameters["remote_host"].c_str();

  if (port_id_ < 30200 || port_id_ > 30209) {
    RCLCPP_ERROR(node_->get_logger(), "Expected port_id in [30200, 30209]. Found %d.", port_id_);
    return hardware_interface::return_type::ERROR;
  }

  if (!spawn_rt_layer_()) {
    return hardware_interface::return_type::ERROR;
  }

  if (!spawn_clients_()) {
    return hardware_interface::return_type::ERROR;
  }

  node_thread_ = std::make_unique<std::thread>([this]() {
    rclcpp::spin(node_);
    disconnect_();
  });

  return hardware_interface::return_type::OK;
}

std::vector<hardware_interface::StateInterface> MerlinHardwareInterface::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;

//   const auto &lbr_fri_sensor = info_.sensors[0];
//   state_interfaces.emplace_back(lbr_fri_sensor.name, HW_IF_SAMPLE_TIME, &hw_sample_time_);
//   state_interfaces.emplace_back(lbr_fri_sensor.name, HW_IF_SESSION_STATE, &hw_session_state_);
//   state_interfaces.emplace_back(lbr_fri_sensor.name, HW_IF_CONNECTION_QUALITY,
//                                 &hw_connection_quality_);
//   state_interfaces.emplace_back(lbr_fri_sensor.name, HW_IF_SAFETY_STATE, &hw_safety_state_);
//   state_interfaces.emplace_back(lbr_fri_sensor.name, HW_IF_OPERATION_MODE, &hw_operation_mode_);
//   state_interfaces.emplace_back(lbr_fri_sensor.name, HW_IF_DRIVE_STATE, &hw_drive_state_);
//   state_interfaces.emplace_back(lbr_fri_sensor.name, HW_IF_CLIENT_COMMAND_MODE,
//                                 &hw_client_command_mode_);
//   state_interfaces.emplace_back(lbr_fri_sensor.name, HW_IF_OVERLAY_TYPE, &hw_overlay_type_);
//   state_interfaces.emplace_back(lbr_fri_sensor.name, HW_IF_CONTROL_MODE, &hw_control_mode_);

//   state_interfaces.emplace_back(lbr_fri_sensor.name, HW_IF_TIME_STAMP_SEC, &hw_time_stamp_sec_);
//   state_interfaces.emplace_back(lbr_fri_sensor.name, HW_IF_TIME_STAMP_NANO_SEC,
//                                 &hw_time_stamp_nano_sec_);

  for (std::size_t i = 0; i < info_.joints.size(); ++i) {
    state_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION,
                                  &hw_position_[i]);

    state_interfaces.emplace_back(info_.joints[i].name, HW_IF_COMMANDED_JOINT_POSITION,
                                  &hw_commanded_joint_position_[i]);

    state_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY,
                                  &hw_velocity_[i]);

    state_interfaces.emplace_back(info_.joints[i].name, HW_IF_COMMANDED_TORQUE,
                                  &hw_commanded_torque_[i]);

    state_interfaces.emplace_back(info_.joints[i].name, HW_IF_EXTERNAL_TORQUE,
                                  &hw_external_torque_[i]);

    state_interfaces.emplace_back(info_.joints[i].name, HW_IF_IPO_JOINT_POSITION,
                                  &hw_ipo_joint_position_[i]);

    state_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY,
                                  &hw_velocity_[i]);
  }

  state_interfaces.emplace_back(lbr_fri_sensor.name, HW_IF_TRACKING_PERFORMANCE,
                                &hw_tracking_performance_);

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
MerlinHardwareInterface::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (std::size_t i = 0; i < info_.joints.size(); ++i) {
    command_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION,
                                    &hw_position_command_[i]);

    command_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY,
                                    &hw_velocity_command_[i]);
  }

  return command_interfaces;
}

hardware_interface::return_type MerlinHardwareInterface::prepare_command_mode_switch(
    const std::vector<std::string> & /*start_interfaces*/,
    const std::vector<std::string> & /*stop_interfaces*/) {
  // if (!command_mode_init_)
  // {
  //     command_mode_init_ = true;
  //     return hardware_interface::return_type::OK;
  // }
  // else
  // {
  // if (start_interfaces != stop_interfaces)
  // {
  //     RCLCPP_ERROR(
  //         node_->get_logger(),
  //         "FRI does not support command mode switches.");
  //     return hardware_interface::return_type::ERROR;
  // }
  // else
  // {
  //     return hardware_interface::return_type::OK;
  // }
  // }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MerlinHardwareInterface::start() {
  if (!connect_()) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to connect to robot.");
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MerlinHardwareInterface::stop() {
  if (!disconnect_()) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to disconnect from robot.");
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MerlinHardwareInterface::read() {
  lbr_->state = *rt_lbr_state_buf_->readFromRT();

  if (!lbr_->state) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to read lbr_state.");
    return hardware_interface::return_type::ERROR;
  }

  if (lbr_->valid_state()) {
    // hw_sample_time_ = lbr_->state->sample_time;
    // hw_session_state_ = static_cast<double>(lbr_->state->session_state);
    // hw_connection_quality_ = static_cast<double>(lbr_->state->connection_quality);
    // hw_safety_state_ = static_cast<double>(lbr_->state->safety_state);
    // hw_operation_mode_ = static_cast<double>(lbr_->state->operation_mode);
    // hw_drive_state_ = static_cast<double>(lbr_->state->drive_state);
    // hw_client_command_mode_ = static_cast<double>(lbr_->state->client_command_mode);
    // hw_overlay_type_ = static_cast<double>(lbr_->state->overlay_type);
    // hw_control_mode_ = static_cast<double>(lbr_->state->control_mode);

    // hw_time_stamp_sec_ = static_cast<double>(lbr_->state->time_stamp_sec);
    // hw_time_stamp_nano_sec_ = static_cast<double>(lbr_->state->time_stamp_nano_sec);

    hw_position_.assign(lbr_->state->measured_joint_position.begin(),
                        lbr_->state->measured_joint_position.end());
    hw_commanded_joint_position_.assign(lbr_->state->commanded_joint_position.begin(),
                                        lbr_->state->commanded_joint_position.end());
    hw_velocity_.assign(lbr_->state->measured_torque.begin(), lbr_->state->measured_torque.end());
    hw_commanded_torque_.assign(lbr_->state->commanded_torque.begin(),
                                lbr_->state->commanded_torque.end());
    hw_external_torque_.assign(lbr_->state->external_torque.begin(),
                               lbr_->state->external_torque.end());
    hw_ipo_joint_position_.assign(lbr_->state->ipo_joint_position.begin(),
                                  lbr_->state->ipo_joint_position.end());

    hw_tracking_performance_ = lbr_->state->tracking_performance;

    compute_hw_velocity_();
    if (!update_last_hw_states_()) {
      return hardware_interface::return_type::ERROR;
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MerlinHardwareInterface::write() {
  lbr_->command->joint_position.assign(hw_position_command_.begin(), hw_position_command_.end());
  lbr_->command->torque.assign(hw_velocity_command_.begin(), hw_velocity_command_.end());
  if (lbr_->valid_command(lbr_->command)) {
    if (rt_lbr_command_pub_->trylock()) {
      rt_lbr_command_pub_->msg_ = *lbr_->command;
      rt_lbr_command_pub_->unlockAndPublish();
    }
  }
  return hardware_interface::return_type::OK;
}

// TODO: re-load controllers
// auto re_load_ctrl = [this]() -> void {

//     // poll current controllers
//     auto list_ctrl_request =
//     std::make_shared<controller_manager_msgs::srv::ListControllers::Request>(); auto
//     list_ctrl_future = list_ctrl_clt_->async_send_request(list_ctrl_request); if
//     (rclcpp::spin_until_future_complete(node_, list_ctrl_future, std::chrono::milliseconds(100))
//     != rclcpp::FutureReturnCode::SUCCESS) {
//         RCLCPP_ERROR(node_->get_logger(), "Failed to call service %s.",
//         list_ctrl_clt_->get_service_name()); return;
//     };

//     std::vector<std::string> controllers;
//     for (auto& controller: list_ctrl_future.get()->controller) {
//         RCLCPP_INFO(node_->get_logger(), "Found controller: %s", controller.name.c_str());
//         controllers.push_back(controller.name);
//     }

//     if (controllers.size() == 0) {
//         RCLCPP_INFO(node_->get_logger(), "No controllers found.");
//         return;
//     }

//     // switch current controllers
//     auto switch_ctrl_request =
//     std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
//     switch_ctrl_request->strictness =
//     controller_manager_msgs::srv::SwitchController::Request::STRICT;
//     switch_ctrl_request->start_asap = true;
//     switch_ctrl_request->stop_controllers = controllers;
//     switch_ctrl_request->start_controllers = controllers;
//     auto switch_ctrl_future = switch_ctrl_clt_->async_send_request(switch_ctrl_request);

//     // wait until switch_ctrl_future
//     if (rclcpp::spin_until_future_complete(node_, switch_ctrl_future,
//     std::chrono::milliseconds(100)) == rclcpp::FutureReturnCode::SUCCESS) {
//         if (switch_ctrl_future.get()->ok) {
//             RCLCPP_INFO(node_->get_logger(), "Re-loaded controllers.");

//             // set commanded state to current state, see
//             https://github.com/ros-controls/ros2_control/issues/674 hw_position_command_ =
//             hw_position_; hw_velocity_command_ = hw_velocity_; fri_and_controllers_in_sync_ = true;
//         } else {
//             RCLCPP_ERROR(node_->get_logger(), "Failed to re-load controllers.");
//         }
//     } else {
//         RCLCPP_ERROR(node_->get_logger(), "Failed to call service %s.",
//         switch_ctrl_clt_->get_service_name());
//     }
// };

// auto re_load_ctrl_thread = std::thread(re_load_ctrl);
// re_load_ctrl_thread.detach();

template <typename ServiceT>
bool MerlinHardwareInterface::wait_for_service_(
    const typename rclcpp::Client<ServiceT>::SharedPtr client, const uint8_t &attempts,
    const std::chrono::seconds &timeout) {
  bool spawned = false;
  uint8_t attempt = 0;
  while (attempt < attempts && !spawned && rclcpp::ok()) {
    RCLCPP_INFO(node_->get_logger(), "Waiting for service %s...", client->get_service_name());
    spawned = client->wait_for_service(timeout);
    ++attempt;
  }
  return spawned;
}

bool MerlinHardwareInterface::init_lloyd_() {
  try {
    lbr_ = std::make_unique<lbr_fri_ros2::Lloyd>();
  } catch (const std::exception &e) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to initialize LBR.\n%s.", e.what());
    return false;
  }
  return true;
}

bool MerlinHardwareInterface::init_command_interfaces_() {
  try {
    hw_position_command_.resize(JOINT_DOF,
                                std::numeric_limits<double>::quiet_NaN());
    hw_velocity_command_.resize(JOINT_DOF,
                              std::numeric_limits<double>::quiet_NaN());
  } catch (const std::exception &e) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to initialize command interfaces.\n%s", e.what());
    return false;
  }
  return true;
}

bool MerlinHardwareInterface::init_state_interfaces_() {
  try {
    hw_sample_time_ = std::numeric_limits<double>::quiet_NaN();
    hw_session_state_ = std::numeric_limits<double>::quiet_NaN();
    hw_connection_quality_ = std::numeric_limits<double>::quiet_NaN();
    hw_safety_state_ = std::numeric_limits<double>::quiet_NaN();
    hw_operation_mode_ = std::numeric_limits<double>::quiet_NaN();
    hw_drive_state_ = std::numeric_limits<double>::quiet_NaN();
    hw_client_command_mode_ = std::numeric_limits<double>::quiet_NaN();
    hw_overlay_type_ = std::numeric_limits<double>::quiet_NaN();
    hw_control_mode_ = std::numeric_limits<double>::quiet_NaN();

    hw_time_stamp_sec_ = std::numeric_limits<double>::quiet_NaN();
    hw_time_stamp_nano_sec_ = std::numeric_limits<double>::quiet_NaN();

    hw_position_.resize(JOINT_DOF, std::numeric_limits<double>::quiet_NaN());
    hw_commanded_joint_position_.resize(JOINT_DOF,
                                        std::numeric_limits<double>::quiet_NaN());
    hw_velocity_.resize(JOINT_DOF, std::numeric_limits<double>::quiet_NaN());
    hw_commanded_torque_.resize(JOINT_DOF,
                                std::numeric_limits<double>::quiet_NaN());
    hw_external_torque_.resize(JOINT_DOF,
                               std::numeric_limits<double>::quiet_NaN());
    hw_ipo_joint_position_.resize(JOINT_DOF,
                                  std::numeric_limits<double>::quiet_NaN());
    hw_tracking_performance_ = std::numeric_limits<double>::quiet_NaN();

    hw_velocity_.resize(JOINT_DOF, std::numeric_limits<double>::quiet_NaN());
  } catch (const std::exception &e) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to initialize state interfaces.\n%s", e.what());
    return false;
  }
  return true;
}

bool MerlinHardwareInterface::verify_number_of_joints_() {
  if (info_.joints.size() != JOINT_DOF) {
    RCLCPP_ERROR(node_->get_logger(), "Expected %d joints in URDF. Found %ld.",
                 JOINT_DOF, info_.joints.size());
    return false;
  }
  return true;
}

bool MerlinHardwareInterface::verify_joint_command_interfaces_() {
  // check command interfaces
  for (auto &joint : info_.joints) {
    if (joint.command_interfaces.size() != LBR_FRI_COMMAND_INTERFACE_SIZE) {
      RCLCPP_ERROR(
          node_->get_logger(),
          "Joint %s received invalid number of command interfaces. Received %ld, expected %d.",
          joint.name.c_str(), joint.command_interfaces.size(), LBR_FRI_COMMAND_INTERFACE_SIZE);
      return false;
    }
    for (auto &ci : joint.command_interfaces) {
      if (ci.name != hardware_interface::HW_IF_POSITION &&
          ci.name != hardware_interface::HW_IF_VELOCITY) {
        RCLCPP_ERROR(node_->get_logger(),
                     "Joint %s received invalid command interface: %s. Expected %s or %s.",
                     joint.name.c_str(), ci.name.c_str(), hardware_interface::HW_IF_POSITION,
                     hardware_interface::HW_IF_VELOCITY);
        return false;
      }
    }
  }
  return true;
}

bool MerlinHardwareInterface::verify_joint_state_interfaces_() {
  // check state interfaces
  for (auto &joint : info_.joints) {
    if (joint.state_interfaces.size() != LBR_FRI_STATE_INTERFACE_SIZE) {
      RCLCPP_ERROR(
          node_->get_logger(),
          "Joint %s received invalid number of state interfaces. Received %ld, expected %d.",
          joint.name.c_str(), joint.state_interfaces.size(), LBR_FRI_STATE_INTERFACE_SIZE);
      return false;
    }
    for (auto &si : joint.state_interfaces) {
      if (si.name != hardware_interface::HW_IF_POSITION &&
          si.name != HW_IF_COMMANDED_JOINT_POSITION &&
          si.name != hardware_interface::HW_IF_VELOCITY && si.name != HW_IF_COMMANDED_TORQUE &&
          si.name != HW_IF_EXTERNAL_TORQUE && si.name != HW_IF_IPO_JOINT_POSITION &&
          si.name != hardware_interface::HW_IF_VELOCITY) {
        RCLCPP_ERROR(
            node_->get_logger(),
            "Joint %s received invalid state interface: %s. Expected %s, %s, %s, %s, %s, %s or %s.",
            joint.name.c_str(), si.name.c_str(), hardware_interface::HW_IF_POSITION,
            HW_IF_COMMANDED_JOINT_POSITION, hardware_interface::HW_IF_VELOCITY,
            HW_IF_COMMANDED_TORQUE, HW_IF_EXTERNAL_TORQUE, HW_IF_IPO_JOINT_POSITION,
            hardware_interface::HW_IF_VELOCITY);
        return false;
      }
    }
  }
  return true;
}


bool MerlinHardwareInterface::spawn_rt_layer_() {
  if (!node_) {
    printf("No node provided.\n");
    return false;
  }

  try {
    rt_lbr_state_buf_ =
        std::make_shared<realtime_tools::RealtimeBuffer<lbr_fri_msgs::msg::LBRState::SharedPtr>>(
            nullptr);
    lbr_state_sub_ = node_->create_subscription<lbr_fri_msgs::msg::LBRState>(
        "/lbr_state", rclcpp::SystemDefaultsQoS(),
        std::bind(&MerlinHardwareInterface::lbr_state_cb_, this, std::placeholders::_1));
    lbr_command_pub_ = node_->create_publisher<lbr_fri_msgs::msg::LBRCommand>(
        "/lbr_command", rclcpp::SystemDefaultsQoS());
    rt_lbr_command_pub_ =
        std::make_shared<realtime_tools::RealtimePublisher<lbr_fri_msgs::msg::LBRCommand>>(
            lbr_command_pub_);
  } catch (const std::exception &e) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to spawn real time layer.\n%s.", e.what());
    return false;
  }
  return true;
}

bool MerlinHardwareInterface::spawn_clients_() {
  if (!node_) {
    printf("No node provided.\n");
    return false;
  }

  list_ctrl_clt_ = node_->create_client<controller_manager_msgs::srv::ListControllers>(
      "/controller_manager/list_controllers", rmw_qos_profile_system_default);
  switch_ctrl_clt_ = node_->create_client<controller_manager_msgs::srv::SwitchController>(
      "/controller_manager/switch_controller", rmw_qos_profile_system_default);

  app_connect_clt_ = node_->create_client<lbr_fri_msgs::srv::AppConnect>(
      "/lbr_app/connect", rmw_qos_profile_system_default);
  if (!wait_for_service_<lbr_fri_msgs::srv::AppConnect>(app_connect_clt_)) {
    RCLCPP_ERROR(node_->get_logger(), "Failed.");
    return false;
  }
  RCLCPP_INFO(node_->get_logger(), "Done.");

  app_disconnect_clt_ = node_->create_client<lbr_fri_msgs::srv::AppDisconnect>(
      "/lbr_app/disconnect", rmw_qos_profile_system_default);
  if (!wait_for_service_<lbr_fri_msgs::srv::AppDisconnect>(app_disconnect_clt_)) {
    RCLCPP_ERROR(node_->get_logger(), "Failed.");
    return false;
  }
  RCLCPP_INFO(node_->get_logger(), "Done.");
  return true;
}

bool MerlinHardwareInterface::connect_() {
  auto connect_request = std::make_shared<lbr_fri_msgs::srv::AppConnect::Request>();
  connect_request->port_id = port_id_;
  connect_request->remote_host = remote_host_ ? remote_host_ : "";
  auto future = app_connect_clt_->async_send_request(connect_request);
  auto status = future.wait_for(std::chrono::seconds(1));
  if (status != std::future_status::ready) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to request connect service %s.",
                 app_connect_clt_->get_service_name());
    return false;
  }
  auto response = future.get();
  if (!response->connected) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to connect.\n%s", response->message.c_str());
  }
  return response->connected;
}

bool MerlinHardwareInterface::disconnect_() {
  auto disconnect_request = std::make_shared<lbr_fri_msgs::srv::AppDisconnect::Request>();
  auto future = app_disconnect_clt_->async_send_request(disconnect_request);
  auto status = future.wait_for(std::chrono::seconds(1));
  if (status != std::future_status::ready) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to request disconnect service %s.",
                 app_disconnect_clt_->get_service_name());
    return false;
  }
  auto response = future.get();
  if (!response->disconnected) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to disconnect.\n%s", response->message.c_str());
  }
  return response->disconnected;
}

void MerlinHardwareInterface::lbr_state_cb_(const lbr_fri_msgs::msg::LBRState::SharedPtr lbr_state) {
  rt_lbr_state_buf_->writeFromNonRT(lbr_state);
}

double MerlinHardwareInterface::time_stamps_to_sec_(const double &sec, const double &nano_sec) const {
  return sec + nano_sec / 1.e9;
}

bool MerlinHardwareInterface::init_last_hw_states_() {
  try {
    last_hw_position_.resize(JOINT_DOF,
                             std::numeric_limits<double>::quiet_NaN());
    last_hw_time_stamp_sec_ = std::numeric_limits<double>::quiet_NaN();
    last_hw_time_stamp_nano_sec_ = std::numeric_limits<double>::quiet_NaN();
  } catch (const std::exception &e) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to init last hw states.");
    return false;
  }
  return true;
}

bool MerlinHardwareInterface::update_last_hw_states_() {
  try {
    last_hw_position_ = hw_position_;
    last_hw_time_stamp_sec_ = hw_time_stamp_sec_;
    last_hw_time_stamp_nano_sec_ = hw_time_stamp_nano_sec_;
  } catch (const std::exception &e) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to update last hw states.");
    return false;
  }
  return true;
}

void MerlinHardwareInterface::compute_hw_velocity_() {
  // state uninitialized
  if (std::isnan(last_hw_time_stamp_nano_sec_) || std::isnan(last_hw_position_[0])) {
    return;
  }

  // state wasn't updated
  if (last_hw_time_stamp_sec_ == hw_time_stamp_sec_ &&
      last_hw_time_stamp_nano_sec_ == hw_time_stamp_nano_sec_) {
    return;
  }

  for (uint8_t i = 0; i < JOINT_DOF; ++i) {
    hw_velocity_[i] = (hw_position_[i] - last_hw_position_[i]) /
                      (time_stamps_to_sec_(hw_time_stamp_sec_, hw_time_stamp_nano_sec_) -
                       time_stamps_to_sec_(last_hw_time_stamp_sec_, last_hw_time_stamp_nano_sec_));
  }
}

} // end of namespace lbr_hardware_interface

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(merlin_hardware_interface::MerlinHardwareInterface, hardware_interface::SystemInterface)