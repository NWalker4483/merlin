#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <merlin_hardware_interface/merlin_hardware_interface.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h> // write(), read(), close()

#include <fcntl.h>
#include <termios.h>

#include <cstddef> // for NULL

// constexpr const char *const SERIAL_PORT_1 = "/dev/ttyACM0";
const int BUFFER_SIZE = 4;

union open_float {
  float val = 0;
  char array[4];
};
// union open_int {
//   int val = 0;
//   char array[4];
// };
using namespace std;
using namespace Eigen;
using namespace hardware_interface;
using joint_limits_interface::JointLimits;
using joint_limits_interface::PositionJointSoftLimitsHandle;
using joint_limits_interface::PositionJointSoftLimitsInterface;
using joint_limits_interface::SoftJointLimits;

namespace merlin_hardware_interface {
MerlinHardwareInterface::MerlinHardwareInterface(ros::NodeHandle &nh)
    : nh_(nh) {
  init();
  controller_manager_.reset(
      new controller_manager::ControllerManager(this, nh_));

  nh_.param("/merlin/hardware_interface/loop_hz", loop_hz_, 0.1);
  nh_.param("/merlin/hardware_interface/port", port_name, "/dev/ttyACM0");
  

  ros::Duration update_freq = ros::Duration(1.0 / loop_hz_);
  non_realtime_loop_ =
      nh_.createTimer(update_freq, &MerlinHardwareInterface::update, this);
}

MerlinHardwareInterface::~MerlinHardwareInterface() {}
void MerlinHardwareInterface::init_serial() {

  /* Open the file descriptor in non-blocking mode */
  serial_port = open(port_name, O_RDWR | O_NOCTTY);

  // Check for errors
  if (serial_port < 0) {
    printf("Error %i from open: %s\n", errno, strerror(errno));
  }
  /* Set up the control structure */
  struct termios toptions;

  /* Get currently set options for the tty */
  tcgetattr(serial_port, &toptions);

  /* Set custom options */

  /* 500000 baud */
  cfsetispeed(&toptions, B500000);
  cfsetospeed(&toptions, B500000);
  /* 8 bits, no parity, no stop bits */
  toptions.c_cflag &= ~PARENB;
  toptions.c_cflag &= ~CSTOPB;
  toptions.c_cflag &= ~CSIZE;
  toptions.c_cflag |= CS8;
  /* no hardware flow control */
  toptions.c_cflag &= ~CRTSCTS;
  /* enable receiver, ignore status lines */
  toptions.c_cflag |= CREAD | CLOCAL;
  /* disable input/output flow control, disable interrupt chars */
  toptions.c_iflag &= ~(IXON | IXOFF | IXANY);
  /* disable canonical input, disable echo,
disable visually erase chars,
disable terminal-generated signals */
  toptions.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  /* disable output processing */
  toptions.c_oflag &= ~OPOST;

  /* wait for 12 characters to come in before read returns */
  /* WARNING! THIS CAUSES THE read() TO BLOCK UNTIL ALL */
  /* CHARACTERS HAVE COME IN! */
  toptions.c_cc[VMIN] = 4;
  /* no minimum time to wait before read returns */
  toptions.c_cc[VTIME] = 10;

  /* commit the options */
  tcsetattr(serial_port, TCSANOW, &toptions);

  /* Wait for the Arduino to reset */
  usleep(1000 * 1000);
  /* Flush anything already in the serial buffer */
  tcflush(serial_port, TCIFLUSH);
}
void MerlinHardwareInterface::init() {
  // Instantiate SerialPort object.
  init_serial();

  // Get joint names
  nh_.getParam("/merlin/hardware_interface/joints", joint_names_);
  num_joints_ = joint_names_.size();

  // Resize vectors
  joint_position_.resize(num_joints_);
  joint_velocity_.resize(num_joints_);
  joint_effort_.resize(num_joints_);

  joint_position_command_.resize(num_joints_);
  joint_velocity_command_.resize(num_joints_);

  last_position_command_.resize(num_joints_);
  last_velocity_command_.resize(num_joints_);

  // Initialize Controller
  for (int i = 0; i < num_joints_; ++i) {
    // Create joint state interface
    JointStateHandle jointStateHandle(joint_names_[i], &joint_position_[i],
                                      &joint_velocity_[i], &joint_effort_[i]);
    joint_state_interface_.registerHandle(jointStateHandle);

    // connect and register the joint position interface
    PosVelJointHandle jointPosVelHandle(jointStateHandle,
                                        &joint_position_command_[i],
                                        &joint_velocity_command_[i]);
    posvelJointInterface.registerHandle(jointPosVelHandle);

    // TODO: Enforce soft joint limits interface
    JointLimits limits;
    getJointLimits(joint_names_[i], nh_, limits);
  };

  registerInterface(&joint_state_interface_);
  registerInterface(&posvelJointInterface);
}

void MerlinHardwareInterface::update(const ros::TimerEvent &e) {
  elapsed_time_ = ros::Duration(e.current_real - e.last_real);
  read();
  controller_manager_->update(ros::Time::now(), elapsed_time_);
  write(elapsed_time_);
}

void MerlinHardwareInterface::read() {
  char msg[] = {'R'};
  ::write(serial_port, msg, sizeof(msg));
  open_float temp;

  for (int i = 0; i < 6; i++) {
    int b = ::read(serial_port, &temp.array, 4);
    joint_position_[i] = temp.val * (3.1415926/180.);
  }
}

void MerlinHardwareInterface::write(ros::Duration elapsed_time) {
  char msg[] = {'W'};
  ::write(serial_port, msg, sizeof(msg));
  // positionJointSoftLimitsInterface.enforceLimits(elapsed_time);

  open_float temp;
  for (int stage = 0; stage < 6; stage++) {
    temp.val = (float)joint_position_command_[stage];

    temp.val *= (180./3.1415926);
    ::write(serial_port, temp.array, 4);
}
  for (int stage = 0; stage < 6; stage++) {
    temp.val = (float)joint_velocity_command_[stage];
    temp.val *= (180./3.1415926);
    ::write(serial_port, temp.array, 4);
}
}
} // namespace merlin_hardware_interface
