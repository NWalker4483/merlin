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

// #include <iostream>
// #include <Eigen/Dense>

// using namespace std;
// using namespace Eigen;

// constexpr const char *const SERIAL_PORT_1 = "/dev/ttyACM0";
const int BUFFER_SIZE = 4;

union open_float {
  float val = 0;
  char array[4];
};

using namespace hardware_interface;
using joint_limits_interface::JointLimits;
using joint_limits_interface::PositionJointSoftLimitsHandle;
using joint_limits_interface::PositionJointSoftLimitsInterface;
using joint_limits_interface::SoftJointLimits;
using namespace std;
using namespace Eigen;


namespace merlin_hardware_interface {
MerlinHardwareInterface::MerlinHardwareInterface(ros::NodeHandle &nh)
    : nh_(nh) {
  init();
  controller_manager_.reset(
      new controller_manager::ControllerManager(this, nh_));
  nh_.param("/merlin/hardware_interface/loop_hz", loop_hz_, 0.1);
  ros::Duration update_freq = ros::Duration(1.0 / loop_hz_);
  non_realtime_loop_ =
      nh_.createTimer(update_freq, &MerlinHardwareInterface::update, this);
}

MerlinHardwareInterface::~MerlinHardwareInterface() {}
void MerlinHardwareInterface::init_serial() {

  /* Open the file descriptor in non-blocking mode */
  serial_port = open("/dev/ttyACM0", O_RDWR | O_NOCTTY);

  // Check for errors
  if (serial_port < 0) {
    printf("Error %i from open: %s\n", errno, strerror(errno));
  }
  /* Set up the control structure */
  struct termios toptions;

  /* Get currently set options for the tty */
  tcgetattr(serial_port, &toptions);

  /* Set custom options */

  /* 9600 baud */
  cfsetispeed(&toptions, B115200);
  cfsetospeed(&toptions, B115200);
  /* 8 bits, no parity, no stop bits */
  toptions.c_cflag &= ~PARENB;
  toptions.c_cflag &= ~CSTOPB;
  toptions.c_cflag &= ~CSIZE;
  toptions.c_cflag |= CS8;
  /* no hardware flow control */
  toptions.c_cflag &= ~CRTSCTS;
  /* enable receiver, ignore status lines */
  toptions.c_cflag |= CREAD | CLOCAL;
  /* disable input/output flow control, disable restart chars */
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
  // 
  motor_reductions << 
      1. / 48., 0, 0, 0, 0, 0,
      0, 1. / 48., 1. / 48., 0, 0, 0,
      0, 0, 1. / 48., 0, 0, 0,
      0, 0, 0, 1. / 24., -1. / 28.8, 1. / 12.,
      0, 0, 0, 0, 1. / 28.8, -1. / 24.,
      0, 0, 0, 0, 0, 1. / 24.;

  motor_spr << 200, 200, 200, 200, 200, 200;
  degrees_per_step = (360./200.) * motor_reductions;
  degrees_per_step_inv = degrees_per_step.transpose().inverse();
  // Get joint names
  nh_.getParam("/merlin/hardware_interface/joints", joint_names_);
  num_joints_ = joint_names_.size();

  // Resize vectors
  joint_position_.resize(num_joints_);
  joint_velocity_.resize(num_joints_);
  joint_effort_.resize(num_joints_);
  joint_position_command_.resize(num_joints_);
  joint_velocity_command_.resize(num_joints_);

  // Initialize Controller
  for (int i = 0; i < num_joints_; ++i) {
    // Create joint state interface
    JointStateHandle jointStateHandle(joint_names_[i], &joint_position_[i],
                                      &joint_velocity_[i], &joint_effort_[i]);
    joint_state_interface_.registerHandle(jointStateHandle);

    PosVelJointHandle jointPosVelHandle(jointStateHandle, &joint_position_[i],
                                        &joint_velocity_[i]);
    JointLimits limits;
    getJointLimits(joint_names_[i], nh_, limits);

    posvelJointInterface.registerHandle(jointPosVelHandle);
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

  for (int i = 0; i < num_joints_; i++) {
    open_float index;
    open_float pulses;

    // Read bytes. The behaviour of read() (e.g. does it block?,
    // how long does it block for?) depends on the configuration
    // settings above, specifically VMIN and VTIME

    int a = ::read(serial_port, &index.array, 4);
    int b = ::read(serial_port, &pulses.array, 4);

    // joint_position_[i] = (double)temp.val;
  }
}

void MerlinHardwareInterface::write(ros::Duration elapsed_time) {
 
  if (last_position_command_!= joint_position_command_ ){
last_position_command_ = joint_position_command_;
    
  } else {return;} char msg[] = {'W'};
  ::write(serial_port, msg, sizeof(msg));

  open_float step_cmd_holder[6][2][6];
  int cmd_len = 0;

  Eigen::Matrix<float, 6, 1> joint_dtheta_holder;

  for (int joint=0; joint < 6; joint++){
        joint_dtheta_holder.row(joint) << joint_position_[joint] - joint_position_command_[joint];
  }

  Eigen::Matrix<float, 6, 1> joint_velocity_holder;

 for (int joint=0; joint < 6; joint++){
        joint_velocity_holder.row(joint) << joint_velocity_command_[joint];
  }
for (int stage=0; stage < 6; stage++){
float min_travel_time = 10000000;
for (int joint=0; joint < 6; joint++){
if (joint_dtheta_holder.coeff(joint,0) != 0 && joint_velocity_holder.coeff(joint,0) != 0){
float travel_time = joint_dtheta_holder.coeff(joint,0) / joint_velocity_holder.coeff(joint, 0);
if (travel_time < min_travel_time) min_travel_time = travel_time;
} else {joint_velocity_holder.row(joint).col(0) << 0;}
}
if(min_travel_time == 10000000)break;
// Breaks Here
Eigen::Matrix<float, 6, 1> steps_per_second = degrees_per_step_inv * joint_velocity_holder;
Eigen::Matrix<float, 6, 1> steps_to_take = steps_per_second * min_travel_time;
Eigen::Matrix<float, 6, 1> sub_delta_theta = steps_to_take.transpose() * degrees_per_step;

joint_dtheta_holder -= sub_delta_theta;
// Storr Results
for (int i = 0 ; i < 6; i++){
step_cmd_holder[cmd_len][0][i].val = steps_to_take.coeff(i, 0);
step_cmd_holder[cmd_len][1][i].val = steps_per_second.coeff(i, 0);
}
cmd_len += 1;
}
// std::cout << "solution x=\n" << x << "\n\nresidual A*x-B=\n" << A * x - B << '\n';
 char msg22[] = {'1'};
  ::write(serial_port, msg22, sizeof(msg22));

  for (int stage = 0; stage < cmd_len; stage++) {
  for (int mode = 0; mode < 2; mode++) {
  for (int joint = 0; joint < 6; joint++) {
    ::write(serial_port, step_cmd_holder[stage][mode][joint].array, 4);
  }}}
}
} // namespace merlin_hardware_interface
