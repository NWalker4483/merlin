#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <ros/console.h>
#include <merlin_hardware_interface/merlin_hardware_interface.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h> // write(), read(), close()

#include <fcntl.h>
#include <termios.h>

#include <cstddef> // for NULL
const int BUFFER_SIZE = 4;

union open_float {
  float val = 0;
  char array[4];
};

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
  // TODO: Move reductions to yaml
  motor_reductions << 
  1. / 48., 0, 0, 0, 0, 0,
  0, 1. / 48., 1. / 48., 0, 0, 0,
  0, 0, 1. / 48., 0, 0, 0,
  0, 0, 0, 1. / 24., -1. / 28.8, -1. / 12.,
  0, 0, 0, 0, 1. / 28.8, 1. / 24., 
  0, 0, 0, 0, 0, 1. / 24.;

  motor_spr << 200, 200, 200, 200, 200, 200;
  radians_per_step = (6.283 / 200.) * motor_reductions;

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

    // connect and register the joint position interface
    JointHandle velocity_joint_handle(jointStateHandle, &joint_velocity_command_[i]);

    velocity_joint_interface_.registerHandle(velocity_joint_handle);

    // TODO: Enforce joint limits
    JointLimits limits;
    getJointLimits(joint_names_[i], nh_, limits);
  };

  registerInterface(&joint_state_interface_);
  registerInterface(&velocity_joint_interface_);
}

void MerlinHardwareInterface::update(const ros::TimerEvent &e) {
  elapsed_time_ = ros::Duration(e.current_real - e.last_real);
  read();
  controller_manager_->update(ros::Time::now(), elapsed_time_);
  write(elapsed_time_);
}

void MerlinHardwareInterface::read() {
  float ppr = 200.; // NOTE: Only use for when using stepper postion from accelstepper
  Eigen::Matrix<float, 6, 1> index_cnt;
  Eigen::Matrix<float, 6, 1> pulse_cnt;
  Eigen::Matrix<float, 6, 1> rev_cnt;

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
    ROS_INFO("Motor %i: ", i);
    ROS_INFO("%d pulses\n",(index.val * 200 ) + pulses.val);
    index_cnt.row(i).col(0) << index.val;
    pulse_cnt.row(i).col(0) << pulses.val;
  }
  // Revs to Pulses
    index_cnt *= ppr;
    // Total Pulses
    pulse_cnt += index_cnt;
    // Total Revs
    rev_cnt = pulse_cnt / ppr;
    // Total Joint Revs
    rev_cnt = motor_reductions * rev_cnt;
    // Total Joint Travel from Zero Position in Radians 
    rev_cnt *= 6.283; //(6.283 / 200.)
    
 for (int i = 0; i<6;i++){
    // ROS_INFO("Motor %d: ",i);
    // ROS_INFO("%d pulses\n",pulse_cnt.coeff(i,0));
   joint_position_.at(i) = (double)rev_cnt.coeff(i,0);
}
}

void MerlinHardwareInterface::write(ros::Duration elapsed_time) {
  char msg[] = {'W'};
  ::write(serial_port, msg, sizeof(msg));

  // TODO: init with pointer to joint_velocity_command_.data()
  Eigen::Matrix<float, 6, 1> joint_velocity_holder;

  for (int joint = 0; joint < 6; joint++) {
    joint_velocity_holder.row(joint) << joint_velocity_command_.at(joint);
  }

  Eigen::Matrix<float, 6, 1> steps_per_second =
      radians_per_step.transpose().inverse() * joint_velocity_holder;
      
    open_float temp;
    // ROS_INFO("Start Write\n");
    for (int i = 0; i < 6; i++) {
      temp.val = steps_per_second.coeff(i, 0);
    // ROS_INFO("%f\n", temp.val);
      ::write(serial_port, temp.array, 4); 
    }
     //ROS_INFO("End Write\n");
  }

} // namespace merlin_hardware_interface
