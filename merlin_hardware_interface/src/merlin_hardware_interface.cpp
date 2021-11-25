#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <merlin_hardware_interface/merlin_hardware_interface.h>

#include <unistd.h> // write(), read(), close()
#include <stdio.h>
#include <stdlib.h>

#include <fcntl.h>
#include <termios.h>

#include <cstddef> // for NULL

constexpr const char *const SERIAL_PORT_1 = "/dev/ttyACM0";
const int BUFFER_SIZE = 4;

union open_float
{
  float val = 0;
  char array[4];
};

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
    ros::Duration update_freq = ros::Duration(1.0 / loop_hz_);
    non_realtime_loop_ =
        nh_.createTimer(update_freq, &MerlinHardwareInterface::update, this);
  }

  MerlinHardwareInterface::~MerlinHardwareInterface() {}
  void MerlinHardwareInterface::init_serial()
  {

    /* Open the file descriptor in non-blocking mode */
    serial_port = open("/dev/ttyACM0", O_RDWR | O_NOCTTY);

    // Check for errors
    if (serial_port < 0)
    {
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
  void MerlinHardwareInterface::init()
  {
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
    joint_effort_command_.resize(num_joints_);

    // Initialize Controller
    for (int i = 0; i < num_joints_; ++i)
    {
      // Create joint state interface
      JointStateHandle jointStateHandle(joint_names_[i], &joint_position_[i],
                                        &joint_velocity_[i], NULL);
      joint_state_interface_.registerHandle(jointStateHandle);

      // Create position joint interface
      JointHandle jointPositionHandle(jointStateHandle,
                                      &joint_position_command_[i]);
      JointLimits limits;
      SoftJointLimits softLimits;
      getJointLimits(joint_names_[i], nh_, limits);
      PositionJointSoftLimitsHandle jointLimitsHandle(jointPositionHandle,
                                                      limits, softLimits);
      positionJointSoftLimitsInterface.registerHandle(jointLimitsHandle);
      position_joint_interface_.registerHandle(jointPositionHandle);
    };

    registerInterface(&joint_state_interface_);
    registerInterface(&position_joint_interface_);
    registerInterface(&positionJointSoftLimitsInterface);
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
    char msg[] = {'R'};
    ::write(serial_port, msg, sizeof(msg));

    for (int i = 0; i < num_joints_; i++)
    {
      open_float temp;
      // Read bytes. The behaviour of read() (e.g. does it block?,
      // how long does it block for?) depends on the configuration
      // settings above, specifically VMIN and VTIME
      int n = ::read(serial_port, &temp.array, 4);
      if (n <= 0)
      {
      }
      joint_position_[i] = (double)temp.val;
    }
  }

  void MerlinHardwareInterface::write(ros::Duration elapsed_time)
  {
    //positionJointSoftLimitsInterface.enforceLimits(elapsed_time);
    char msg[] = {'W'};
    ::write(serial_port, msg, sizeof(msg));

    open_float conv;

//     #include <iostream>
// #include <Eigen/Dense>
 
// using namespace std;
// using namespace Eigen;
 
// int main()
// {
//    Matrix3f A;
//    Vector3f b;
//    A << 1,2,3,  4,5,6,  7,8,10;
//    b << 3, 3, 4;
//    cout << "Here is the matrix A:\n" << A << endl;
//    cout << "Here is the vector b:\n" << b << endl;
//    Vector3f x = A.colPivHouseholderQr().solve(b);
//    cout << "The solution is:\n" << x << endl;
// }

// #include <Eigen/LU>
// #include <iostream>

// int main() {
//   Eigen::Matrix3d A;
//   A << 64, 256, 1024, 48, 256, 1280, 24, 192, 1280;
//   Eigen::Vector3d B;
//   B << -9.0, 0, 0;
//   Eigen::Matrix3d A_inv = A.inverse();
//   Eigen::Vector3d x = A_inv * B;

//   std::cout << "solution x=\n" << x << "\n\nresidual A*x-B=\n" << A * x - B << '\n';
// }
    for (int i = 0; i < num_joints_; i++)
    {
      conv.val = (float)joint_position_command_[i];
      ::write(serial_port, conv.array, 4);
    }
  }
} // namespace merlin_hardware_interface
