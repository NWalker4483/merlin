#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <merlin_hardware_interface/merlin_hardware_interface.h>
// #include <sstream>

constexpr const char *const SERIAL_PORT_1 = "/dev/ttyACM0";
const int BUFFER_SIZE = 4;

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
    serial_port = open("/dev/ttyACM0", O_RDWR);

    // Check for errors
    if (serial_port < 0)
    {
      printf("Error %i from open: %s\n", errno, strerror(errno));
    }

    // Create new termios struct, we call it 'tty' for convention
    // No need for "= {0}" at the end as we'll immediately write the existing
    // config to this struct
    struct termios tty;

    // Read in existing settings, and handle any error
    // NOTE: This is important! POSIX states that the struct passed to tcsetattr()
    // must have been initialized with a call to tcgetattr() overwise behaviour
    // is undefined
    if (tcgetattr(serial_port, &tty) != 0)
    {
      printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
    }

    tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)

    tty.c_cflag &= ~CSTOPB;  // Clear stop field, only one stop bit used in communication (most common)
    tty.c_cflag &= ~CSIZE;   // Clear all the size bits, then use one of the statements below
    tty.c_cflag |= CS8;      // 8 bits per byte (most common)
    tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)

    tty.c_lflag &= ~ICANON; // Disable canonical mode

    tty.c_lflag &= ~ECHO;   // Disable echo
    tty.c_lflag &= ~ECHOE;  // Disable erasure
    tty.c_lflag &= ~ECHONL; // Disable new-line echo

    tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP

    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl

    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL); // Disable any special handling of received bytes

    tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
    tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed

    tty.c_cc[VTIME] = 10; // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
    tty.c_cc[VMIN] = 0;

    cfsetispeed(&tty, B115200);
    cfsetospeed(&tty, B115200);

    // Save tty settings, also checking for error
    if (tcsetattr(serial_port, TCSANOW, &tty) != 0)
    {
      printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
    }
  }
  void MerlinHardwareInterface::init()
  {
    // Instantiate SerialPort object.
    init_serial()
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
                                        &joint_velocity_[i], &joint_effort_[i]);
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
    }

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
    unsigned char msg[] = {'R'};
    write(serial_port, msg, sizeof(msg));

    for (int i = 0; i < num_joints_; i++)
    {
      // Allocate memory for read buffer, set size according to your needs
      char read_buf[4];
      float temp;

      // Read bytes. The behaviour of read() (e.g. does it block?,
      // how long does it block for?) depends on the configuration
      // settings above, specifically VMIN and VTIME
      int n = read(serial_port, &read_buf, sizeof(read_buf));
      memcpy(&temp, &read_buf, sizeof(temp));
      joint_position_[i] = (double)temp;
    }

    void MerlinHardwareInterface::write(ros::Duration elapsed_time)
    {
      positionJointSoftLimitsInterface.enforceLimits(elapsed_time);
      unsigned char msg[] = {'W'};
      write(serial_port, msg, sizeof(msg));

      union
      {
        float val;
        char array[4];
      } conv;
      for (int i = 0; i < num_joints_; i++)
      {
        conv.val = (float)joint_position_command_[i];
        write(serial_port, conv.array, 4);
      }
    }
  } // namespace merlin_hardware_interface
