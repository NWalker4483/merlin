#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <merlin_hardware_interface/merlin_hardware_interface.h>
#include <sstream>


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

  void MerlinHardwareInterface::init()
  {
    // Instantiate SerialPort object.

    // try
    // {
    //   // Open the Serial Ports at the desired hardware devices.
    //   serial_port_.Open(SERIAL_PORT_1);
    // }
    // catch (const OpenFailed &)
    // {
    //   std::cerr << "The serial ports did not open correctly." << std::endl;
    //   // return  LibSerial::EXIT_FAILURE ;
    // }
    // // Set the baud rates.
    // serial_port_.SetBaudRate(BaudRate::BAUD_115200);

    // // Set the number of data bits.
    // serial_port_.SetCharacterSize(CharacterSize::CHAR_SIZE_8);

    // // Set the hardware flow control.
    // serial_port_.SetFlowControl(FlowControl::FLOW_CONTROL_NONE);

    // // Set the parity.
    // serial_port_.SetParity(Parity::PARITY_NONE);

    // // Set the number of stop bits.
    // serial_port_.SetStopBits(StopBits::STOP_BITS_1);

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
    // // Write a single byte of data to the serial ports.
    // serial_port_.WriteByte((char)'R');

    // // Wait until the data has actually been transmitted.
    // serial_port_.DrainWriteBuffer();

    // for (int i = 0; i < num_joints_; i++)
    // {
    //   // Specify a timeout value (in milliseconds).
    //   size_t timeout_milliseconds = 500; // Testing

    //   // Read a whole array of data from the serial port.
      
    //   DataBuffer input_buffer; // [BUFFER_SIZE];
    //   float temp;
    //   char t2;
    //   try
    //   {
    //     // Read the appropriate number of bytes from each serial port.
    //     serial_port_.ReadByte(t2, 500);
    //     std::cout << t2<< std::endl;
    //     //serial_port_.Read(input_buffer, 1, timeout_milliseconds);
    //     //memcpy(&temp, &input_buffer, sizeof(temp));
    //     //joint_position_[i] = 0;(double) temp;
    //   }
    //   catch (const ReadTimeout &)
    //   {
    //     std::cerr << "The Read() call has timed out." << std::endl;
    //   }
    // }
  }

  void MerlinHardwareInterface::write(ros::Duration elapsed_time)
  {
  //   positionJointSoftLimitsInterface.enforceLimits(elapsed_time);
  //   // Write a single byte of data to the serial ports.
  //   serial_port_.WriteByte('W');

  //   // Wait until the data has actually been transmitted.
  //   serial_port_.DrainWriteBuffer();
  //   union {
  //     float val;
  //     char array[BUFFER_SIZE];
  //   } conv;
  //   for (int i = 0; i < num_joints_; i++)
  //   {
  //     conv.val = (float)joint_position_command_[i];
  //     // serial_port_.Write( conv.array, BUFFER_SIZE );
  //   }
  }
} // namespace merlin_hardware_interface
