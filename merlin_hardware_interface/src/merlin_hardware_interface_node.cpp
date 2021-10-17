#include <merlin_hardware_interface/merlin_hardware_interface.h>
#include <ros/callback_queue.h>
int main(int argc, char **argv)
{
  ros::init(argc, argv, "merlin_hardware_interface");
  ros::CallbackQueue ros_queue;

  ros::NodeHandle nh;
  nh.setCallbackQueue(&ros_queue);
  merlin_hardware_interface::MerlinHardwareInterface rhi(nh);

  ros::MultiThreadedSpinner spinner(0);
  spinner.spin(&ros_queue);
  return 0;
}
