#include <ros/ros.h>

#include <rosflight/mavrosflight/mavrosflight.h>
#include <rosflight/mavrosflight/mavlink_comm.h>
#include <rosflight/mavrosflight/mavlink_listener_interface.h>
#include <rosflight/mavrosflight/param_listener_interface.h>
#include <rosflight/mavrosflight/serial_exception.h>
#include <rosflight/mavrosflight/mavlink_serial.h>

#include <rosflight_msgs/Command.h>

namespace long_range_telem
{

class LRTelem:
  public mavrosflight::MavlinkListenerInterface
{
public:
  LRTelem();
  ~LRTelem();
  void handle_mavlink_message(const mavlink_message_t &msg);

private:
  // Mavrosflight objects
  mavrosflight::MavlinkComm *mavlink_comm_;
  mavrosflight::MavROSflight *mavrosflight_;

  ros::Timer heartbeat_timer_;
  ros::NodeHandle nh_, nh_private_;

  // Mavlink -> ROS functions
  void handle_offboard_command_msg(const mavlink_message_t &msg);
  void handle_heartbeat_msg(const mavlink_message_t &msg);
  ros::Publisher command_pub_;

  // ROS -> Mavlink functions
  void commandCallback(rosflight_msgs::Command::ConstPtr msg);
  ros::Subscriber command_sub_;

};

}
