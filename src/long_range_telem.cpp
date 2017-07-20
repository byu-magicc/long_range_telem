#include "long_range_telem/long_range_telem.h"

namespace long_range_telem
{

LRTelem::LRTelem() :
  nh_(), nh_private_("~")
{
  command_sub_ = nh_.subscribe("command", 1, &LRTelem::commandCallback, this);

  // Connect to MAVlink
  std::string port = nh_private_.param<std::string>("port", "/dev/ttyUSB0");
  int baud_rate = nh_private_.param<int>("baud_rate", 921600);
  ROS_INFO("Connecting to serial port \"%s\", at %d baud", port.c_str(), baud_rate);
  mavlink_comm_ = new mavrosflight::MavlinkSerial(port, baud_rate);

  try
  {
    mavlink_comm_->open(); //! \todo move this into the MavROSflight constructor
    mavrosflight_ = new mavrosflight::MavROSflight(*mavlink_comm_);
  }
  catch (mavrosflight::SerialException e)
  {
    ROS_FATAL("%s", e.what());
    ros::shutdown();
  }

  mavrosflight_->comm.register_mavlink_listener(this);
}

LRTelem::~LRTelem()
{
  delete mavrosflight_;
  delete mavlink_comm_;
}

void LRTelem::handle_mavlink_message(const mavlink_message_t &msg)
{
  switch (msg.msgid)
  {
    case MAVLINK_MSG_ID_HEARTBEAT:
      handle_heartbeat_msg(msg);
      break;
    case MAVLINK_MSG_ID_OFFBOARD_CONTROL:
      handle_offboard_command_msg(msg);
      break;
  }
}

void LRTelem::handle_heartbeat_msg(const mavlink_message_t &msg)
{
  ROS_INFO_ONCE("Got HEARTBEAT, connected.");
}

void LRTelem::handle_offboard_command_msg(const mavlink_message_t &msg)
{

}

void LRTelem::commandCallback(rosflight_msgs::Command::ConstPtr msg)
{

}

}
