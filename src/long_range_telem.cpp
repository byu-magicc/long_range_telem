#include "long_range_telem/long_range_telem.h"

namespace long_range_telem
{

LRTelem::LRTelem() :
  nh_(), nh_private_("~")
{
  command_sub_ = nh_.subscribe("command", 1, &LRTelem::commandCallback, this);

  heartbeat_timer_ = nh_.createTimer(ros::Duration(1.0), &LRTelem::heartbeatTimerCallback, this);

  // Connect to MAVlink
  std::string port = nh_private_.param<std::string>("port", "/dev/ttyUSB0");
  int baud_rate = nh_private_.param<int>("baud_rate", 921600);

  system_id_ = nh_private_.param<int>("system_id", 2);
  component_id_ = nh_private_.param<int>("component_id", 150);
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
  last_heartbeat_received_ = ros::Time::now();
}

void LRTelem::handle_offboard_command_msg(const mavlink_message_t &msg)
{
  mavlink_offboard_control_t control_msg;
  mavlink_msg_offboard_control_decode(&msg, &control_msg);
  rosflight_msgs::Command out_msg;
  out_msg.header.stamp = ros::Time::now();
  out_msg.ignore = control_msg.ignore;
  out_msg.mode = control_msg.mode;
  out_msg.x = control_msg.x;
  out_msg.y = control_msg.y;
  out_msg.z = control_msg.z;
  out_msg.F = control_msg.F;

  if (command_pub_.getTopic().empty())
  {
    command_pub_ = nh_.advertise<rosflight_msgs::Command>("command", 1);
  }
  command_pub_.publish(out_msg);
}

void LRTelem::commandCallback(rosflight_msgs::Command::ConstPtr msg)
{
  mavlink_message_t mavlink_msg;
  mavlink_msg_offboard_control_pack(system_id_, component_id_, &mavlink_msg,
                                    msg->mode, msg->ignore, msg->x, msg->y, msg->z, msg->F);
  mavrosflight_->comm.send_message(mavlink_msg);
}

void LRTelem::heartbeatTimerCallback(const ros::TimerEvent &e)
{
  // send a heartbeat
  mavlink_message_t msg;
  mavlink_msg_heartbeat_pack(system_id_, component_id_, &msg, 0, 0, 0, 0, 0);
  mavrosflight_->comm.send_message(msg);

  // check to make sure we are still connected to the other side
  ros::Time now = ros::Time::now();
  if (now - last_heartbeat_received_ > ros::Duration(2.0))
  {
    ROS_ERROR("[long_range_telemetry - %d,%d]lost connection", system_id_, component_id_);
  }
}
}
