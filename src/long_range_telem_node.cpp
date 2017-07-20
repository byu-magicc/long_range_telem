#include <ros/ros.h>
#include <long_range_telem/long_range_telem.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "rosflight_io");
  long_range_telem::LRTelem long_range_telem;
  ros::spin();
}
