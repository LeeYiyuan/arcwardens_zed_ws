#include <ros/ros.h>
#include <sensor_msgs/Image.h>

void imageRightRectifiedCallback(const sensor_msgs::Image::ConstPtr& msg) {
  ROS_INFO("Right Rectified image received from ZED - Size: %dx%d", msg->width, msg->height);
}

void imageLeftRectifiedCallback(const sensor_msgs::Image::ConstPtr& msg) {
  ROS_INFO("Left Rectified image received from ZED - Size: %dx%d", msg->width, msg->height);
}
int main(int argc, char** argv) {
  ros::init(argc, argv, "zed_video_subscriber");

  ros::NodeHandle n;

  ros::Subscriber subRightRectified = n.subscribe("/zed/right/image_rect_color", 10, imageRightRectifiedCallback);
  ros::Subscriber subLeftRectified  = n.subscribe("/zed/left/image_rect_color", 10, imageLeftRectifiedCallback);

  ros::spin();

  return 0;
}
