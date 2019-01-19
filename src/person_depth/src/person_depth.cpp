#include <ros/ros.h>
#include "iostream"
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/image_encodings.h"
#include "sensor_msgs/Image.h"
#include "darknet_ros_msgs/BoundingBoxes.h"
#include "darknet_ros_msgs/BoundingBox.h"
#include "std_msgs/Int64.h"
#include "person_depth/person.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using namespace std;
namespace enc = sensor_msgs::image_encodings;
int xMin, xMax, yMin, yMax;
ros::Publisher pub;

void person_location(const darknet_ros_msgs::BoundingBoxes::ConstPtr &detected_people) {
    //detected_people is the array of bounding boxes
  int peopleNum = sizeof(detected_people->bounding_boxes);
  
  ::xMin = detected_people->bounding_boxes[0].xmin;
  ::yMin = detected_people->bounding_boxes[0].ymin;
  ::xMax = detected_people->bounding_boxes[0].xmax;
  ::yMax = detected_people->bounding_boxes[0].ymax;
}

void depth_callback(const sensor_msgs::Image::ConstPtr& img) {
/*  cv_bridge::CvImagePtr cv_ptr;

  try { 
    cv_ptr = cv_bridge::toCvCopy(img, enc::TYPE_16UC1);
  } catch (cv_bridge::Exception e) {
      ROS_ERROR("%s", e.what()); 
      return;
  }

  cv::Point center;
*/

  float* depths = (float*)(&img->data[0]);

  int x = (xMin + xMax) / 2;
  int y = (yMin + yMax) / 2;

  int centerIdx = x + img->width * y;
  float depth = depths[centerIdx];

  ros::NodeHandle ns;
  person_depth::person person;
  
  person.x = x;
  person.y = y;
  person.image_width = img->width;
  person.z = depth;
  pub.publish(person);

  ROS_INFO("Distance of person: %g", depths[centerIdx]);

}

int main(int argc, char **argv) {
    ros::init(argc, argv, "depth");
    ros::NodeHandle nh;
    pub = nh.advertise<person_depth::person>("/person", 1);

    ros::Subscriber sub = nh.subscribe("/darknet_ros/bounding_boxes", 100, person_location);
    
  //  image_transport::ImageTransport it(nh);
    ros::Subscriber image_sub = nh.subscribe("/zed/depth/depth_registered", 1, depth_callback);

    ros::spin();

}
