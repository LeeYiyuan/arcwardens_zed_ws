#include <ros/ros.h>
#include <cmath>
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
typedef std::vector<double> histogram_t;
typedef std::vector<histogram_t> histograms_t;

ros::Publisher pub;

bool has_image_ = false;
cv::Mat image_color_;
bool has_target_ = false;
darknet_ros_msgs::BoundingBox target_box_;
histograms_t target_histograms_;

cv::Mat msg_to_cv(const sensor_msgs::Image::ConstPtr& msg) {
  return cv_bridge::toCvCopy(msg, enc::BGR8)->image;
}

histograms_t getHistogram(const darknet_ros_msgs::BoundingBox& box, const cv::Mat& image) {
  histograms_t histograms(3, std::vector<double>(256, 0));

  int count = 1;
  for (int x = box.xmin; x <= box.xmax; x++) {
    for (int y = box.ymin; y <= box.ymax; y++) {
      const cv::Vec3b& pixel = image.at<cv::Vec3b>(cv::Point(x, y));
      for (int c = 0; c < 3; c++) {
        histograms[c][pixel[c]]++;
      }
      count++;
    }
  }

  for (int c = 0; c < 3; c++) {
    for (double& p : histograms[c]) {
      p /= count;
    }
  }

  return histograms;
}

double getSimilarity(const histograms_t& a, const histograms_t& b) {
  std::vector<double> bc(3, 0);
  for (int c = 0; c < 3; c++) {
    for (int v = 0; v < 256; v++) {
      bc[c] += std::sqrt(a[c][v] * b[c][v]);
    }
  }

  return bc[0] + bc[1] + bc[2];
}

void person_location(const darknet_ros_msgs::BoundingBoxes::ConstPtr &detected_people) {
  if (!has_image_) {
    return;
  }
  if (!has_target_) {
    if (detected_people->bounding_boxes.empty()) {
      return;
    }

    for (const darknet_ros_msgs::BoundingBox& box : detected_people->bounding_boxes) {
      if (!has_target_ || box.probability > target_box_.probability) {
        target_box_ = box;
      }
      has_target_ = true;
    }
    target_histograms_ = getHistogram(target_box_, image_color_);
  } else {

    std::vector<histograms_t> histograms_list(detected_people->bounding_boxes.size());
    std::vector<double> probabilities(detected_people->bounding_boxes.size());

    double best_score = -1;
    int best_box_index;

    for (int i = 0; i < detected_people->bounding_boxes.size(); i++) {
      const darknet_ros_msgs::BoundingBox& box = detected_people->bounding_boxes[i];

      histograms_list[i] = getHistogram(box, image_color_);
      probabilities[i] = box.probability;

      if (probabilities[i] < 0.5) {
        continue;
      }

      double score = getSimilarity(target_histograms_, histograms_list[i]);

      if (score > best_score) {
        best_score = score;
        best_box_index = i;
      }
    }
    
    if (best_score >= 0) {
      target_box_ = detected_people->bounding_boxes[best_box_index];
      target_histograms_ = histograms_list[best_box_index];
    }
  }
}

void imageLeftRectifiedCallback(const sensor_msgs::Image::ConstPtr& msg) {
  image_color_ = msg_to_cv(msg);
  has_image_ = true;
}

void depth_callback(const sensor_msgs::Image::ConstPtr& img) {
  if (!has_target_) {
    return;
  }

  float* depths = (float*)(&img->data[0]);

  int x = (target_box_.xmin + target_box_.xmax) / 2;
  int y = (target_box_.ymin + target_box_.ymax) / 2;

  int centerIdx = x + img->width * y;
  float depth = depths[centerIdx];

  ros::NodeHandle ns;
  person_depth::person person;
  
  person.x = x;
  person.y = y;
  person.image_width = img->width;
  person.z = depth;
  pub.publish(person);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "depth");
    ros::NodeHandle nh;
    pub = nh.advertise<person_depth::person>("/person", 1);

    ros::Subscriber sub = nh.subscribe("/darknet_ros/bounding_boxes", 100, person_location);
    ros::Subscriber image_sub = nh.subscribe("/zed/depth/depth_registered", 1, depth_callback);
    ros::Subscriber subLeftRectified  = nh.subscribe("/zed/left/image_rect_color", 1, imageLeftRectifiedCallback);

    ros::spin();

}
