#include "fri_stickler/depth_map.h"

void DepthMap::process_image(const sensor_msgs::Image::ConstPtr &msg) {
  cv_bridge::CvImageConstPtr depth_img_cv;
  depth_img_cv = cv_bridge::toCvCopy(msg,
      sensor_msgs::image_encodings::TYPE_32FC1);
  this->depth_map = depth_img_cv->image;
}

float DepthMap::get_depth(int x, int y) {
  return depth_map.at<float>(cv::Point(x, y));
}

void DepthMap::set_depth(int x, int y, float depth) {
  depth_map.at<float>(y, x, 0) = depth; // Not sure why y, x
}

int DepthMap::get_width() {
  return depth_map.cols;
}

int DepthMap::get_height() {
  return depth_map.rows;
}

sensor_msgs::Image DepthMap::get_image_msg() {
  cv_bridge::CvImage msg;
  msg.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
  msg.image = depth_map;

  return *msg.toImageMsg();
}
