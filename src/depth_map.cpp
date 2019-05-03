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

int DepthMap::get_width() {
  return depth_map.cols;
}

int DepthMap::get_height() {
  return depth_map.rows;
}
