#include "fri_stickler/depth_map.h"

#include <math.h>

void DepthMap::process_image(const sensor_msgs::Image::ConstPtr &msg) {
  cv_bridge::CvImageConstPtr depth_img_cv;
  depth_img_cv = cv_bridge::toCvCopy(msg,
      sensor_msgs::image_encodings::TYPE_32FC1);
  this->depth_map = depth_img_cv->image;
}

float DepthMap::get_depth(int x, int y) {
  if (x < 0 || x > get_width() || y < 0 || y > get_height())
    return NAN;

  return depth_map.at<float>(cv::Point(x, y));
}

void DepthMap::set_depth(int x, int y, float depth) {
  if (x < 0 || x > get_width() || y < 0 || y > get_height())
    return;

  depth_map.at<float>(y, x, 0) = depth;
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

cv::Mat* DepthMap::get_mat() {
  return &depth_map;
}

cv::Mat DepthMap::get_sub_map(darknet_ros_msgs::BoundingBox box,
    const float X_ALPHA, const float Y_ALPHA) {
  const float DEPTH_MAX = 5;
  const float DEPTH_MIN = 0;
  cv::Mat sub(box.ymax - box.ymin, box.xmax - box.xmin, CV_8UC1);

  for (int x = 0; x < sub.cols; x++)
    for (int y = 0; y < sub.rows; y++) {
      float depth = get_depth(x + box.xmin + X_ALPHA, y + box.ymin + Y_ALPHA);
      uint8_t gray = std::isnan(depth) ? 0 : (255 - (depth / DEPTH_MAX) * 255);
      sub.at<uint8_t>(y, x, 0) = gray;
    }

  return sub;
}
