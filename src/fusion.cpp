#include "fri_stickler/fusion.h"

#include <ros/ros.h>

FusionProcessor::FusionProcessor(int width, int height) {
  image_width = width;
  image_height = height;
}

void FusionProcessor::intersection_knockout(DepthMap &depth_map,
    darknet_ros_msgs::BoundingBoxes &boxes, std::string focus_label) {
  // Compute conversion from RGB image to depth image dimensions
  float x_ratio = depth_map.get_width() / image_width;
  float y_ratio = depth_map.get_height() / image_height;

  // For each bounding box that is not our focal object
  for (int i = 0; i < boxes.bounding_boxes.size(); i++) {
    darknet_ros_msgs::BoundingBox box = boxes.bounding_boxes[i];
    if (box.Class != focus_label) {
      std::string box_label = box.Class;
      ROS_INFO("[intersection_knockout] Knocking out \"%s\" (%d pixels)...",
          box_label, (box.xmax - box.xmin) * (box.ymax - box.ymin));
      // Replace depth data within that box with NAN
      for (int r = box.xmin; r <= box.xmax; r++)
        for (int c = box.ymin; c <= box.ymax; c++) {
          float xd = r * x_ratio, yd = c * y_ratio;
          depth_map.set_depth(xd, yd, NAN);
        }
    }
  }
}
