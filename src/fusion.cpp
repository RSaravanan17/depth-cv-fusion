#include "fri_stickler/fusion.h"

#include <cmath>
#include <ros/ros.h>

FusionProcessor::FusionProcessor(int width, int height) {
  image_width = width;
  image_height = height;
}

void FusionProcessor::intersection_knockout(DepthMap &depth_map,
    darknet_ros_msgs::BoundingBoxes &boxes, int focus_index) {
  // Compute conversion from RGB image to depth image dimensions
  float x_ratio = depth_map.get_width() / image_width;
  float y_ratio = depth_map.get_height() / image_height;

  // For each bounding box that is not our focal object
  for (int i = 0; i < boxes.bounding_boxes.size(); i++)
    if (i != focus_index) {
      darknet_ros_msgs::BoundingBox box = boxes.bounding_boxes[i];
      ROS_INFO("[intersection_knockout] Knocking out \"%s\" (%d pixels)...",
          box.Class.c_str(), (box.xmax - box.xmin) * (box.ymax - box.ymin));
      // Replace depth data within that box with NAN
      for (int r = box.xmin; r <= box.xmax; r++)
        for (int c = box.ymin; c <= box.ymax; c++) {
          float xd = r * x_ratio, yd = c * y_ratio;
          depth_map.set_depth(xd, yd, NAN);
        }
    }
}

float FusionProcessor::estimate_distance(DepthMap &depth_map,
    darknet_ros_msgs::BoundingBoxes &boxes, int focus_index) {
  // Remove bounding box intersections
  intersection_knockout(depth_map, boxes, focus_index);

  darknet_ros_msgs::BoundingBox focus_box = boxes.bounding_boxes[focus_index];

  // Average all remaining depth data
  float total_depth = 0;
  int points_processed = 0;

  for (int r = focus_box.xmin; r <= focus_box.xmax; r++)
    for (int c = focus_box.ymin; c <= focus_box.ymax; c++) {
      float depth = depth_map.get_depth(r, c);
      if (!std::isnan(depth)) {
        total_depth += depth;
        points_processed++;
      }
    }

  return total_depth / points_processed;
}
