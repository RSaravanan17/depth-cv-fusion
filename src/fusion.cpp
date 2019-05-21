#include "fri_stickler/fusion.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <ros/ros.h>
#include <vector>

FusionProcessor::FusionProcessor(int width, int height, cv::Mat base_image) :
    extrema_clusters(1, 0.25) {
  image_width = width;
  image_height = height;
  this->base_image = base_image;
}

int FusionProcessor::find_extreme(std::vector<float> &dat, int partitions,
    int thoroughness, float minimum_extreme) {
  int w = dat.size() / 2 / partitions, lower = 0, upper = dat.size() / 2;
  bool valid_extreme = false;

  // Levels of thoroughness
  for (int i = 0; i < thoroughness && w > 0; i++) {
    float record_high = std::numeric_limits<float>::min();
    int record_high_pos = -1;
    bool found_min_extreme = false;

    // Evaluate slopes of each partition
    for (int pos = lower; pos < upper; pos += w) {
      int pos_end = pos + w > upper ? upper - 1 : pos + w;
      float pos_d = std::isnan(dat[pos]) ? 0 : dat[pos];
      float pos_end_d = std::isnan(dat[pos_end]) ? 0 : dat[pos_end];
      float slope = fabs(pos_end_d - pos_d);
      if ((found_min_extreme || slope > FDEXT_MINIMUM_EXTREME) &&
          slope > record_high) {
        dirty_distance_total += std::max(dat[pos], dat[pos_end]);
        dirty_distance_points_processed++;
        record_high = slope;
        record_high_pos = pos;
        found_min_extreme = true;
      }
    }

    // If a valid extreme wasn't found, we go no further
    if (record_high_pos != -1)
      valid_extreme = true;
    else
      break;

    // Record high partition becomes next search range
    lower = record_high_pos;
    upper = lower + w;
    w = (upper - lower) / partitions;
  }

  return valid_extreme ? (upper + lower) / 2 : 0;
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
          int xd = r * x_ratio + X_ALPHA, yd = c * y_ratio + Y_ALPHA;
          depth_map.set_depth(xd, yd, NAN);
        }
    }
}

float FusionProcessor::estimate_distance(DepthMap &depth_map,
    darknet_ros_msgs::BoundingBoxes &boxes, int focus_index) {
  dirty_distance_total = 0;
  dirty_distance_points_processed = 0;
  darknet_ros_msgs::BoundingBox focus_box = boxes.bounding_boxes[focus_index];

  // Images for visualization purposes
  cv::imwrite("base_rgb.png", base_image);
  darknet_ros_msgs::BoundingBox big_box;
  big_box.xmin = big_box.ymin = 0;
  big_box.xmax = depth_map.get_width();
  big_box.ymax = depth_map.get_height();
  cv::imwrite("base_depth.png", depth_map.get_sub_map(big_box, 0, 0));
  cv::imwrite("depth_focus.png", depth_map.get_sub_map(focus_box,
      X_ALPHA, Y_ALPHA));

  // STEP 0 - Remove bounding box intersections
  if (ALGO_STEPS >= ALGO_STEP_KNOCKOUT)
    intersection_knockout(depth_map, boxes, focus_index);

  cv::imwrite("step0.png", depth_map.get_sub_map(focus_box, X_ALPHA, Y_ALPHA));

  // Helpful for checking the accuracy of the algorithm's answer
  int mx = (focus_box.xmin + focus_box.xmax) / 2;
  int my = (focus_box.ymin + focus_box.ymax) / 2;
  float mid_depth = depth_map.get_depth(mx + X_ALPHA, my + Y_ALPHA);
  ROS_INFO("[estimate_distance] Middlemost focus box position has depth %f m",
      mid_depth);

  // STEP 1 - Identify "extreme" values in the depth map and cluster them
  std::vector<float> x_fdext, y_fdext;

  // Remove extreme vertical ranges
  if (ALGO_STEPS >= ALGO_STEP_EXTREMA) {
    for (int i = focus_box.xmin; i < focus_box.xmax; i++) {
      // Dump column data into a vector
      std::vector<float> col;
      for (int j = focus_box.ymin; j < focus_box.ymax; j++)
        col.push_back(depth_map.get_depth(i + X_ALPHA, j + Y_ALPHA));

      // Eliminate lower extreme
      int p = find_extreme(col, FDEXT_PARTITIONS, FDEXT_THOROUGHNESS,
          FDEXT_MINIMUM_EXTREME);
      for (int j = 0; j <= p; j++) {
        x_fdext.push_back(i + X_ALPHA);
        y_fdext.push_back(focus_box.ymin + j + Y_ALPHA);
      }

      // Eliminate upper extreme
      std::reverse(col.begin(), col.end());

      p = find_extreme(col, FDEXT_PARTITIONS, FDEXT_THOROUGHNESS,
          FDEXT_MINIMUM_EXTREME);
      for (int j = 0; j <= p; j++) {
        x_fdext.push_back(i + X_ALPHA);
        y_fdext.push_back(focus_box.ymax - j + Y_ALPHA);
      }
    }

    // Remove extreme horizontal ranges
    for (int i = focus_box.ymin; i < focus_box.ymax; i++) {
      // Dump row data into a vector
      std::vector<float> row;
      for (int j = focus_box.xmin; j < focus_box.xmax; j++)
        row.push_back(depth_map.get_depth(j + X_ALPHA, i + Y_ALPHA));

      // Eliminate lower extreme
      int p = find_extreme(row, FDEXT_PARTITIONS, FDEXT_THOROUGHNESS,
          FDEXT_MINIMUM_EXTREME);
      for (int j = 0; j <= p; j++) {
        x_fdext.push_back(focus_box.xmin + j + X_ALPHA);
        y_fdext.push_back(i + Y_ALPHA);
      }

      // Eliminate upper extreme
      std::reverse(row.begin(), row.end());

      p = find_extreme(row, FDEXT_PARTITIONS, FDEXT_THOROUGHNESS,
          FDEXT_MINIMUM_EXTREME);
      for (int j = 0; j <= p; j++) {
        x_fdext.push_back(focus_box.xmax - j + X_ALPHA);
        y_fdext.push_back(i + Y_ALPHA);
      }
    }

    for (int i = 0; i < x_fdext.size(); i++) {
      float depth = depth_map.get_depth(x_fdext[i], y_fdext[i]);
      if (!std::isnan(depth)) {
        extrema_clusters.add_point(depth, false);
        depth_map.set_depth(x_fdext[i], y_fdext[i], NAN);
      }
    }
  }

  extrema_clusters.summarize(false);

  // Remove extreme points
  for (int i = 0; i < x_fdext.size(); i++)
    depth_map.set_depth(x_fdext[i] + X_ALPHA, y_fdext[i] + Y_ALPHA, NAN);

  cv::imwrite("step1.png", depth_map.get_sub_map(focus_box, X_ALPHA, Y_ALPHA));

  // STEP 2 - Identify the largest extrema cluster and remove remaining points
  // that could logically fit into it
  if (ALGO_STEPS >= ALGO_STEP_CLUSTER && extrema_clusters.num_clusters() > 1) {
    PointCluster *dirty_cluster = extrema_clusters.get_largest_cluster();

    if (dirty_cluster != nullptr) {
      for (int r = focus_box.xmin; r <= focus_box.xmax; r++)
        for (int c = focus_box.ymin; c <= focus_box.ymax; c++) {
          float depth = depth_map.get_depth(r + X_ALPHA, c + Y_ALPHA);
          if (!std::isnan(extrema_clusters.rate_fit(*dirty_cluster, depth)))
            depth_map.set_depth(r + X_ALPHA, c + Y_ALPHA, NAN);
        }
    }
  }

  // Assemble a set of points that will be averaged for the final answer
  std::vector<float> point_set;
  for (int r = focus_box.xmin; r <= focus_box.xmax; r++)
    for (int c = focus_box.ymin; c <= focus_box.ymax; c++) {
      float depth = depth_map.get_depth(r + X_ALPHA, c + Y_ALPHA);
      if (!std::isnan(depth))
        point_set.push_back(depth);
    }

  cv::imwrite("step2.png", depth_map.get_sub_map(focus_box, X_ALPHA, Y_ALPHA));

  std::sort(point_set.begin(), point_set.end());

  // STEP 3 - Destroy fringe data
  if (ALGO_STEPS >= ALGO_STEP_FRINGE) {
    int fringe_width = point_set.size() * ((1 - INTERQUARTILE) / 2);

    if (fringe_width > 0) {
      float lower_fringe = point_set[fringe_width];
      float upper_fringe = point_set[point_set.size() - fringe_width];

      for (int r = focus_box.xmin; r <= focus_box.xmax; r++)
        for (int c = focus_box.ymin; c <= focus_box.ymax; c++) {
          float depth = depth_map.get_depth(r + X_ALPHA, c + Y_ALPHA);
          if (depth <= lower_fringe || depth >= upper_fringe)
            depth_map.set_depth(r + X_ALPHA, c + Y_ALPHA, NAN);
        }
    }

    for (int i = 0; i < point_set.size() * (1 - INTERQUARTILE); i++) {
      point_set.erase(point_set.begin());
      point_set.erase(point_set.end() - 1);
    }
  }

  cv::imwrite("step3.png", depth_map.get_sub_map(focus_box, X_ALPHA, Y_ALPHA));

  // Average trimmed point set for the final estimate
  float total_depth = 0;

 for (int i = 0; i < point_set.size(); i++)
  total_depth += point_set[i];

  return total_depth / point_set.size();
}
