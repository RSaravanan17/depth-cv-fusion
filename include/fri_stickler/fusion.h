#include "fri_stickler/depth_map.h"

#include <cv_bridge/cv_bridge.h>
#include <darknet_ros_msgs/CheckForObjectsAction.h>
#include <math.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <ros/ros.h>

/**
  Wrapper for sensor fusion processing.
*/
class FusionProcessor {
protected:
  // Arbitrary constants for aligning the IR and RGB images from the Kinect
  const float X_ALPHA = 22; // 27
  const float Y_ALPHA = -30;

  // find_extreme configuration
  const float FDEXT_MINIMUM_EXTREME = 0.1;
  const int FDEXT_PARTITIONS = 8;
  const int FDEXT_THOROUGHNESS = 3;

  // Misc algorithm configuration
  const float DEPTH_WHITE = 5;
  const float DIRTY_SIMILARITY_THRESH = 0.95;
  const float INTERQUARTILE = 0.8;

  int image_width, image_height;
  float dirty_distance_total;
  int dirty_distance_points_processed;

public:
  cv::Mat depth_thresh;

  /**
    Creates a new processor.

    @param width pixel width of processed RGB images
    @param height pixel height of processed RGB images
  */
  FusionProcessor(int width, int height);

  /**
    Estimates the location of greatest slope in a vector of depth data.

    @param dat depth data vector
    @param partitions partition count (increase for accuracy/performance tradeoff)
    @param thoroughness iteration depth of binary search (increase for accuracy/
      performance tradeoff)
    @param minimum_extreme smallest slope magnitude considered extreme
    @return position in vector of extreme, or 0 if not found
  */
  int find_extreme(std::vector<float> &dat, int partitions, int thoroughness,
      float minimum_extreme);

  /**
    Removes rectangular regions of depth data from a DepthMap.

    @param depth_map map to edit
    @param boxes object bounding boxes produced by YOLO
    @param focus_index index in bounding box list of object of interest
  */
  void intersection_knockout(DepthMap &depth_map,
      darknet_ros_msgs::BoundingBoxes &boxes, int focus_label);

  /**
    Estimates the distance between the camera lens and a bounding box. This is
    the pinnacle function of the project.

    @param depth_map depth map of scene
    @param boxes object bounding boxes produced by YOLO
    @param focus_index index in bounding box list of object of interest
  */
  float estimate_distance(DepthMap &depth_map,
      darknet_ros_msgs::BoundingBoxes &boxes, int focus_index);
};
