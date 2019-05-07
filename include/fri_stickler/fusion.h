#include "fri_stickler/depth_map.h"

#include <cv_bridge/cv_bridge.h>
#include <darknet_ros_msgs/CheckForObjectsAction.h>
#include <math.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

/**
  Wrapper for sensor fusion processing.
*/
class FusionProcessor {
protected:
  int image_width, image_height;

public:
  /**
    Creates a new processor.

    @param width pixel width of processed RGB images
    @param height pixel height of processed RGB images
  */
  FusionProcessor(int width, int height);

  /**
    Removes rectangular regions of depth data from a DepthMap.

    @param depth_map map to edit
    @param boxes object bounding boxes produced by YOLO
    @param focus_label label of box not to remove
  */
  void intersection_knockout(DepthMap &depth_map,
      darknet_ros_msgs::BoundingBoxes &boxes, std::string focus_label);
};
