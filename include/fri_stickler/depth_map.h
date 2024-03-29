#ifndef DEPTH_MAP_H
#define DEPTH_MAP_H

#include <cv_bridge/cv_bridge.h>
#include <darknet_ros_msgs/CheckForObjectsAction.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

class DepthMap {
protected:
  cv::Mat depth_map;

public:
  /**
    Loads a ROS image into the map.

    @param msg ROS image, probably provided by a callback
  */
  void process_image(const sensor_msgs::Image::ConstPtr &msg);

  /**
    Gets the depth of a pixel in meters.

    @param x x position
    @param y y position
    @returns depth in meters, or nan if no data available
  */
  float get_depth(int x, int y);

  /**
    Forcibly overrides the depth of a pixel. Be careful with this.

    @param x x position
    @param y y position
    @param depth new depth in meters
  */
  void set_depth(int x, int y, float depth);

  /**
    @brief Gets the width of the map in pixels.
  */
  int get_width();

  /**
    @brief Gets the height of the map in pixels.
  */
  int get_height();

  /**
    @brief Gets a visualizable ROS image of the map
  */
  sensor_msgs::Image get_image_msg();

  /**
    @brief Gets a pointer to the internal matrix
  */
  cv::Mat* get_mat();

  cv::Mat get_sub_map(darknet_ros_msgs::BoundingBox box,
      const float X_ALPHA, const float Y_ALPHA);
};

#endif
