#include <cv_bridge/cv_bridge.h>
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

    @param x position
    @param y position
    @returns depth in meters, or nan if no data available
  */
  float get_depth(int x, int y);

  /**
    @brief Get the width of the map in pixels.
  */
  int get_width();

  /**
    @brief Get the height of the map in pixels.
  */
  int get_height();
};