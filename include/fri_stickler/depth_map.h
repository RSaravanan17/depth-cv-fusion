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
};
