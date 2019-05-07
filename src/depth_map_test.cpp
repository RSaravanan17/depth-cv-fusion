#include "fri_stickler/depth_map.h"

#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <ros/ros.h>

static DepthMap map;

class DepthMapProc {
public:
  void process(const sensor_msgs::Image::ConstPtr &msg) {
    map.process_image(msg);
    float depth = map.get_depth(map.get_width() / 2, map.get_height() / 2);
    std::cout << depth << " m" << std::endl;
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "depth_map_test");
  ros::NodeHandle nh;
  DepthMapProc proc;

  ros::Subscriber sub =
      nh.subscribe("camera/depth/image", 100, &DepthMapProc::process, &proc);

  ros::spin();
  return 0;
}
