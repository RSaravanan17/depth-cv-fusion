#include "fri_stickler/cv_sweep.h"

/**
  cv_sweep test on a local image file.
  Usage: rosrun fri_stickler cv_sweep_test <relative file path>
*/
int main(int argc, char **argv) {
  ros::init(argc, argv, "cv_test_node");
  ros::NodeHandle node;

  darknet_ros_msgs::BoundingBoxes boxes;
  cv_sweep_local(node, (std::string)argv[argc - 1], &boxes);

  for (int i = 0; i < boxes.bounding_boxes.size(); i++) {
    darknet_ros_msgs::BoundingBox box = boxes.bounding_boxes[i];
    std::cout << box.Class << ": " << box.probability << std::endl;
  }
}
