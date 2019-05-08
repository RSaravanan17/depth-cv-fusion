#include "fri_stickler/cv_sweep.h"
#include "fri_stickler/fusion.h"

#include <ros/ros.h>

int image_msgs = 0;
int depth_msgs = 0;

class FusionProc {
protected:
  ros::NodeHandle *nh;
  ros::Publisher pub_depth_knockout;
  darknet_ros_msgs::BoundingBoxes boxes;
  DepthMap depth_map;
  int image_width, image_height;

public:
  FusionProc(ros::NodeHandle *nh) {
    this->nh = nh;
    pub_depth_knockout = nh->advertise<sensor_msgs::Image>("depth_knockout", 1);
  }

  void receive_rgb(const sensor_msgs::Image::ConstPtr &msg) {
    if (image_msgs > 1)
      return;

    ROS_INFO("RGB received. Sweeping image...");

    cv_sweep(*nh, *msg, &boxes);
    image_msgs++;
    image_width = msg->width;
    image_height = msg->height;

    ROS_INFO("Sweep complete.");

    if (image_msgs == 2 && depth_msgs > 0)
      process();
  }

  void receive_depth(const sensor_msgs::Image::ConstPtr &msg) {
    if (depth_msgs > 0)
      return;

    ROS_INFO("Depth received. Mapping...");

    depth_map.process_image(msg);
    depth_msgs++;

    ROS_INFO("Depth mapping complete.");

    if (image_msgs == 2)
      process();
  }

  void process() {
    FusionProcessor proc(image_width, image_height);

    int focus_index = -1;
    for (int i = 0; i < boxes.bounding_boxes.size(); i++)
      if (boxes.bounding_boxes[i].Class == "keyboard") {
        focus_index = i;
        break;
      }

    if (focus_index == -1) {
      ROS_INFO("Failed to detect target in frame");
      return;
    }

    float depth = proc.estimate_distance(depth_map, boxes, focus_index);
    pub_depth_knockout.publish(depth_map.get_image_msg());
    ROS_INFO("Estimated distance: %f", depth);
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "fusion_test_node");
  ros::NodeHandle nh;
  FusionProc proc(&nh);

  ros::Subscriber sub1 =
      nh.subscribe("camera/depth/image", 100, &FusionProc::receive_depth,
          &proc);
  ros::Subscriber sub2 =
      nh.subscribe("camera/rgb/image_color", 100, &FusionProc::receive_rgb,
          &proc);

  ros::spin();
  return 0;
}
