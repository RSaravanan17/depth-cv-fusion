#include "fri_stickler/cv_sweep.h"
#include "fri_stickler/fusion.h"

#include <ros/ros.h>

class FusionProc {
protected:
  ros::NodeHandle *nh;
  ros::Publisher pub_depth_knockout, pub_depth_thresh;
  darknet_ros_msgs::BoundingBoxes boxes;
  DepthMap depth_map;
  cv::Mat rgb;
  int image_width, image_height;
  bool ir_loaded, rgb_loaded;
  std::string target_label;

public:
  FusionProc(ros::NodeHandle *nh, std::string target_label) {
    this->nh = nh;
    pub_depth_knockout = nh->advertise<sensor_msgs::Image>("depth_knockout", 1);
    pub_depth_thresh = nh->advertise<sensor_msgs::Image>("depth_thresh", 1);
    ros::Duration(1.0).sleep();
    ir_loaded = rgb_loaded = false;
    this->target_label = target_label;
  }

  void receive_rgb(const sensor_msgs::Image::ConstPtr &msg) {
    if (rgb_loaded)
      return;

    ROS_INFO("%dx%d RGB image received. Sweeping image...", msg->width,
        msg->height);

    cv_sweep(*nh, *msg, &boxes);
    image_width = msg->width;
    image_height = msg->height;
    rgb = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
    rgb_loaded = true;

    ROS_INFO("Sweep complete.");

    if (ir_loaded)
      process();
  }

  void receive_depth(const sensor_msgs::Image::ConstPtr &msg) {
    if (ir_loaded)
      return;

    ROS_INFO("%dx%d depth image received. Mapping...", msg->width,
        msg->height);

    depth_map.process_image(msg);
    ir_loaded = true;

    ROS_INFO("Depth mapping complete.");

    if (rgb_loaded)
      process();
  }

  void process() {
    FusionProcessor proc(image_width, image_height, rgb);

    int focus_index = -1;
    for (int i = 0; i < boxes.bounding_boxes.size(); i++)
      if (boxes.bounding_boxes[i].Class == target_label) {
        focus_index = i;
        break;
      }

    if (focus_index == -1) {
      ROS_INFO("Failed to detect target in frame");
      return;
    }

    float depth = proc.estimate_distance(depth_map, boxes, focus_index);
    pub_depth_knockout.publish(depth_map.get_image_msg());

    cv_bridge::CvImage depth_thresh_out;
    depth_thresh_out.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
    // depth_thresh_out.image = proc.depth_thresh;
    pub_depth_thresh.publish(depth_thresh_out.toImageMsg());

    ROS_INFO("Estimated distance: %f", depth);
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "fusion_test_node");
  ros::NodeHandle nh;
  FusionProc proc(&nh, "chair");

  ros::Duration(1.0).sleep();

  ros::Subscriber sub1 =
      nh.subscribe("camera/depth/image", 100, &FusionProc::receive_depth,
          &proc);
  ros::Subscriber sub2 =
      nh.subscribe("camera/rgb/image_color", 100, &FusionProc::receive_rgb,
          &proc);

  ros::spin();
  return 0;
}
