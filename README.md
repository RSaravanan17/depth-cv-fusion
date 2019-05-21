# depth-cv-fusion

A research project on depth map and CV fusion done for Dr. Justin Hart's Autonomous Intelligent Robotics course at the University of Texas at Austin.

YOLO V3 and an Xbox 360 Kinect supply bounding boxes and depth maps to a ROS node that predicts the distance between some focal object and the camera lens. The main implementation is in `fusion.cpp/FusionProcessor::estimate_distance`, and a paper detailing the algorithm and its efficacy can be found [here](https://stefandebruyn.github.io/papers/depth_cv_fusion.pdf).

## ROS Dependencies

* darknet_ros
* freenect
