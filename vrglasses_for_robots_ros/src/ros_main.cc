#include <iostream>

#include <vrglasses_for_robots_ros/vrglasses_node.h>

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "ros_backend_node");

  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");

  ROS_INFO("ros_backend_node started");

  VRGlassesNode node(nh, nh_private);

  if (!node.initialized()) {
    return EXIT_FAILURE;
  }

  node.run();

  ROS_INFO("shutting down ros_backend_node");
  return EXIT_SUCCESS;
}
