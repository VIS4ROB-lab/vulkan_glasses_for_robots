
#ifndef VRGLASSES4ROBOTS_VRGLASSES_NODE_H__
#define VRGLASSES4ROBOTS_VRGLASSES_NODE_H__

#define GLM_FORCE_RADIANS
#define GLM_FORCE_DEPTH_ZERO_TO_ONE
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/hash.hpp>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <image_transport/image_transport.h>

class VRGlassesNode {
public:
    VRGlassesNode(const ros::NodeHandle& nh,
                  const ros::NodeHandle& nh_private): nh_(nh), nh_private_(nh_private),
        rgb_imt_(nh_private_),
        semantic_imt_(nh_private_),
        depth_imt_(nh_private_){
    }
    void run();

private:

    // ros
    ::ros::NodeHandle nh_;
    ::ros::NodeHandle nh_private_;
    ::ros::Subscriber odom_sub_;
    ::ros::Publisher target_pub_;

    image_transport::Publisher rgb_pub_;
    image_transport::ImageTransport rgb_imt_;

    image_transport::Publisher semantic_pub_;
    image_transport::ImageTransport semantic_imt_;

    image_transport::Publisher depth_pub_;
    image_transport::ImageTransport depth_imt_;


    void odomCallback(const nav_msgs::Odometry &msg);

    // rendering
    //glm::mat4 computeMVP(DataEntry& entry);
    glm::mat4 perpective_;

    void buildOpenglProjectionFromIntrinsics(
            glm::mat4& matPerspective, glm::mat4& matProjection,
            /*glm::mat4& matCVProjection,*/ int img_width, int img_height, float alpha,
            float beta, float skew, float u0, float v0, float near, float far);
};

#endif
