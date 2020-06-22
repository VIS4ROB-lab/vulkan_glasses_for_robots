
#ifndef VRGLASSES4ROBOTS_VRGLASSES_NODE_H__
#define VRGLASSES4ROBOTS_VRGLASSES_NODE_H__

#define GLM_FORCE_RADIANS
#define GLM_FORCE_DEPTH_ZERO_TO_ONE
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/hash.hpp>
#include <eigen3/Eigen/Core>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <image_transport/image_transport.h>
#include <kindr/minimal/quat-transformation.h>
#include <opencv2/core.hpp>

namespace  vrglasses_for_robots{
class VulkanRenderer;
}


class VRGlassesNode {
public:
    VRGlassesNode(const ros::NodeHandle& nh,
                  const ros::NodeHandle& nh_private);
    void run();
    ~VRGlassesNode();

private:

    //parameters
    ros::Duration diff_frames_;
    ros::Time last_frame_time_;


    // ros
    ::ros::NodeHandle nh_;
    ::ros::NodeHandle nh_private_;
    ::ros::Subscriber odom_sub_;
    ::ros::Publisher target_pub_;

    image_transport::Publisher color_pub_;
    //image_transport::ImageTransport rgb_imt_;
    ros::Publisher dense_pointcloud_pub_;

    image_transport::Publisher semantic_pub_;
    //image_transport::ImageTransport semantic_imt_;

    image_transport::Publisher depth_pub_;
    //image_transport::ImageTransport depth_imt_;
    image_transport::ImageTransport image_transport_;



    void odomCallback(const nav_msgs::Odometry &msg);

    void publishDenseSemanticCloud(const std_msgs::Header header, const sensor_msgs::ImagePtr &depth_map, const cv::Mat& semantic_map);

    // rendering
    vrglasses_for_robots::VulkanRenderer* renderer_;
    glm::mat4 perpective_;
    double near_ = 0.1, far_ = 500.0;
    glm::mat4 computeMVP(const geometry_msgs::Pose &pose);
    cv::Mat result_depth_map_, result_rgbs_map_,result_rgb_map_,result_s_map_;

    void buildOpenglProjectionFromIntrinsics(
            glm::mat4& matPerspective, glm::mat4& matProjection,
            /*glm::mat4& matCVProjection,*/ int img_width, int img_height, float alpha,
            float beta, float skew, float u0, float v0, float near, float far);

    class VisimProject {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        double f;
        float cx, cy;
        int w, h, sampling_factor, imu_image_delay;
        kindr::minimal::QuatTransformation T_SC;
        VisimProject() {
            f = 455;
            cx = 376.5;
            cy = 240.5;
            w = 752;
            h = 480;
            imu_image_delay = 1;  // nanosec

            sampling_factor = 10;
            Eigen::Matrix<double, 4, 4> t_sc;
            t_sc << 0, 0, 1, 0.015, -1, 0, 0, 0.055, 0, -1, 0, 0.0065, 0, 0, 0, 1;
            T_SC = kindr::minimal::QuatTransformation(t_sc);
        }
        // todo load from json project
    };

    VisimProject visim_project_;


};

#endif
