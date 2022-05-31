#include "vrglasses_for_robots_ros/vrglasses_node.h"

#include <iostream>
#include <vrglasses_for_robots/vulkan_renderer.h>
#include <opencv2/highgui.hpp>
#include <ros/console.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/PointCloud.h>
#include <minkindr_conversions/kindr_msg.h>

VRGlassesNode::VRGlassesNode(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private): nh_(nh), nh_private_(nh_private),
    image_transport_(nh_private_), initialized_(false) {
    renderer_ = nullptr;
    odom_sub_ = nh_.subscribe("odometry", 500, &VRGlassesNode::odomCallback, this);

    // Parameters
    double framerate;
    if (!nh_private_.getParam("framerate", framerate)) {
      framerate = 20;
      ROS_WARN("framerate parameter not found, using default(20)");
    }
    diff_frames_.fromSec(1.0 / framerate);
    last_frame_time_ = ros::Time(0);

    int ncameras;
    if(!nh_private_.getParam("ncameras", ncameras)) {
        ROS_WARN("ncameras not specified, using default (mono)");
    } else {
      if (ncameras <= 0) {
        ROS_ERROR("Invalid camera configuration specified! Abort.");
        return;
      }
    }

    std::string body_frame;
    if (!nh_private_.getParam("body_frame", body_frame)) {
      ROS_WARN("body_frame not specified, using default (body)");
      body_frame = "body";
    }

    for (int i = 0; i < ncameras; ++i) {
      // Get the extrinisics transformation for the camera
      Eigen::Matrix<double, 4, 4> T_SC(Eigen::Matrix<double, 4, 4>::Identity());

      std::string ns = "camera_" + std::to_string(i) + "/";
      double roll = 0.0;
      double pitch = nh_private_.param(ns + "pitch", 0.0);
      double yaw = nh_private_.param(ns + "yaw", 0.0);
      Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
      Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
      Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());

      Eigen::Matrix<double, 3, 3> T_fixed;
      T_fixed << 0, 0, 1, -1, 0, 0, 0, -1, 0;
      T_SC.block<3,3>(0,0) = Eigen::Quaterniond(yawAngle * pitchAngle * rollAngle).toRotationMatrix() * T_fixed;

      double p_x = nh_private_.param(ns + "p_x", 0.015);
      double p_y = nh_private_.param(ns + "p_y", 0.055);
      double p_z = nh_private_.param(ns + "p_z", 0.0065);
      T_SC.block<3,1>(0,3) = Eigen::Vector3d(p_x, p_y, p_z);

      visim_project_.T_SCs.push_back(kindr::minimal::QuatTransformation(T_SC));

      // Push back a new camera
      cams_info_.push_back(sensor_msgs::CameraInfo());

      // Register i-th camera
      std::string param_name = ns + "camera_frame_id";
      if (!nh_private_.getParam(param_name,
                                cams_info_.back().header.frame_id)) {
        cams_info_.back().header.frame_id = "world";
        ROS_WARN("%s parameter not found, using default('world')", param_name.c_str());
      }

      // Check that the frames make sense
      bool valid = true;
      for (size_t j = 0; j < cams_info_.size() - 1; ++j) {
        valid &= cams_info_[j].header.frame_id != cams_info_.back().header.frame_id;
        if (!valid) {
          ROS_WARN("Cameras use the same frame! This is unlikely to be correct.");
          break;
        }
      }

      // Fill camera info
      auto camera = visim_project_.camera;
      cams_info_.back().width = camera.w;
      cams_info_.back().height = camera.h;
      cams_info_.back().distortion_model = "plumb_bob";
      cams_info_.back().D = {0.0, 0.0, 0.0, 0.0, 0.0};
      cams_info_.back().K = {camera.f, 0.0, camera.cx,
                       0.0, camera.f, camera.cy,
                       0.0, 0.0, 1};
      cams_info_.back().R = {1.0, 0.0, 0.0,
                       0.0, 1.0, 0.0,
                       0.0, 0.0, 1.0};
      cams_info_.back().P = {camera.f, 0.0, camera.cx, 0.0,
                       0.0, camera.f, camera.cy, 0.0,
                       0.0, 0.0, 1, 0.0};

      // FIXME For stereo camera we account for baseline in field "P"
      /*
      double baseline = (visim_project_.T_SC_left.getPosition() -
                         visim_project_.T_SC_right.getPosition()).norm();

      cam_info_r_.P = {visim_project_.f, 0.0, visim_project_.cx, -visim_project_.f * baseline,
                       0.0, visim_project_.f, visim_project_.cy, 0.0,
                       0.0, 0.0, 1, 0.0};
      */

      // Camera publisher
      cams_pub_.push_back(image_transport::CameraPublisher());
      cams_pub_.back() = image_transport_.advertiseCamera("cam_" + std::to_string(i) + "/image", 1);

      // Images publishers
      depth_pub_.push_back(image_transport_.advertise("depth_map_" + std::to_string(i), 1));
      semantic_pub_.push_back(image_transport_.advertise("semantic_map_" + std::to_string(i), 1));
      cameras_odom_pub_.push_back(nh_private_.advertise<nav_msgs::Odometry>("camera_odometry_out_" + std::to_string(i), 50));

      // Output from renderer
      result_depth_map_.push_back(cv::Mat());
      result_rgbs_map_.push_back(cv::Mat());
      result_rgb_map_.push_back(cv::Mat());
      result_s_map_.push_back(cv::Mat());

      result_rgb_map_.back().create(camera.h, camera.w, CV_8UC3);
      result_s_map_.back().create(camera.h, camera.w, CV_8UC1);

      // Add TF broadcasters
      tf_broadcasters_.push_back(tf::TransformBroadcaster());

      // Calculate tf
      tf::StampedTransform transform;
      transform.setOrigin(tf::Vector3(p_x, p_y, p_z));

      tf::Quaternion q_tf;
      Eigen::Quaterniond q_rpy(T_SC.block<3,3>(0,0));
      q_tf.setX(q_rpy.x());
      q_tf.setY(q_rpy.y());
      q_tf.setZ(q_rpy.z());
      q_tf.setW(q_rpy.w());
      transform.setRotation(q_tf);

      transform.frame_id_ = body_frame;
      transform.child_frame_id_ = cams_info_.back().header.frame_id;

      tf_cameras_.push_back(transform);
    }


    // Initialization is fine
    initialized_ = true;
}

void VRGlassesNode::run()
{
    // Renderer
    std::string shader_folder;
    if (!nh_private_.getParam("shader_folder", shader_folder)) 
    {
        ROS_ERROR("shader_folder not defined");
    }
    

    // ROS Parameters
    nh_private_.param("render_far",far_,far_);    
    
    nh_private_.param("render_near",near_,near_);    

    auto camera = visim_project_.camera;
    renderer_ = new vrglasses_for_robots::VulkanRenderer(camera.w,
          camera.h, near_, far_, shader_folder);
    glm::mat4 empty;
    buildOpenglProjectionFromIntrinsics(perpective_, empty, 
            camera.w, camera.h, camera.f,
            camera.f, 0, camera.cx, camera.cy, near_, far_);

    std::string mesh_obj_file;
    std::string texture_file;
    std::string model_folder;
    std::string model_list_file;
    std::string model_pose_file;

    if(nh_private_.getParam("model_folder", model_folder) && nh_private_.getParam("model_list_file", model_list_file))
    {
        renderer_->loadMeshs(model_folder,model_list_file);
        if(nh_private_.getParam("model_pose_file", model_pose_file))
        {
            ROS_INFO("Load with pose file: %s", model_pose_file.c_str());
            renderer_->loadScene(model_pose_file);
        }
        else
        {
            ROS_INFO("Load without scene file");
            renderer_->noFileScene();
        }
        
    } 
    else if( nh_private_.getParam("mesh_obj_file", mesh_obj_file) && nh_private_.getParam("texture_file", texture_file))
    {
        // Load Mesh
        ROS_INFO("Loading single file");
        renderer_->loadMesh(mesh_obj_file,texture_file);
        renderer_->noFileScene();
    }
    else{
        ROS_ERROR("mesh_obj_file and texture_file need to be defined parameter, alternatively model_folder and model_list_file");
    }

    
    while (::ros::ok()) {
      ::ros::spinOnce();
    }
}

VRGlassesNode::~VRGlassesNode(){
    if (renderer_ != nullptr)
    {
        delete renderer_;
    }
}

void VRGlassesNode::odomCallback(const nav_msgs::Odometry &msg)
{    
    if( msg.header.stamp - last_frame_time_ >= diff_frames_)
    {
        last_frame_time_ = msg.header.stamp;

        // Iterate over the cameras
        for (size_t i = 0; i < result_s_map_.size(); ++i) {
          // Fetch images from renderer
          kindr::minimal::QuatTransformation T_WC_left = computeT_WC_camera(i, msg.pose.pose);
          glm::mat4 mvp_left = computeMVP(T_WC_left);
          renderer_->setCamera(mvp_left);
          renderer_->renderMesh(result_depth_map_[i], result_rgbs_map_[i]);

          // RGB
          cv::Mat out_l[] = { result_rgb_map_[i], result_s_map_[i] };
          int from_to[] = { 0,2, 1,1, 2,0, 3,3 };
          cv::mixChannels(&result_rgbs_map_[i],1,out_l,2,from_to,4);
          sensor_msgs::ImagePtr rgb_msg;
          rgb_msg = cv_bridge::CvImage(msg.header, "rgb8", result_rgb_map_[i]).toImageMsg();
          rgb_msg->header.frame_id = cams_info_[i].header.frame_id;

          cams_info_[i].header.seq = msg.header.seq;
          cams_info_[i].header.stamp = msg.header.stamp;

          cams_pub_[i].publish(*rgb_msg, cams_info_[i]);

          // Semantic
          sensor_msgs::ImagePtr semantic_msg;
          semantic_msg = cv_bridge::CvImage(msg.header, "mono8", result_s_map_[i]).toImageMsg();
          semantic_msg->header.frame_id = cams_info_[i].header.frame_id;
          semantic_pub_[i].publish(semantic_msg);

          // Depth
          sensor_msgs::ImagePtr depth_msg;
          depth_msg = cv_bridge::CvImage(msg.header, "32FC1", result_depth_map_[i]).toImageMsg();
          depth_msg->header.frame_id = cams_info_[i].header.frame_id;
          depth_pub_[i].publish(depth_msg);

          // Camera odometry
          nav_msgs::Odometry odom_msg = msg;
          tf::poseKindrToMsg(T_WC_left,&(odom_msg.pose.pose));
          cameras_odom_pub_[i].publish(odom_msg);

          // Publish TF
          tf_cameras_[i].stamp_ = ros::Time::now();
          tf_broadcasters_[i].sendTransform(tf_cameras_[i]);
        }
    }
}

kindr::minimal::QuatTransformation VRGlassesNode::computeT_WC_camera(size_t cam_id, const geometry_msgs::Pose &pose) const
{
    Eigen::Vector3d p_WS(pose.position.x, pose.position.y, pose.position.z);

    Eigen::Quaterniond q_WS;
    q_WS.x() = pose.orientation.x;
    q_WS.y() = pose.orientation.y;
    q_WS.z() = pose.orientation.z;
    q_WS.w() = pose.orientation.w;
    q_WS.normalize();

    kindr::minimal::QuatTransformation T_WC =
            kindr::minimal::QuatTransformation(p_WS, q_WS) * visim_project_.T_SCs[cam_id];

    return T_WC;
}

glm::mat4 VRGlassesNode::computeMVP(const kindr::minimal::QuatTransformation &T_WC)
{
    kindr::minimal::QuatTransformation T_CW_cv = T_WC.inverse();
    auto T_CW_cv_eigen = T_CW_cv.getTransformationMatrix();
    glm::mat4 T_CW_cv_glm;
    glm::mat4 conversion_gl_cv = glm::mat4(1,0,0,0,0,-1,0,0,0,0,-1,0,0,0,0,1);

    for (int i = 0; i < 4; ++i)
    {
        for (int j = 0; j < 4; ++j)
        {
            T_CW_cv_glm[j][i] = T_CW_cv_eigen(i, j);
        }
    }
    return perpective_ * conversion_gl_cv * T_CW_cv_glm; //mvp_fix;//
}

void VRGlassesNode::buildOpenglProjectionFromIntrinsics(glm::mat4 &matPerspective, glm::mat4 &matProjection/*, glm::mat4 &matCVProjection*/, int img_width, int img_height, float alpha, float beta, float skew, float u0, float v0, float near, float far) {
    // These parameters define the final viewport that is rendered into by the
    // camera.
    float l = 0;
    float r = img_width;
    float b = 0;
    float t = img_height;

    // near and far clipping planes, these only matter for the mapping from
    // world-space z-coordinate into the depth coordinate for OpenGL
    float n = near;
    float f = far;

    //  // set the viewport parameters
    //  viewport[0] = l;
    //  viewport[1] = b;
    //  viewport[2] = r - l;
    //  viewport[3] = t - b;

    // construct an orthographic matrix which maps from projected coordinates to
    // normalized device coordinates in the range
    // [-1, 1].  OpenGL then maps coordinates in NDC to the current viewport
    glm::mat4 ndc(0);
    ndc[0][0] = 2.0 / (r - l);
    ndc[3][0] = -(r + l) / (r - l);
    ndc[1][1] = 2.0 / (t - b);
    ndc[3][1] = -(t + b) / (t - b);
    ndc[2][2] = -2.0 / (f - n);
    ndc[3][2] = -(f + n) / (f - n);
    ndc[3][3] = 1.0;

    // construct a projection matrix, this is identical to the projection matrix
    // computed for the intrinsic,
    // except an additional row is inserted to map the z-coordinate to OpenGL.
    // CMatrix4<T> matProjection(0);    // the 3rd column is inverted to make the
    // camera look towards +Z (instead of -Z in opengl)
    matProjection = glm::mat4(0);
    matProjection[0][0] = alpha;
    matProjection[1][0] = skew;
    matProjection[2][0] = -u0;
    matProjection[1][1] = beta;
    matProjection[2][1] = -v0;
    matProjection[2][2] = (n + f);
    matProjection[3][2] = n * f;
    matProjection[2][3] = -1.0;

    //    matCVProjection = glm::mat4(0);
    //    matCVProjection[0][0] = alpha;
    //    matCVProjection[1][0] = skew;
    //    matCVProjection[2][0] = u0;
    //    matCVProjection[1][1] = beta;
    //    matCVProjection[2][1] = v0;
    //    matCVProjection[2][2] = 1;
    //    matCVProjection[3][2] = 0;
    //    matCVProjection[2][3] = 1;
    //    matCVProjection[3][3] = 1;

    // resulting OpenGL frustum is the product of the orthographic
    // mapping to normalized device coordinates and the augmented camera intrinsic
    // matrix
    matPerspective = ndc * matProjection;
    matPerspective[1][1] *= -1; //was originally designed for OpenGL, where the Y coordinate of the clip coordinates is inverted in relation Vulkan.
}
