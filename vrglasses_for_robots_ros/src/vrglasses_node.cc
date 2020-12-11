
#include <vrglasses_for_robots_ros/vrglasses_node.h>
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
    last_frame_time_ = ros::Time::now();

    if(!nh_private_.getParam("stereo", stereo_)) {
      stereo_ = false;
      ROS_WARN("stereo parameter not found, using default(false)");
    }

    if(stereo_){
      if (!nh_private_.getParam("camera_frame_id_l", cam_info_l_.header.frame_id)) {
        cam_info_l_.header.frame_id = "world";
        ROS_WARN("camera_frame_id_l parameter not found, using default('world')");
      }
      if (!nh_private_.getParam("camera_frame_id_r", cam_info_r_.header.frame_id)) {
        cam_info_r_.header.frame_id = "world";
        ROS_WARN("camera_frame_id_r parameter not found, using default('world')");
      }

      if(cam_info_l_.header.frame_id == cam_info_r_.header.frame_id){
        ROS_WARN("Both cameras use the same frame! This is unlikely to be correct.");
      }

    }else{
      if (!nh_private_.getParam("camera_frame_id", cam_info_l_.header.frame_id)) {
        cam_info_l_.header.frame_id = "world";
        ROS_WARN("camera_frame_id parameter not found, using default('world')");
      }
    }

    // Publishers
    depth_pub_ = image_transport_.advertise("depth_map", 1);
    semantic_pub_ = image_transport_.advertise("semantic_map", 1);

    camera_odom_pub_ = nh_private_.advertise<nav_msgs::Odometry>("camera_odometry_out", 50);

    if(stereo_){
      cam_l_pub_ = image_transport_.advertiseCamera("stereo/left/image", 1);
      cam_r_pub_ = image_transport_.advertiseCamera("stereo/right/image", 1);
    }else{
      cam_l_pub_ = image_transport_.advertiseCamera("color_map", 1);
    }

    cam_info_l_.width = visim_project_.w;
    cam_info_l_.height = visim_project_.h;
    cam_info_l_.distortion_model = "plumb_bob";
    cam_info_l_.D = {0.0, 0.0, 0.0, 0.0, 0.0};
    cam_info_l_.K = {visim_project_.f, 0.0, visim_project_.cx,
                     0.0, visim_project_.f, visim_project_.cy,
                     0.0, 0.0, 1};
    cam_info_l_.R = {1.0, 0.0, 0.0,
                     0.0, 1.0, 0.0,
                     0.0, 0.0, 1.0};
    cam_info_l_.P = {visim_project_.f, 0.0, visim_project_.cx, 0.0,
                     0.0, visim_project_.f, visim_project_.cy, 0.0,
                     0.0, 0.0, 1, 0.0};

    if(stereo_){
      cam_info_r_.width = cam_info_l_.width;
      cam_info_r_.height = cam_info_l_.height;
      cam_info_r_.distortion_model = cam_info_l_.distortion_model;
      cam_info_r_.D = cam_info_l_.D;
      cam_info_r_.K = cam_info_l_.K;
      cam_info_r_.R = cam_info_l_.R;

      double baseline = (visim_project_.T_SC_left.getPosition() - 
                         visim_project_.T_SC_right.getPosition()).norm();

      cam_info_r_.P = {visim_project_.f, 0.0, visim_project_.cx, -visim_project_.f * baseline,
                       0.0, visim_project_.f, visim_project_.cy, 0.0,
                       0.0, 0.0, 1, 0.0};
    }

    //dense_pointcloud_pub_ =
    //        nh_private_.advertise<sensor_msgs::PointCloud>("labelled_dense_pointcloud", 5); //TODO add param to enable the point cloud publication

    result_rgb_map_l_.create(visim_project_.h,visim_project_.w,CV_8UC3);
    result_s_map_l_.create(visim_project_.h,visim_project_.w,CV_8UC1);
    result_rgb_map_r_.create(visim_project_.h,visim_project_.w,CV_8UC3);
    result_s_map_r_.create(visim_project_.h,visim_project_.w,CV_8UC1);

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
    std::string shader_vert_spv =
        shader_folder + "/vrglasses4robots_shader.vert.spv";
    std::string shader_frag_spv =
        shader_folder + "/vrglasses4robots_shader.frag.spv";

    // ROS Parameters
    nh_private_.param("render_far",far_,far_);    
    
    nh_private_.param("render_near",near_,near_);    

    renderer_ = new vrglasses_for_robots::VulkanRenderer(visim_project_.w, 
          visim_project_.h, near_, far_, shader_vert_spv, shader_frag_spv);
    glm::mat4 empty;
    buildOpenglProjectionFromIntrinsics(perpective_, empty, 
            visim_project_.w, visim_project_.h, visim_project_.f,
            visim_project_.f, 0, visim_project_.cx, visim_project_.cy, near_, far_);

    std::string mesh_obj_file;
    std::string texture_file;
    if(!nh_private_.getParam("mesh_obj_file", mesh_obj_file))
    {
        ROS_ERROR("mesh_obj_file parameter not defined");
    }

    if(!nh_private_.getParam("texture_file", texture_file))
    {
        ROS_ERROR("texture_file parameter not defined");
    }

    // Load Mesh
    renderer_->loadMesh(mesh_obj_file,texture_file);
    
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
        kindr::minimal::QuatTransformation T_WC_left = computeT_WC_left(msg.pose.pose);
        glm::mat4 mvp_left = computeMVP(T_WC_left);
        renderer_->setCamera(mvp_left);
        renderer_->renderMesh(result_depth_map_l_, result_rgbs_map_l_);

        nav_msgs::Odometry odom_msg = msg;        
        tf::poseKindrToMsg(T_WC_left,&(odom_msg.pose.pose));
        camera_odom_pub_.publish(odom_msg);

        cv::Mat out_l[] = { result_rgb_map_l_,result_s_map_r_ };
        int from_to[] = { 0,2, 1,1, 2,0, 3,3 };
        cv::mixChannels(&result_rgbs_map_l_,1,out_l,2,from_to,4);
        sensor_msgs::ImagePtr rgb_msg_l, rgb_msg_r;
        rgb_msg_l = cv_bridge::CvImage(msg.header, "rgb8", result_rgb_map_l_).toImageMsg();
        rgb_msg_l->header.frame_id = cam_info_l_.header.frame_id;

        ros::Time now = ros::Time().now();
        cam_info_l_.header.stamp = now;
        rgb_msg_l->header.stamp = now;

        cam_l_pub_.publish(*rgb_msg_l, cam_info_l_);

        if(stereo_){
          kindr::minimal::QuatTransformation T_WC_right = computeT_WC_right(msg.pose.pose);
          glm::mat4 mvp_right = computeMVP(T_WC_right);
          renderer_->setCamera(mvp_right);
          renderer_->renderMesh(result_depth_map_r_, result_rgbs_map_r_);
          
          cv::Mat out_r[] = { result_rgb_map_r_,result_s_map_r_ };
          cv::mixChannels(&result_rgbs_map_r_,1,out_r,2,from_to,4);
          rgb_msg_r = cv_bridge::CvImage(msg.header, "rgb8", result_rgb_map_r_).toImageMsg();
          rgb_msg_r->header.frame_id = cam_info_r_.header.frame_id;

          cam_info_r_.header.stamp = now;
          rgb_msg_r->header.stamp = now;

          cam_r_pub_.publish(*rgb_msg_r, cam_info_r_);
        }

        sensor_msgs::ImagePtr semantic_msg;
        semantic_msg = cv_bridge::CvImage(msg.header, "mono8", result_s_map_l_).toImageMsg();
        semantic_msg->header.frame_id = cam_info_l_.header.frame_id;
        semantic_pub_.publish(semantic_msg);

        sensor_msgs::ImagePtr depth_msg;
        depth_msg = cv_bridge::CvImage(msg.header, "32FC1", result_depth_map_l_).toImageMsg();
        depth_msg->header.frame_id = cam_info_l_.header.frame_id;
        depth_pub_.publish(depth_msg);

        //publishDenseSemanticCloud(msg.header,depth_msg,result_s_map_); //TODO add param to enable the point cloud publication
    }
}

void VRGlassesNode::publishDenseSemanticCloud(const ::std_msgs::Header header, const sensor_msgs::ImagePtr &depth_map, const cv::Mat &semantic_map)
{
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvShare(depth_map);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    sensor_msgs::PointCloud labelled_pointcloud;
    labelled_pointcloud.header = header;

    sensor_msgs::ChannelFloat32 quality_channel;
    quality_channel.name = "quality";

    float* data = (float*)cv_ptr->image.data;
    for (int u = 0; u < cv_ptr->image.cols; u += 4)
    {
      for (int v = 0; v < cv_ptr->image.rows; v += 4)
      {
        float val = data[v * cv_ptr->image.cols + u];
        float semantic_val = semantic_map.at<uchar>(v,u)/255.0;
        if (std::isfinite(val))
        {

          geometry_msgs::Point32 p_C_msg;
          p_C_msg.x = val * (u - visim_project_.cx) / visim_project_.f;
          p_C_msg.y = val * (v - visim_project_.cy) / visim_project_.f;
          p_C_msg.z = val;

          quality_channel.values.push_back(semantic_val);
          labelled_pointcloud.points.push_back(p_C_msg);
        }
      }
    }

    // Now add the channel indicating the quality
    labelled_pointcloud.channels.push_back(quality_channel);

    // Publish the pointcloud
    dense_pointcloud_pub_.publish(labelled_pointcloud);
  }

kindr::minimal::QuatTransformation VRGlassesNode::computeT_WC_left(const geometry_msgs::Pose &pose)
{
    Eigen::Vector3d p_WS(pose.position.x, pose.position.y, pose.position.z);

    Eigen::Quaterniond q_WS;
    q_WS.x() = pose.orientation.x;
    q_WS.y() = pose.orientation.y;
    q_WS.z() = pose.orientation.z;
    q_WS.w() = pose.orientation.w;

    kindr::minimal::QuatTransformation T_WC =
            kindr::minimal::QuatTransformation(p_WS, q_WS) * visim_project_.T_SC_left;

    return T_WC;
}
kindr::minimal::QuatTransformation VRGlassesNode::computeT_WC_right(const geometry_msgs::Pose &pose)
{
    Eigen::Vector3d p_WS(pose.position.x, pose.position.y, pose.position.z);

    Eigen::Quaterniond q_WS;
    q_WS.x() = pose.orientation.x;
    q_WS.y() = pose.orientation.y;
    q_WS.z() = pose.orientation.z;
    q_WS.w() = pose.orientation.w;

    kindr::minimal::QuatTransformation T_WC =
            kindr::minimal::QuatTransformation(p_WS, q_WS) * visim_project_.T_SC_right;

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
