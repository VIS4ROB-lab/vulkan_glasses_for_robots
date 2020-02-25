
#include <vrglasses_for_robots_ros/vrglasses_node.h>
#include <iostream>
#include <vrglasses_for_robots/vulkan_renderer.h>
#include <opencv2/highgui.hpp>
#include <ros/console.h>
#include <cv_bridge/cv_bridge.h>




VRGlassesNode::VRGlassesNode(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private): nh_(nh), nh_private_(nh_private),
    image_transport_(nh_private_){
    renderer_ = nullptr;
    odom_sub_ = nh_.subscribe("odometry", 500, &VRGlassesNode::odomCallback, this);

    depth_pub_ = image_transport_.advertise("depth_map", 1);
    color_pub_ = image_transport_.advertise("color_map", 1);
    semantic_pub_ = image_transport_.advertise("semantic_map", 1);

    result_rgb_map_.create(visim_project_.h,visim_project_.w,CV_8UC3);
    result_s_map_.create(visim_project_.h,visim_project_.w,CV_8UC1);



    double framerate;

    if(!nh_private_.getParam("framerate", framerate))
    {
        framerate = 20;
        ROS_WARN("framerate parameter not found, using default(20)");
    }
    diff_frames_.fromSec(1.0/framerate);
    last_frame_time_ = ros::Time::now();

}

void VRGlassesNode::run()
{
    renderer_ = new vrglasses_for_robots::VulkanRenderer(visim_project_.w,visim_project_.h,near_,far_);
    glm::mat4 empty;
    buildOpenglProjectionFromIntrinsics(perpective_,empty,visim_project_.w,visim_project_.h,visim_project_.f,visim_project_.f,0,visim_project_.cx,visim_project_.cy,near_,far_);

    cv::Mat result_depth_map, result_rgb_map, result_semantic_map;

    renderer_->loadMesh("/media/secssd/code/vrglasses4robots/data/models/50s_house_v2_45_3_Zu_Xf.obj","/media/secssd/code/vrglasses4robots/data/textures/new_texture_small.tga");

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
    //std::cout << msg.pose.pose.position.x << std::endl;
    if( msg.header.stamp - last_frame_time_ >= diff_frames_)
    {
        //std::cout << "######"  << std::endl;
        last_frame_time_ = msg.header.stamp;
        glm::mat4 mvp = computeMVP(msg.pose.pose);
        renderer_->setCamera(mvp);
        renderer_->renderMesh(result_depth_map_, result_rgbs_map_);

        cv::Mat out[] = { result_rgb_map_,result_s_map_ };
        int from_to[] = { 0,0, 1,1, 2,2, 3,3 };
        cv::mixChannels(&result_rgbs_map_,1,out,2,from_to,4);
        sensor_msgs::ImagePtr rgb_msg;
        rgb_msg = cv_bridge::CvImage(msg.header, "rgb8", result_rgb_map_).toImageMsg();
        color_pub_.publish(rgb_msg);
        cv::imshow("RGB",result_rgb_map_);

        sensor_msgs::ImagePtr semantic_msg;
        semantic_msg = cv_bridge::CvImage(msg.header, "mono8", result_s_map_).toImageMsg();
        semantic_pub_.publish(semantic_msg);

        sensor_msgs::ImagePtr depth_msg;
        depth_msg = cv_bridge::CvImage(msg.header, "32FC1", result_depth_map_).toImageMsg();
        depth_pub_.publish(depth_msg);
        cv::waitKey(1);

    }
}


//bool VisimProcessor::initialization(const std::string &visim_project_folder, const std::string &camera_id, const std::string &output_folder_path, int step) {
//    if (!boost::filesystem::exists(output_folder_path)) {
//        if (!boost::filesystem::create_directories(output_folder_path)) {
//            LOG(ERROR) << "the output folder could not be created :"
//                       << output_folder_path.c_str();
//            return false;
//        }
//    } else {
//        LOG(ERROR)
//                << "the output folder already exist - please delete it or "
//                   "change the arg:"
//                << output_folder_path.c_str();
//        // return false;
//    }

//    visim_data_source_ = DataSource::Create(visim_project_folder, camera_id);

//    if (!visim_data_source_) {
//        LOG(ERROR) << "could not open visim data project";
//        return false;
//    }

//    boost::filesystem::path output_path_sparse = output_folder_path;


//    return true;
//}


//void VisimProcessor::runHeadless()
//{
//    VisimProject project =  visim_data_source_->getProject();
//    double near = 0.1, far = 100.0;
//    vrglasses_for_robots::VulkanRenderer app = vrglasses_for_robots::VulkanRenderer(project.w,project.h,near,far);
//    cv::Mat result_depth_map, result_rgb_map, result_semantic_map;

//    app.loadMesh("/media/secssd/code/vrglasses4robots/data/models/50s_house_v2_45_3_Zu_Xf.obj","/media/secssd/code/vrglasses4robots/data/textures/new_texture_small.tga");

//    try {
//        size_t count = 0;
//        glm::mat4 empty;

//        buildOpenglProjectionFromIntrinsics(perpective_,empty,project.w,project.h,project.f,project.f,0,project.cx,project.cy,near,far);
//        while(count < 1000){
//            DataEntry current = visim_data_source_->at(count);
//            std::cout << current.sequence << std::endl;
//            glm::mat4 mvp = computeMVP(current);
//            app.setCamera(mvp);
//            cv::Mat show_img, channels[4];
//            app.renderMesh(result_depth_map, result_semantic_map);
//            cv::split(result_semantic_map,channels);
//            //cv::imshow("RGB",result_semantic_map);
//            //cv::imshow("Semantics",channels[3]);
//            if (1)
//            {
//                    double min_depth = 0, max_depth = 30;
//                    //cv::minMaxLoc(mesh_depth_image, &min_depth, &max_depth);

//                    result_depth_map.convertTo(
//                        show_img, CV_8U, 255.0 / (max_depth - min_depth),
//                        -min_depth * 255.0 / (max_depth - min_depth));
//                    //cv::imshow("depth map render", show_img);
//                    //LOG(INFO) << "rende " << min_depth << " / " << max_depth << " - "
//                    //          << okvis_reader.getNextId() << " / " << okvis_reader.size();
//            }
//            count = (count +1) % visim_data_source_->size();
//            //cv::waitKey(1);
//        }
//    } catch (const std::exception& e) {
//        std::cerr << e.what() << std::endl;
//        return;
//    }
//}



glm::mat4 VRGlassesNode::computeMVP(const geometry_msgs::Pose &pose)
{
        Eigen::Vector3d p_WS(pose.position.x, pose.position.y, pose.position.z);

        Eigen::Quaterniond q_WS;
        q_WS.x() = pose.orientation.x;
        q_WS.y() = pose.orientation.y;
        q_WS.z() = pose.orientation.z;
        q_WS.w() = pose.orientation.w;;

        kindr::minimal::QuatTransformation T_WC =
                kindr::minimal::QuatTransformation(p_WS, q_WS) * visim_project_.T_SC;

//    Eigen::Vector3d p_WS(16.0169, 10.601, 23.9317);

//    Eigen::Quaterniond q_WS;
//    q_WS.x() = -0.392015;
//    q_WS.y() = -0.839755;
//    q_WS.z() = 0.344885;
//    q_WS.w() = 0.148966;

//    kindr::minimal::QuatTransformation T_WC =
//            kindr::minimal::QuatTransformation( q_WS,p_WS);// * visim_project_.T_SC;

    glm::mat4 mvp_fix = glm::mat4(-0.571972, 1.187686, -0.620847, -0.619607, 1.066311, 0.619799, -0.340906, -0.340225, -0.013738, -1.341432, -0.708759, -0.707343, 3.019675, 7.037336, 30.097649, 30.237314);



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
