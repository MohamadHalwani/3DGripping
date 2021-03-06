#include <librealsense2/rs.hpp>
#include <algorithm>            // std::min, std::max
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>

#include <ros/ros.h>

#include <pcl/common/common_headers.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/extract_indices.h>
#include <pcl_ros/transforms.h>

#include <geometry_msgs/PoseStamped.h>
#include "geometry_msgs/Pose.h"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread/thread.hpp>
#include <string>

typedef pcl::PointXYZRGB P_pcl;
typedef pcl::PointCloud<P_pcl> point_cloud;
typedef point_cloud::Ptr ptr_cloud;
Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity();

ros::Subscriber pose_sub;
using namespace cv;


// Get RGB values based on normals - texcoords, normals value [u v]
std::tuple<uint8_t, uint8_t, uint8_t> get_texcolor(rs2::video_frame texture, rs2::texture_coordinate texcoords)
{
    const int w = texture.get_width(), h = texture.get_height();
    
    // convert normals [u v] to basic coords [x y]
    int x = std::min(std::max(int(texcoords.u*w + .5f), 0), w - 1);
    int y = std::min(std::max(int(texcoords.v*h + .5f), 0), h - 1);

    int idx = x * texture.get_bytes_per_pixel() + y * texture.get_stride_in_bytes();
    const auto texture_data = reinterpret_cast<const uint8_t*>(texture.get_data());
    return std::tuple<uint8_t, uint8_t, uint8_t>(texture_data[idx], texture_data[idx+1], texture_data[idx+2]);
}


ptr_cloud points_to_pcl(const rs2::points& points, const rs2::video_frame& color){

    // OpenCV Mat for showing the rgb color image, just as part of processing
    Mat colorr(Size(640, 480), CV_8UC3, (void*)color.get_data(), Mat::AUTO_STEP);
    namedWindow("Display Image", WINDOW_AUTOSIZE );
    imshow("Display Image", colorr);
        
    auto sp = points.get_profile().as<rs2::video_stream_profile>();
    ptr_cloud cloud(new point_cloud);
    
    // Config of PCL Cloud object
    cloud->width = static_cast<uint32_t>(sp.width());
    cloud->height = static_cast<uint32_t>(sp.height());
    cloud->is_dense = false;
    cloud->points.resize(points.size());

    auto tex_coords = points.get_texture_coordinates();
    auto vertices = points.get_vertices();

    // Iterating through all points and setting XYZ coordinates
    // and RGB values
    for (int i = 0; i < points.size(); ++i)
    {
        cloud->points[i].x = vertices[i].x;
        cloud->points[i].y = vertices[i].y;
        cloud->points[i].z = vertices[i].z;

        std::tuple<uint8_t, uint8_t, uint8_t> current_color;
        current_color = get_texcolor(color, tex_coords[i]);

        // Reversed order- 2-1-0 because of BGR model used in camera
        cloud->points[i].r = std::get<2>(current_color);
        cloud->points[i].g = std::get<1>(current_color);
        cloud->points[i].b = std::get<0>(current_color);

    }
    
   return cloud;
}

void FitPlanetoPC(ptr_cloud filtered_cloud)
{
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  
  // Create the segmentation object
  pcl::SACSegmentation<P_pcl> seg;
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.01);
  seg.setInputCloud (filtered_cloud);
  seg.segment (*inliers, *coefficients);

  if (inliers->indices.size () == 0)
  {
    PCL_ERROR ("Could not estimate a planar model for the given dataset.");
  }

  Eigen::Vector4f plane_parameters;
  pcl::ModelCoefficients plane_coeff; 
  plane_coeff.values.resize (4);
  plane_coeff.values[0] =  coefficients->values[0]; 
  plane_coeff.values[1] =  coefficients->values[1]; 
  plane_coeff.values[2] =  coefficients->values[2]; 
  plane_coeff.values[3] =  coefficients->values[3]; 

  std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
                                      << coefficients->values[1] << " "
                                      << coefficients->values[2] << " " 
                                      << coefficients->values[3] << std::endl;

}

void PoseCallback(const geometry_msgs::PoseStamped::ConstPtr &camera_pose)
{
    
    const Eigen::Vector3d position(camera_pose->pose.position.x,
                                    camera_pose->pose.position.y,
                                    camera_pose->pose.position.z);
    const Eigen::Quaterniond quat(camera_pose->pose.orientation.w,
                                    camera_pose->pose.orientation.x,
                                    camera_pose->pose.orientation.y,
                                    camera_pose->pose.orientation.z);

    ros::shutdown();
}

int main(int argc, char **argv) {

    // ros::init(argc, argv, "listener");
    // ros::NodeHandle n;

    // Subscribers 
    //pose_sub = n.subscribe("/dvs/pose", 1, PoseCallback);
    
    //Publisher 
    //ros::Publisher FinalPC_pub = n.advertise<sensor_msgs::PointCloud2>("d345_rosPC", 1);
    

    // Declare pointcloud object, for calculating pointclouds and texture mappings
    rs2::pointcloud pc;
    
    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;

    // Create a configuration for configuring the pipeline with a non default profile
    rs2::config cfg;

    // Add desired streams to configuration
    cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
    cfg.enable_stream(RS2_STREAM_INFRARED, 640, 480, RS2_FORMAT_Y8, 30);
    cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
   
    // Start streaming with default recommended configuration
    pipe.start(cfg);

    // Camera warmup - dropping several first frames to let auto-exposure stabilize
    for(int i = 0; i < 100; i++)
    {
        // Wait for all configured streams to produce a frame
        auto frames = pipe.wait_for_frames();
    }
        
    auto frames = pipe.wait_for_frames();
    auto depth = frames.get_depth_frame();
    auto colored_frame = frames.get_color_frame();

    // Order here is crucial! 
    // map_to() color frame has to be done befor point calculation
    // otherwise texture won't be mapped
    pc.map_to(colored_frame);
    auto points = pc.calculate(depth);
   
    // Actual calling of conversion and saving XYZRGB cloud to file
    ptr_cloud cloud = points_to_pcl(points, colored_frame);
    ptr_cloud filtered_cloud(new point_cloud);
    ptr_cloud filtered_cloud_Inertial(new point_cloud);

    // Config of PCL Cloud object
    filtered_cloud->width = 100;
    filtered_cloud->height = 1;
    filtered_cloud->is_dense = false;
    filtered_cloud->points.resize(100);;
    //filtered_cloud->header.frame_id = "camera";
    pcl::PointXYZRGB p_rv;
    pcl::PointXYZRGB prev_point;
    prev_point.z = 0;  
    for(int i=0; i<cloud->size(); i++)
    {
        if(cloud->points[i].x < 0.1 && cloud->points[i].x > -0.1 && cloud->points[i].y < 0.16 && cloud->points[i].y > -0.16 && cloud->points[i].z < 0.6 && cloud->points[i].z !=0) 
        {
            // 3D point in camera view
            p_rv.x = cloud->points[i].x;
            p_rv.y = cloud->points[i].y;
            p_rv.z = cloud->points[i].z;
            p_rv.r = 255;
            filtered_cloud->push_back(p_rv);
        }
    } 

    // sensor_msgs::PointCloud2 ros_pointcloud_;
    // pcl::toROSMsg(*filtered_cloud, ros_pointcloud_);
    // std::cout << "TEST2 " << std::endl;
    // //FinalPC_pub.publish(ros_pointcloud_);
    // std::cout << "TEST3 " << std::endl;

    std::cout << *filtered_cloud << std::endl;
    std::cout << *cloud << std::endl;
    pcl::io::savePCDFileASCII("filtered_cloud_test.pcd", *filtered_cloud);
    pcl::io::savePCDFileASCII("cloud_test.pcd", *cloud);

   
    FitPlanetoPC(filtered_cloud);

    
    // std::cout << "Trans 3 "<< transformation_matrix << std::endl;
    //map_pc_.reset(new pcl::PointCloud<pcl::PointXYZI>);
    //ros::spin();
    return EXIT_SUCCESS;
}