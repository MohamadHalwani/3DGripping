#include <librealsense2/rs.hpp>
#include <algorithm>            // std::min, std::max
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>

#include <pcl/common/common_headers.h>
#include <pcl_ros/point_cloud.h>

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
#include "Mission_Management/my_msg.h"

#include <geometry_msgs/PoseStamped.h>
#include "geometry_msgs/Pose.h"
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>

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

pcl::PointCloud<pcl::PointXYZI>::Ptr d345_cloud (new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PointXYZI>::Ptr DAVIS_cloud (new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PointXYZI>::Ptr d345_cloud_Inertial (new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PointXYZI>::Ptr d345_cloud_TCP (new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PointXYZI>::Ptr DAVIS_cloud_TCP (new pcl::PointCloud<pcl::PointXYZI>);
pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
Mission_Management::my_msg Holes;
ros::Subscriber holes_sub;
using namespace cv;



void FitPlanetoPC(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
  
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZI> seg;
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.01);
  seg.setInputCloud (cloud);
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
void ReadpcdFiles()
{
    pcl::io::loadPCDFile<pcl::PointXYZI> ("/home/ubuntu/emvs_ws/rosbagsexp/Orientation3.pcd", *d345_cloud);
    std::cout << "Loaded "
            << d345_cloud->width * d345_cloud->height
            << " data points with the following fields: "
            << std::endl;
    for (const auto& point: *d345_cloud);
    // std::cout << "    " << point.x
    //           << " "    << point.y
    //           << " "    << point.z << std::endl;
}

void ReadpcdFiles2()
{
    pcl::io::loadPCDFile<pcl::PointXYZI> ("/home/ubuntu/emvs_ws/PC_CAMframe.pcd", *DAVIS_cloud);
    std::cout << "Loaded "
            << DAVIS_cloud->width * DAVIS_cloud->height
            << " data points with the following fields: "
            << std::endl;
    for (const auto& point: *DAVIS_cloud);
    // std::cout << "    " << point.x
    //           << " "    << point.y
    //           << " "    << point.z << std::endl;
}

void DistancetoPlane(pcl::ModelCoefficients::Ptr coefficients, Mission_Management::my_msg holes)
{ 
  Eigen::Vector3f NormaltoPlane;
  NormaltoPlane[0] = coefficients->values[0]; //x-component
  NormaltoPlane[1] = coefficients->values[1]; //y-component
  NormaltoPlane[2] = coefficients->values[2]; //z-component

  Eigen::Vector3f DispVect;

  Eigen::Vector3f PointOnPlane;
  PointOnPlane[0] = coefficients->values[0];
  PointOnPlane[1] = coefficients->values[1];
  PointOnPlane[2] = -coefficients->values[3] - (std::pow(PointOnPlane[0],2) + std::pow(PointOnPlane[1],2)) / coefficients->values[2]; //TODO: zero divisiin check
  
  //Eigen::Vector3f Distance;
  float Distance;
  //Eigen::Vector3f PerpendicularToNormal;
  
  for(int i=0; i < holes.points.size(); i++)
  { 
    DispVect[0] = holes.points[i].x - PointOnPlane[0];
    DispVect[1] = holes.points[i].y - PointOnPlane[1];
    DispVect[2] = holes.points[i].z - PointOnPlane[2];
    Distance = std::abs(DispVect.dot(NormaltoPlane)); ///std::pow(NormaltoPlane.norm(),2)) * NormaltoPlane;
    // PerpendicularToNormal = DispVect - ParallelToNormal;
    std::cout << "Distance from Point" << i << " to plane is: " << Distance << " " << std::endl;

    // EMVS::PointType P;
    // P.x = PointOnPlane[0] + PerpendicularToNormal[0];
    // P.y = PointOnPlane[1] + PerpendicularToNormal[1];
    // P.z = PointOnPlane[2] + PerpendicularToNormal[2];
    // holes_pos->push_back(P);
  }
  
}
void HoleCallback(Mission_Management::my_msg msg)
{
    Holes = msg;
    ros::shutdown();
}

void TransformtoMatrixRot(geometry_msgs::TransformStamped transformStamped)
{   
    transformation_matrix(0,3) = transformStamped.transform.translation.x;
    transformation_matrix(1,3) = transformStamped.transform.translation.y;
    transformation_matrix(2,3) = transformStamped.transform.translation.z;
    transformation_matrix(3,3) = 1;
    Eigen::Quaterniond q;
    q.x() = transformStamped.transform.rotation.x;
    q.y() = transformStamped.transform.rotation.y;
    q.z() = transformStamped.transform.rotation.z;
    q.w() = transformStamped.transform.rotation.w;
    Eigen::Matrix3d Rotation = q.normalized().toRotationMatrix();
    transformation_matrix.block<3,3>(0,0) = Rotation;
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "listener");
    ros::NodeHandle n;

    // Subscribers 
    holes_sub = n.subscribe("/hole_pos", 1, HoleCallback);

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    geometry_msgs::TransformStamped Trans_basetoCAMdepthOpticFrame;
    geometry_msgs::TransformStamped Trans_CAMdepth_to_TCP;
    geometry_msgs::TransformStamped Trans_DAVIS_to_TCP;
    Trans_basetoCAMdepthOpticFrame = tfBuffer.lookupTransform("base", "camera_color_frame", ros::Time(0), ros::Duration(25.0));
    //Trans_CAMdepth_to_TCP = tfBuffer.lookupTransform("camera_depth_optical_frame", "davis", ros::Time(0), ros::Duration(25.0));
    //Trans_DAVIS_to_TCP = tfBuffer.lookupTransform("TCP", "davis", ros::Time(0), ros::Duration(25.0));
    TransformtoMatrixRot(Trans_basetoCAMdepthOpticFrame);
    std::cout << "Transformation Matrix  = "<< transformation_matrix << std::endl;

    ros::spin();

    ReadpcdFiles();
    //ReadpcdFiles2();

    TransformtoMatrixRot(Trans_basetoCAMdepthOpticFrame);
    std::cout << "Transformation Matrix  = "<< transformation_matrix << std::endl;

    pcl::transformPointCloud(*d345_cloud, *d345_cloud_Inertial, transformation_matrix);
    
    
    
    // TransformtoMatrixRot(Trans_CAMdepth_to_TCP);
    // pcl::transformPointCloud(*DAVIS_cloud, *DAVIS_cloud_TCP, transformation_matrix);
    
    // TransformtoMatrixRot(Trans_DAVIS_to_TCP);
    // pcl::transformPointCloud(*DAVIS_cloud, *DAVIS_cloud_TCP, transformation_matrix);
    

    pcl::PCDWriter writer;
    writer.write ("d345_PC_Inertial.pcd", *d345_cloud_Inertial, false);
    //writer.write ("DAVIS_PC_d345.pcd", *DAVIS_cloud_TCP, false);
    

    FitPlanetoPC(d345_cloud_Inertial);
    DistancetoPlane(coefficients, Holes);
    
    //std::cout << "Trans 2"<< transformation_matrix << std::endl;
    //pcl::io::savePCDFileASCII("filtered_cloud_Inertial.pcd", *filtered_cloud_Inertial);
    // Fit Plane
    //

    
    //std::cout << "Trans 3 "<< transformation_matrix << std::endl;
    waitKey(0);
    return EXIT_SUCCESS;
}