#include <mapper_emvs/pc_geometry.hpp>
#include <mapper_emvs/mapper_emvs.hpp>
#include <mapper_emvs/median_filtering.hpp>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_types.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>

#include "Mission_Management/my_msg.h"
#include <vector>

namespace EMVS {

using namespace geometry_utils;


void PCGeometry::FitPlanetoPC(PointCloud::Ptr cloud_filtered, pcl::ModelCoefficients::Ptr coefficients)
{
  //pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  
  // Create the segmentation object
  pcl::SACSegmentation<PointType> seg;
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.01);
  seg.setInputCloud (cloud_filtered);
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

void PCGeometry::PlaneRotationVector(pcl::ModelCoefficients::Ptr coefficients, geometry_utils::Transformation last_pose, Eigen::Vector4f& Quat)
{
  Eigen::Vector3f NormaltoPlane;
  NormaltoPlane[0] = coefficients->values[0]; //x-component
  NormaltoPlane[1] = coefficients->values[1]; //y-component
  NormaltoPlane[2] = coefficients->values[2]; //z-component
  
  float ax = atan2(sqrt(pow(NormaltoPlane[1],2)+pow(NormaltoPlane[2],2)),NormaltoPlane[0]);
  float ay = atan2(sqrt(pow(NormaltoPlane[2],2)+pow(NormaltoPlane[0],2)),NormaltoPlane[1]);
  
  Eigen::Vector3f RotationVector;
  Eigen::Vector3f X(1,0,0);
  Eigen::Vector3f Y(0,1,0);
  Eigen::Vector3f Z(0,0,1);
  RotationVector = Z.cross(NormaltoPlane); //TODO: check all rotations

  float RotAngle = 0;
  if (RotationVector.norm() > std::numeric_limits<float>::epsilon())
  {
    RotationVector = RotationVector/RotationVector.norm();  
    RotAngle = acos(NormaltoPlane.dot(Z)/NormaltoPlane.norm());
  }
  else
  {
    RotAngle = 0;
    RotationVector << 0, 0, 0;
  }  
  
  // Find Quaternion of the roatation vector
  Quat[0] = cos(RotAngle/2);
  Quat[1] = RotationVector[0] * sin(RotAngle/2);
  Quat[2] = RotationVector[1] * sin(RotAngle/2);
  Quat[3] = RotationVector[2] * sin(RotAngle/2);

  LOG(INFO) << "Rot Angle :" << RotAngle*(180/3.14);
  LOG(INFO) << "Rot Vector :" << RotationVector[0] << ", " << RotationVector[1] << ", " << RotationVector[2];
  
  LOG(INFO) << "Quat X Y Z W :" << Quat[1] << ", " << Quat[2] << ", " << Quat[3] << ", " << Quat[0];
  
}

void PCGeometry::ProjectPointsOnPlane(pcl::ModelCoefficients::Ptr coefficients, PointCloud::Ptr cloud_filtered, PointCloud::Ptr& holes_pos)
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
  
  Eigen::Vector3f ParallelToNormal;
  Eigen::Vector3f PerpendicularToNormal;
  
  for(int i=0; i < cloud_filtered->size(); i++)
  { 
    DispVect[0] = cloud_filtered->points[i].x - PointOnPlane[0];
    DispVect[1] = cloud_filtered->points[i].y - PointOnPlane[1];
    DispVect[2] = cloud_filtered->points[i].z - PointOnPlane[2];
    ParallelToNormal = ((DispVect.dot(NormaltoPlane))/std::pow(NormaltoPlane.norm(),2)) * NormaltoPlane;
    PerpendicularToNormal = DispVect - ParallelToNormal;
  
    EMVS::PointType P;
    P.x = PointOnPlane[0] + PerpendicularToNormal[0];
    P.y = PointOnPlane[1] + PerpendicularToNormal[1];
    P.z = PointOnPlane[2] + PerpendicularToNormal[2];
    holes_pos->push_back(P);
  }
  LOG(INFO) << "Projected Points: " << holes_pos->points[0] << holes_pos->points[1] << "Before Projection: " << cloud_filtered->points[0] << cloud_filtered->points[1];

}

void PCGeometry::PlaneinInertial(PointCloud::Ptr holes_pos, geometry_utils::Transformation last_pose, Eigen::Vector4f Quat, Eigen::Vector4f& PlaneQuatInertial, geometry_msgs::Point& point, int i)
{
  //Get Camera pose
  kindr::minimal::RotationQuaternion CamPose = last_pose.getRotation();

  Eigen::Vector4f CamPoseQuat;
  CamPoseQuat <<  CamPose.w(), CamPose.x(), CamPose.y(), CamPose.z();
  //Transform from camera frame to inertial frame 
  PlaneQuatInertial[0] = CamPoseQuat[0] * Quat[0] - CamPoseQuat[1] * Quat[1] - CamPoseQuat[2] * Quat[2] - CamPoseQuat[3] * Quat[3];  // 1
  PlaneQuatInertial[1] = CamPoseQuat[0] * Quat[1] + CamPoseQuat[1] * Quat[0] + CamPoseQuat[2] * Quat[3] - CamPoseQuat[3] * Quat[2];  // i
  PlaneQuatInertial[2] = CamPoseQuat[0] * Quat[2] - CamPoseQuat[1] * Quat[3] + CamPoseQuat[2] * Quat[0] + CamPoseQuat[3] * Quat[1];  // j
  PlaneQuatInertial[3] = CamPoseQuat[0] * Quat[3] + CamPoseQuat[1] * Quat[2] - CamPoseQuat[2] * Quat[1] + CamPoseQuat[3] * Quat[0];  // k

  Eigen::Matrix4d TransformationMat = last_pose.getTransformationMatrix();

  Eigen::Vector4d pcinCamFrame;
  Eigen::Vector4d pcinInertialFrame;


  pcinCamFrame[0] = holes_pos->points[i].x;
  pcinCamFrame[1] = holes_pos->points[i].y;
  pcinCamFrame[2] = holes_pos->points[i].z;
  pcinCamFrame[3] = 1.0;
  pcinInertialFrame = TransformationMat * pcinCamFrame;
  point.x = pcinInertialFrame[0];
  point.y = pcinInertialFrame[1];
  point.z = pcinInertialFrame[2];


   pcl::PCDWriter writer;
   writer.write ("Holes.pcd", *holes_pos, false);
  //NavigatetoPlane(pcinInertialFrame, PlaneQuatInertial);
}

void PCGeometry::PointsRegistration(PointCloud::Ptr registeredPC, PointCloud::Ptr holes_pos_intertial, geometry_msgs::Quaternion& icp_Quat)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr sourceCloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr targetCloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr finalCloud(new pcl::PointCloud<pcl::PointXYZ>);
  Eigen::Matrix4f icp_Transformation;

  //Assign both point clouds as source and target
  pcl::io::loadPCDFile<pcl::PointXYZ>("WhiteBox_pcd.pcd", *sourceCloud);
  pcl::copyPointCloud(*holes_pos_intertial, *targetCloud);
  
  // ICP object.
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> registration;
	registration.setInputSource(sourceCloud);
	registration.setInputTarget(targetCloud);
  // double RMS = EMVS::PCGeometry::computeCloudRMS(targetCloud, sourceCloud, 1.0);
  // std::cout << "RMS is :" << RMS <<  std::endl;
	registration.align(*finalCloud);
	if (registration.hasConverged())
	{
		std::cout << "ICP converged." << std::endl
				  << "The score is " << registration.getFitnessScore() << std::endl;
		std::cout << "Transformation matrix:" << std::endl;
		std::cout << registration.getFinalTransformation() << std::endl;
    icp_Transformation = registration.getFinalTransformation();

    icp_Quat.w = (std::sqrt(1 + icp_Transformation(0,0) + icp_Transformation(1,1) + icp_Transformation(2,2)))/2.0;
    icp_Quat.x = (icp_Transformation(2,1) - icp_Transformation(1,2))/(4*icp_Quat.w);
    icp_Quat.y = (icp_Transformation(0,2) - icp_Transformation(2,0))/(4*icp_Quat.w);
    icp_Quat.z = (icp_Transformation(1,0) - icp_Transformation(0,1))/(4*icp_Quat.w);
    //std::cout << icp_Quat << std::endl;
    //std::cout << *finalCloud << std::endl;
    pcl::copyPointCloud(*finalCloud, *registeredPC);
    //std::cout << "Registered Point Cloud: "<< *registeredPC << std::endl;
	}
	else std::cout << "ICP did not converge." << std::endl;

}

// void PCGeometry::BoundingBox(PointCloud::Ptr registeredPC)
// {
//   // your point cloud
//   pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud_ptr = registeredPC;
//   std::cout << "1" << std::endl;
//   // compute principal direction
//   Eigen::Vector4f centroid;
//   pcl::compute3DCentroid(*point_cloud_ptr, centroid);
//   Eigen::Matrix3f covariance;
//   computeCovarianceMatrixNormalized(*point_cloud_ptr, centroid, covariance);
//   std::cout << "2" << std::endl;
//   Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
//   Eigen::Matrix3f eigDx = eigen_solver.eigenvectors();
//   eigDx.col(2) = eigDx.col(0).cross(eigDx.col(1));
//   std::cout << "3" << std::endl;
//   // move the points to the that reference frame
//   Eigen::Matrix4f p2w(Eigen::Matrix4f::Identity());
//   p2w.block<3,3>(0,0) = eigDx.transpose();
//   p2w.block<3,1>(0,3) = -1.f * (p2w.block<3,3>(0,0) * centroid.head<3>());
//   pcl::PointCloud<pcl::PointXYZI> cPoints;
//   pcl::transformPointCloud(*point_cloud_ptr, cPoints, p2w);
//   std::cout << "4" << std::endl;
//   pcl::PointXYZI min_pt, max_pt;
//   pcl::getMinMax3D(cPoints, min_pt, max_pt);
//   const Eigen::Vector3f mean_diag = 0.5f*(max_pt.getVector3fMap() + min_pt.getVector3fMap());
//   std::cout << "5" << std::endl;
//   // final transform
//   const Eigen::Quaternionf qfinal(eigDx);
//   const Eigen::Vector3f tfinal = eigDx*mean_diag + centroid.head<3>();
//   std::cout << "6" << std::endl;
//   // draw the cloud and the box
//   pcl::visualization::PCLVisualizer viewer (new pcl::visualization::PCLVisualizer ("3D Viewer")); 
//   //pcl::visualization::PCLVisualizer viewer;
//   std::cout << "7" << std::endl;
//   viewer->addPointCloud<pcl::PointXYZI>(point_cloud_ptr);
//   std::cout << "8" << std::endl;
//   viewer->addCube(tfinal, qfinal, max_pt.x - min_pt.x, max_pt.y - min_pt.y, max_pt.z - min_pt.z);
//   std::cout << "9" << std::endl;
//   viewer->spin();
  
// }

double PCGeometry::computeCloudRMS(pcl::PointCloud<pcl::PointXYZ>::ConstPtr target, pcl::PointCloud<pcl::PointXYZ>::ConstPtr source, double max_range){
  //Reports the RMS displacement between nearest neighbors in a point cloud in mm (assuming clouds use units of meters).
  //Will ignore correspondences with distances greater than max_range (given in meters). 
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(target);

        double fitness_score = 0.0;

        std::vector<int> nn_indices (1);
        std::vector<float> nn_dists (1);

        // For each point in the source dataset
        int nr = 0;
        for (size_t i = 0; i < source->points.size (); ++i){
                //Avoid NaN points as they crash nn searches
                if(!pcl_isfinite((*source)[i].x)){
                        continue;
                }

                // Find its nearest neighbor in the target
                //tree->nearestKSearch (source->points[i], 1, nn_indices, nn_dists);
                 if ( tree->nearestKSearch (source->points[i], 1, nn_indices, nn_dists) > 0 )
                {
                  for (std::size_t ii = 0; ii < nn_indices.size (); ++ii)
                    std::cout << "    "  <<   (*target)[ nn_indices[ii] ].x 
                              << " " << (*target)[ nn_indices[ii] ].y 
                              << " " << (*target)[ nn_indices[ii] ].z 
                              << " (squared distance: " << nn_dists[ii] << ")" << std::endl;
                }
                // Deal with occlusions (incomplete targets)
                if (nn_dists[0] <= max_range*max_range){
                        //std::cout << "" <<  std::endl;
                        // Add to the fitness score
                        fitness_score += nn_dists[0];
                        nr++;
                }
        }
        
        if (nr > 0){
                
                std::cout << "RMS1 is :" << sqrt(fitness_score / nr)*1000.0 <<  std::endl;
                return sqrt(fitness_score / nr)*1000.0;
        }else{
                std::cout << "RMS2 is :" << (std::numeric_limits<double>::max ()) <<  std::endl;
                return (std::numeric_limits<double>::max ());
        }
} 
void PCGeometry::FillPCintomsgtype(PointCloud::Ptr registeredPC, geometry_msgs::Point& point, int i)
{
  point.x = registeredPC->points[i].x;
  point.y = registeredPC->points[i].y;
  point.z = registeredPC->points[i].z;
}

}
