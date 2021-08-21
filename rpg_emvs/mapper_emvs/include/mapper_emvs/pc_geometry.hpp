#pragma once

#include <cartesian3dgrid/cartesian3dgrid.h>
#include <mapper_emvs/geometry_utils.hpp>
#include <mapper_emvs/trajectory.hpp>
#include <mapper_emvs/depth_vector.hpp>

#include <image_geometry/pinhole_camera_model.h>
#include <dvs_msgs/Event.h>
#include <geometry_msgs/PoseStamped.h>
#include "geometry_msgs/Pose.h"

#include <mapper_emvs/geometry_utils.hpp>
#include <Eigen/Geometry> 

#include <pcl/common/common_headers.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/pcl_base.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/extract_indices.h>
#include<math.h> 

namespace EMVS {

typedef pcl::PointXYZI PointType;
typedef pcl::PointCloud<PointType> PointCloud;

class PCGeometry
{
public:

  void FitPlanetoPC(PointCloud::Ptr cloud_filtered, pcl::ModelCoefficients::Ptr coefficients);
  void PlaneRotationVector(pcl::ModelCoefficients::Ptr coefficients, geometry_utils::Transformation last_pose, Eigen::Vector4f& Quat);
  //void PlaneinInertial(PointCloud::Ptr cloud_UNfiltered, PointCloud::Ptr cloud_filtered, geometry_utils::Transformation last_pose, Eigen::Vector4f Quat, Eigen::Vector4d& pcinInertialFrame, Eigen::Vector4f& PlaneQuatInertial, Eigen::Vector4d& UNfilteredPCInertial);
  void ProjectPointsOnPlane(pcl::ModelCoefficients::Ptr coefficients, PointCloud::Ptr cloud_filtered, PointCloud::Ptr& holes_pos);
  void PlaneinInertial(PointCloud::Ptr cloud_filtered, geometry_utils::Transformation last_pose, Eigen::Vector4f Quat, Eigen::Vector4f& PlaneQuatInertial, geometry_msgs::Point& point, int i);
  //void NavigatetoPlane(Eigen::Vector4d pc_, Eigen::Vector4f PlaneQuatInertial);
  void PointsRegistration(PointCloud::Ptr registeredPC, PointCloud::Ptr holes_pos_intertial, geometry_msgs::Quaternion& icp_Quat);
  void BoundingBox(PointCloud::Ptr registeredPC);
  void FillPCintomsgtype(PointCloud::Ptr registeredPC, geometry_msgs::Point& point, int i);
  double computeCloudRMS(pcl::PointCloud<pcl::PointXYZ>::ConstPtr target, pcl::PointCloud<pcl::PointXYZ>::ConstPtr source, double max_range);
private:

};

}