#include <mapper_emvs/data_loading.hpp>
#include <mapper_emvs/mapper_emvs.hpp>
#include <mapper_emvs/pc_geometry.hpp>

#include <image_geometry/pinhole_camera_model.h>

#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <pcl/io/pcd_io.h>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include <chrono>
#include <iostream>
#include <thread>

#include <ros/ros.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/PoseStamped.h>
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include <tf2/LinearMath/Quaternion.h>

#include "Mission_Management/my_msg.h"
#include <vector>

#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include <std_msgs/Float64MultiArray.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>

#include <dynamic_reconfigure/server.h>
#include <mapper_emvs/EMVSCfgConfig.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/registration/incremental_registration.h>
#include <pcl/registration/icp.h>

#include <std_srvs/SetBool.h>



// Input parameters
DEFINE_string(bag_filename, "input.bag", "Path to the rosbag");
//DEFINE_string(event_topic, "/filtered_events", "Name of the event topic (default: /dvs/events)");
//DEFINE_string(event_topic, "/dvs/events", "Name of the event topic (default: /dvs/events)");
DEFINE_string(event_topic, "/dvs/corners_FA", "Name of the event topic (default: /dvs/events)");
DEFINE_string(pose_topic, "/dvs/pose", "Name of the pose topic (default: /optitrack/davis)");
DEFINE_string(camera_info_topic, "/dvs/camera_info", "Name of the camera info topic (default: /dvs/camera_info)");
DEFINE_double(start_time_s, 0.0, "Start time in seconds (default: 0.0)");
DEFINE_double(stop_time_s, 1000.0, "Stop time in seconds (default: 1000.0)");

// Disparity Space Image (DSI) parameters. Section 5.2 in the IJCV paper.
DEFINE_int32(dimX, 0, "X dimension of the voxel grid (if 0, will use the X dim of the event camera) (default: 0)");
DEFINE_int32(dimY, 0, "Y dimension of the voxel grid (if 0, will use the Y dim of the event camera) (default: 0)");
DEFINE_int32(dimZ, 100, "Z dimension of the voxel grid (default: 100) must be <= 256");
DEFINE_double(fov_deg, 0.0, "Field of view of the DSI, in degrees (if < 10, will use the FoV of the event camera) (default: 0.0)");
DEFINE_double(min_depth, 0.3, "Min depth, in meters (default: 0.3)");
DEFINE_double(max_depth, 5.0, "Max depth, in meters (default: 5.0)");

// Depth map parameters (selection and noise removal). Section 5.2.3 in the IJCV paper.
DEFINE_int32(adaptive_threshold_kernel_size, 5, "Size of the Gaussian kernel used for adaptive thresholding. (default: 5)");
DEFINE_double(adaptive_threshold_c, 5., "A value in [0, 255]. The smaller the noisier and more dense reconstruction (default: 5.)");
DEFINE_int32(median_filter_size, 5, "Size of the median filter used to clean the depth map. (default: 5)");

// Point cloud parameters (noise removal). Section 5.2.4 in the IJCV paper.
DEFINE_double(radius_search, 0.05, "Size of the radius filter. (default: 0.05)");
DEFINE_int32(min_num_neighbors, 3, "Minimum number of points for the radius filter. (default: 3)");

//Voxel Grid Parameters (size of the voxel grid to filter the point clouds after generating them)
DEFINE_double(leaf_size_x, 0.1, "size of the voxel grid_x. (default: 0.1)");
DEFINE_double(leaf_size_y, 0.1, "size of the voxel grid_y. (default: 0.1)");
DEFINE_double(leaf_size_z, 0.1, "size of the voxel grid_z. (default: 0.1)");

class DepthEstimator
{
  public:
    DepthEstimator(const std::string& camero_info_topic, const std::string& event_topic, const std::string& pose_topic, float distance_thresh,
                   float dim_X, float dim_Y, float dim_Z, float min_depth, float max_depth, float fov_deg,
                   float adaptive_threshold_kernel_size, float adaptive_threshold_c, float median_filter_size,
                   float radius_search, float min_num_neighbors)
    {
      
      start_EMVS_service = ros_node_.advertiseService("startEMVS", &DepthEstimator::start_EMVS, this);

      dsi_shape_ = EMVS::ShapeDSI(dim_X, dim_Y, dim_Z,
                                min_depth, max_depth,
                                fov_deg);

      opts_depth_map_.adaptive_threshold_kernel_size_ = adaptive_threshold_kernel_size;
      opts_depth_map_.adaptive_threshold_c_ = adaptive_threshold_c;
      opts_depth_map_.median_filter_size_ = median_filter_size;

      opts_pc_.radius_search_ = radius_search;
      opts_pc_.min_num_neighbors_ = min_num_neighbors;   
      this->pc_.reset(new EMVS::PointCloud);
      this->map_pc_.reset(new EMVS::PointCloud);
      this->global_pc_.reset(new EMVS::PointCloud);
      this->ros_pointcloud_.reset(new sensor_msgs::PointCloud2);
      this->ros_voxelcloud_.reset(new sensor_msgs::PointCloud2);
      this->ros_finalcloud_.reset(new sensor_msgs::PointCloud2);
      this->icp_.reset(new pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI>);
      this->icp_->setMaxCorrespondenceDistance (0.05);
      this->icp_->setMaximumIterations (50);
      this->iicp_.setRegistration(this->icp_);

      update_distance_ = distance_thresh;

      f_ = boost::bind(&DepthEstimator::parameter_callback, this, _1, _2);
      server_.setCallback(f_);
    }
   
  private:
      ros::NodeHandle ros_node_;

      std::vector<dvs_msgs::Event> events_;
      image_geometry::PinholeCameraModel cam_;

      bool cam_initialized = false;
      bool pose_initialized_ = false;

      float update_distance_;

      float leaf_size_x;
      float leaf_size_y;
      float leaf_size_z;

      bool apply_color_mask = false;

      std::vector<cv::Point> image_contour;

      std::map<ros::Time, geometry_utils::Transformation> poses_;
      std::vector<geometry_msgs::PoseStamped> pose_list_;
      std::vector<ros::Time> pose_timestaps_;

      geometry_msgs::PoseStamped processed_pose_;
      

      ros::Publisher depthmap_publisher_ = ros_node_.advertise<sensor_msgs::Image>("depth_image", 1);
      ros::Publisher pointcloud_publisher_ = ros_node_.advertise<sensor_msgs::PointCloud2>("point_cloud", 1);
      ros::Publisher cmd_pos_pub = ros_node_.advertise<geometry_msgs::Pose>("/ur_cmd_pose", 1);
      ros::Publisher FinalPC_pub = ros_node_.advertise<sensor_msgs::PointCloud2>("ICP_pointcloud", 1);
      ros::Publisher depth_map_pub = ros_node_.advertise<cv_bridge::CvImage>("/depth_map", 1);
      ros::Publisher annotated_image_publisher = ros_node_.advertise<sensor_msgs::Image>("/annotated_image", 1);
    
      ros::Publisher ObjectCentroid_pub = ros_node_.advertise<geometry_msgs::Point>("/hole_pos", 1);
      ros::Publisher ICP_Transformation_pub = ros_node_.advertise<geometry_msgs::Quaternion>("/icp_transformation", 1);
      ros::Publisher ICP_Quat_pub = ros_node_.advertise<geometry_msgs::Quaternion>("/icp_Quat", 1);
      
      ros::Subscriber event_subs_; 
      ros::Subscriber cam_info_subs_;
      ros::Subscriber pose_sub;
      ros::Subscriber image_sub;


      //ROS Service
      ros::ServiceServer start_EMVS_service;

      //ROS message
      Mission_Management::my_msg PCInertial;
      Mission_Management::my_msg registered_PCInertial;
  
      LinearTrajectory trajectory_;

      EMVS::ShapeDSI dsi_shape_;
      EMVS::MapperEMVS mapper_;
      EMVS::PCGeometry geo_;
      cv::Mat depth_map_, confidence_map_, semidense_mask_, depth_map_255_;
      EMVS::OptionsDepthMap opts_depth_map_;
      sensor_msgs::Image ros_depth_map_;
      cv_bridge::CvImage depth_map_bridge_;

      EMVS::OptionsPointCloud opts_pc_;
      EMVS::PointCloud::Ptr pc_;
      EMVS::PointCloud::Ptr map_pc_;
      EMVS::PointCloud::Ptr global_pc_;
      sensor_msgs::PointCloud2::Ptr ros_pointcloud_;
      sensor_msgs::PointCloud2::Ptr ros_voxelcloud_;
      sensor_msgs::PointCloud2::Ptr ros_finalcloud_;
      pcl::IterativeClosestPoint<EMVS::PointType, EMVS::PointType>::Ptr icp_;
      pcl::registration::IncrementalRegistration<pcl::PointXYZI> iicp_;
     
      float X_corner;
      float Y_corner;

      Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity();

      dynamic_reconfigure::Server<mapper_emvs::EMVSCfgConfig> server_;
      dynamic_reconfigure::Server<mapper_emvs::EMVSCfgConfig>::CallbackType f_;

      struct Point{
        float x;
        float y;
        float z;
      };

      
      
      bool start_EMVS(std_srvs::SetBool::Request &msg, std_srvs::SetBool::Response &msg_output)
      { 
        if(msg.data == 1)
        {
          LOG(INFO) << "Start EMVS Service";
          //event_subs_ = ros_node_.subscribe("/dvs/events", 1, &DepthEstimator::EventsCallback, this);
          //event_subs_ = ros_node_.subscribe("/filtered_events", 1, &DepthEstimator::EventsCallback, this);
          event_subs_ = ros_node_.subscribe("/dvs/corners_FA", 1, &DepthEstimator::EventsCallback, this);
          cam_info_subs_ = ros_node_.subscribe("/dvs/camera_info", 1, &DepthEstimator::CamInfoCallback, this);
          pose_sub = ros_node_.subscribe("/dvs/pose", 1, &DepthEstimator::PoseCallback, this);
          //image_sub = ros_node_.subscribe("/dvs/image_raw", 1, &DepthEstimator::ImageCallback, this);
          msg_output.success = true;
        }
        else 
        {
          LOG(INFO) << "EMVS Service Stopped";
          event_subs_.shutdown();
          cam_info_subs_.shutdown();
          pose_sub.shutdown();
          image_sub.shutdown();
          pose_initialized_ = false;
          msg_output.success = true;
        }
        return 1;
      } 

      void EventsCallback(const dvs_msgs::EventArray::ConstPtr &event_stream)
      {
        //ROS_INFO("Event Stream Received");
        for (uint i = 0 ; i < event_stream->events.size() ; i++)
        {
          if(event_stream->events[i].x < 165 || event_stream->events[i].x > 175 || event_stream->events[i].y < 80 || event_stream->events[i].y > 90 || event_stream->events[i].x < 90 || event_stream->events[i].x > 95 || event_stream->events[i].y < 96 ||event_stream->events[i].y > 103 )
          {
            if (this->apply_color_mask)
            { 
              if (this->image_contour.size() > 0)
              {
                if (cv::pointPolygonTest(this->image_contour, cv::Point2f(event_stream->events[i].x, event_stream->events[i].y), false))
                {
                  this->events_.push_back(event_stream->events[i]);
                }
              }
            }
            else
            {
              this->events_.push_back(event_stream->events[i]);
            }
          }
        } 
      }

      void ImageCallback(const sensor_msgs::ImageConstPtr& input_image)
      {
        ROS_INFO("Image Received");
        cv_bridge::CvImagePtr input_image_bridge = cv_bridge::toCvCopy(input_image, input_image->encoding);

        cv::Mat cv_image = input_image_bridge->image;

        //convert to HSV
        cv::Mat hsv_image;
        cv::cvtColor(cv_image, hsv_image, CV_RGB2HSV);


        //Threshold RGB image
        cv::Mat threshold_image;
        cv::inRange(hsv_image, cv::Scalar(55, 100, 30), cv::Scalar(80, 255, 200), threshold_image);

        //Contour detection
        std::vector<std::vector<cv::Point>> contours;
        cv::Mat contour_erode_kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
	      cv::erode(threshold_image, threshold_image, contour_erode_kernel);
	      cv::findContours(threshold_image, contours, cv::RETR_CCOMP, cv::CHAIN_APPROX_SIMPLE);

        if (contours.size() > 0)
        {
          //find largest contour
          int largest_ctr_size =0;
          int largest_ctr_idx = 0;
          for (int j=0 ; j<contours.size(); j++)
          {
            if(contours[j].size() > largest_ctr_size)
            {
              largest_ctr_size = contours[j].size();
              largest_ctr_idx = j;
            }
          }

          this->image_contour = contours[largest_ctr_idx];
    
          cv::drawContours(cv_image, contours, largest_ctr_idx, cv::Scalar(255,0,0));

          input_image_bridge->image = cv_image;
          sensor_msgs::Image output_image;
          input_image_bridge->toImageMsg(output_image);
          this->annotated_image_publisher.publish(output_image);
        }
      }

      void CamInfoCallback(const sensor_msgs::CameraInfo::ConstPtr &camera_info)
      {
        ROS_INFO("Camera Info Received");
        sensor_msgs::CameraInfo cam_info_ = *camera_info;
        cam_info_.width = 240;//346; //240;
        cam_info_.height = 180;//260; //180;
        this->cam_.fromCameraInfo(cam_info_);
        this->cam_initialized = true;

        //initialize mapper
        new(&this->mapper_) EMVS::MapperEMVS(this->cam_, this->dsi_shape_);

        //Disable camera info callbacks
        this->cam_info_subs_.shutdown();        
      }

      void PoseCallback(const geometry_msgs::PoseStamped::ConstPtr &camera_pose)
      {
        //ROS_INFO("Pose Received");
        if (!pose_initialized_)
        {
          this->processed_pose_ = *camera_pose;
          pose_initialized_ = true;
        }

        this->pose_list_.push_back(*camera_pose);
        this->pose_timestaps_.push_back(camera_pose->header.stamp);
    
        const Eigen::Vector3d position(camera_pose->pose.position.x,
                                      camera_pose->pose.position.y,
                                      camera_pose->pose.position.z);
        const Eigen::Quaterniond quat(camera_pose->pose.orientation.w,
                                      camera_pose->pose.orientation.x,
                                      camera_pose->pose.orientation.y,
                                      camera_pose->pose.orientation.z);

        this->transformation_matrix.block(0,0,3,3) = quat.normalized().toRotationMatrix();
        this->transformation_matrix.block(0,3,3,1) = position;
        

        geometry_utils::Transformation T(position, quat);
        poses_.insert(std::pair<ros::Time, geometry_utils::Transformation>(camera_pose->header.stamp, T));

        double distance_to_last_keyframe = std::sqrt( std::pow(camera_pose->pose.position.x - this->processed_pose_.pose.position.x, 2) +
                                                      std::pow(camera_pose->pose.position.y - this->processed_pose_.pose.position.y, 2) +
                                                      std::pow(camera_pose->pose.position.z - this->processed_pose_.pose.position.z, 2));
        //ROS_INFO("Distance to last KEy Frame:", distance_to_last_keyframe);
        if (this->processed_pose_.header.stamp.toSec() > camera_pose->header.stamp.toSec())
        {
          this->processed_pose_ = *camera_pose;
          this->poses_.clear();
          this->pose_list_.clear();
          this->pose_timestaps_.clear();
          this->events_.clear();
          this->iicp_.reset();
          this->global_pc_.reset(new EMVS::PointCloud);
        }
        else if ((distance_to_last_keyframe > update_distance_) && this->cam_initialized && (this->pose_list_.size() > 2))
        {
          this->ProcessEvents();
          this->processed_pose_ = *camera_pose;
          poses_.insert(std::pair<ros::Time, geometry_utils::Transformation>(camera_pose->header.stamp, T));
        }
      }

      void ProcessEvents()
      {
        ROS_INFO("Processing Events ...");
        this->trajectory_ = LinearTrajectory(this->poses_);
        
        // Set the position of the reference view in the middle of the trajectory
        geometry_utils::Transformation T1_;
        ros::Time t1_;
        this->trajectory_.getLastControlPose(&T1_, &t1_);

        // geometry_utils::Transformation T0_, T1_;
        // ros::Time t0_, t1_;
        // trajectory.getFirstControlPose(&T0_, &t0_);
        // trajectory.getLastControlPose(&T1_, &t1_);
        // geometry_utils::Transformation T_w_rv;
        // trajectory.getPoseAt(ros::Time(0.5 * (t0_.toSec() + t1_.toSec())), T_w_rv);
        // geometry_utils::Transformation T_rv_w = T_w_rv.inverse();

        ROS_INFO("Evaluating DSI ...");
        //evaluate DSI
        std::chrono::high_resolution_clock::time_point t_start_dsi = std::chrono::high_resolution_clock::now();
        this->mapper_.evaluateDSI(this->events_, this->trajectory_ , T1_.inverse());
        std::chrono::high_resolution_clock::time_point t_end_dsi = std::chrono::high_resolution_clock::now();
        auto duration_dsi = std::chrono::duration_cast<std::chrono::milliseconds>(t_end_dsi - t_start_dsi ).count();
        LOG(INFO) << "Time to evaluate DSI: " << duration_dsi << " milliseconds";
        LOG(INFO) << "Number of events processed: " << this->events_.size() << " events";
        LOG(INFO) << "Number of events processed per second: " << static_cast<float>(events_.size()) / (1000.f * static_cast<float>(duration_dsi)) << " Mev/s";

        //this->mapper_.evaluateDSI(this->events_, this->trajectory_ , T_rv_w);

        // Write the DSI (3D voxel grid) to disk
        this->mapper_.dsi_.writeGridNpy("dsi.npy");
  
        //evaluate depth map
        this->mapper_.getDepthMapFromDSI(this->depth_map_, this->confidence_map_, this->semidense_mask_, this->opts_depth_map_);
        
        cv_bridge::CvImage ros_depth_map;
        ros_depth_map.encoding = sensor_msgs::image_encodings::TYPE_32FC1; // Or whatever
        ros_depth_map.image    = this->depth_map_; // Your cv::Mat
        //LOG(INFO) << "ROS Depth Map: " << ros_depth_map.image;  
        
        this->depth_map_255_ = (this->depth_map_ - this->dsi_shape_.min_depth_) * (255.0 / (this->dsi_shape_.max_depth_ - this->dsi_shape_.min_depth_));
        this->depth_map_255_.convertTo(this->depth_map_255_, CV_8U);
        cv::Mat depthmap_color;
        cv::applyColorMap(this->depth_map_255_, depthmap_color, cv::COLORMAP_RAINBOW);
        cv::Mat depth_on_canvas = cv::Mat(depthmap_color.rows, depthmap_color.cols, CV_8UC3, cv::Scalar(1,1,1)*255);
        depthmap_color.copyTo(depth_on_canvas, this->semidense_mask_);
        cv::imwrite("depth_map.png", depth_on_canvas);
    
        ROS_INFO("Publishing Depth Map ...");
        //publish depth map
        this->ros_depth_map_.header.seq = 1; // user defined counter
        this->ros_depth_map_.header.stamp = ros::Time::now(); // time
        this->depth_map_bridge_ = cv_bridge::CvImage(this->ros_depth_map_.header, sensor_msgs::image_encodings::RGB8, depth_on_canvas);

        this->depth_map_bridge_.toImageMsg(this->ros_depth_map_); // from cv_bridge to sensor_msgs::Image

        this->depth_map_pub.publish(this->ros_depth_map_);

        ROS_INFO("converting to Point Cloud ...");    
        EMVS::PointCloud::Ptr pc_2 (new EMVS::PointCloud);   // point cloud in camera frame from Triangulation  
        EMVS::PointCloud::Ptr mappc_2 (new EMVS::PointCloud); // point cloud in Inertial frame from Triangulation        
        this->mapper_.getPointcloud(this->depth_map_, this->semidense_mask_, this->opts_pc_, this->pc_);
        this->mapper_.getIntersectionPointCloudFromDSI(this->opts_depth_map_, this->opts_pc_, pc_2);
      
        pcl::transformPointCloud(*this->pc_, *this->map_pc_, this->transformation_matrix);
        pcl::transformPointCloud(*pc_2, *mappc_2, this->transformation_matrix);
        pcl::PCDWriter writer;
        writer.write ("Holes_inertial_Intersection.pcd", *mappc_2, false);
        writer.write ("Holes_CAM_Intersection.pcd", *pc_2, false);
        writer.write ("Holes_inertial_EMVS.pcd", *this->map_pc_, false);
        writer.write ("Holes_CAM_EMVS.pcd", *this->pc_, false);
        // this->iicp_.registerCloud(this->map_pc_);
        // pcl::PointCloud<pcl::PointXYZI>::Ptr tmp (new pcl::PointCloud<pcl::PointXYZI>);
        // pcl::transformPointCloud (*this->map_pc_, *tmp, this->iicp_.getAbsoluteTransform ());
        // *this->global_pc_ += *tmp;

        // this->global_pc_->header.frame_id = "camera";
        this->pc_->header.frame_id = "camera";
        this->map_pc_->header.frame_id = "map";
        pcl::toROSMsg(*this->map_pc_, *this->ros_pointcloud_);
        //this->pointcloud_publisher_.publish(*this->ros_pointcloud_);

        pc_2->header.frame_id = "camera";
        mappc_2->header.frame_id = "map";
        //pcl::toROSMsg(*this->map_pc_, *this->ros_pointcloud_);
        //this->FinalPC_pub.publish(*this->ros_pointcloud_);

        EMVS::PointCloud::Ptr cloud_filtered (new EMVS::PointCloud); // point cloud "Camera Frame" after filtering using Voxels
        //this->mapper_.PCtoVoxelGrid(this->pc_, cloud_filtered, leaf_size_x, leaf_size_y, leaf_size_z);
        this->mapper_.PCtoVoxelGrid(pc_2, cloud_filtered, leaf_size_x, leaf_size_y, leaf_size_z);
        writer.write ("PC_CAMframe.pcd", *cloud_filtered, false);
        geometry_utils::Transformation last_pose = poses_.at(t1_);
        pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients); // coefficients of plane equation
        //this->geo_.FitPlanetoPC(cloud_filtered, coefficients);
        
        Eigen::Vector4f Quat;
        //this->geo_.PlaneRotationVector(coefficients, last_pose, Quat);
        
        Eigen::Vector4f PlaneQuatInertial; 
        geometry_msgs::Point point;
    
        EMVS::PointCloud::Ptr holes_pos (new EMVS::PointCloud); //Point Cloud "Camera Frame" after Projection of plane
        //geo_.ProjectPointsOnPlane(coefficients, cloud_filtered, holes_pos);
        
        for(int i=0; i < holes_pos->size(); i++)
        {
          this->geo_.PlaneinInertial(holes_pos, last_pose, Quat, PlaneQuatInertial, point, i);
          PCInertial.points.push_back(point);
        }

        //LOG(INFO) << "Quat W X Y Z in inertial frame :" << PlaneQuatInertial;
        //LOG(INFO) << "PCINERTAIL Points:" << PCInertial;

        geometry_msgs::Quaternion PlaneQuat;
        PlaneQuat.x = PlaneQuatInertial[1];
        PlaneQuat.y = PlaneQuatInertial[2];
        PlaneQuat.z = PlaneQuatInertial[3];
        PlaneQuat.w = PlaneQuatInertial[0];

        //LOG(INFO) << " Pose message :" <<  PlaneQuat.x << PlaneQuat.y << PlaneQuat.z << PlaneQuat.w;

        //Publish hole (voxel) position, pc position and plane orientation TODO: Correct the points to be in ROS compatibale type
        EMVS::PointCloud::Ptr holes_pos_intertial (new EMVS::PointCloud); //Point Cloud "Inertial Frame" 
        //pcl::transformPointCloud(*holes_pos, *holes_pos_intertial, this->transformation_matrix);
        pcl::transformPointCloud(*cloud_filtered, *holes_pos_intertial, this->transformation_matrix);
        holes_pos_intertial->header.frame_id = "map";
        //holes_pos->header.frame_id = "camera";
        //cloud_filtered->header.frame_id = "camera";
        pcl::toROSMsg(*holes_pos_intertial, *this->ros_voxelcloud_);
        // this->FinalPC_pub.publish(*this->ros_voxelcloud_);
        // pcl::PCDWriter writer;
        writer.write ("Voxel_holes_inertial_.pcd", *holes_pos_intertial, false);
        this->pointcloud_publisher_.publish(*this->ros_voxelcloud_);
        
        // for(int i=0; i < holes_pos_intertial->size(); i++)
        // {
        //   this->geo_.FillPCintomsgtype(holes_pos_intertial, point, i);
        //   this->registered_PCInertial.points.push_back(point);
        // }
        // LOG(INFO) << "rPoints in Inertial Frame:" << registered_PCInertial;

        // // // Point Cloud Registration
        EMVS::PointCloud::Ptr registeredPC (new EMVS::PointCloud); // Registered Point Cloud in plate 
        geometry_msgs::Quaternion icp_Quat;
        this->geo_.PointsRegistration(registeredPC, holes_pos_intertial, icp_Quat);
        //this->geo_.BoundingBox(registeredPC);
        registeredPC->header.frame_id = 'map';
        pcl::toROSMsg(*registeredPC, *this->ros_finalcloud_);
        writer.write ("Final_Box_inertial_.pcd", *registeredPC, false);
        this->FinalPC_pub.publish(*this->ros_finalcloud_);
        
        for(int i=0; i < registeredPC->size(); i++)
        {
          this->geo_.FillPCintomsgtype(registeredPC, point, i);
          this->registered_PCInertial.points.push_back(point);
        }

        // LOG(INFO) << " after point registration";
        LOG(INFO) << "registered_PCINERTAIL Points:" << registered_PCInertial;

        geometry_msgs::Point Object_Centroid;
        for(int i=0; i < registered_PCInertial.points.size(); i++)
        {
          Object_Centroid.x += registered_PCInertial.points[i].x/registered_PCInertial.points.size();
          Object_Centroid.y += registered_PCInertial.points[i].y/registered_PCInertial.points.size();
          Object_Centroid.z += registered_PCInertial.points[i].z/registered_PCInertial.points.size();
        }
        LOG(INFO) << "Object Centroid:" << Object_Centroid;

        this->ObjectCentroid_pub.publish(Object_Centroid);
        sleep(1);
        //this->ICP_Quat_pub.publish(icp_Quat);
        
        
        this->events_.clear();
        this->poses_.clear();   
        this->pose_list_.clear();
        this->pose_timestaps_.clear();
        this->PCInertial.points.clear();  
        this->registered_PCInertial.points.clear();      
        event_subs_.shutdown();
        cam_info_subs_.shutdown();
        pose_sub.shutdown();
        image_sub.shutdown();
      }

      void parameter_callback(mapper_emvs::EMVSCfgConfig &config, uint32_t level){
        this->dsi_shape_ = EMVS::ShapeDSI(config.dimX, config.dimY, config.dimZ,
                                    config.min_depth, config.max_depth,
                                    config.fov_deg);

        if(this->cam_initialized)
        {
          new(&this->mapper_) EMVS::MapperEMVS(this->cam_, this->dsi_shape_);
        }

        this->opts_depth_map_.adaptive_threshold_kernel_size_ = config.adaptive_threshold_kernel_size;
        this->opts_depth_map_.adaptive_threshold_c_ = config.adaptive_threshold_c;
        this->opts_depth_map_.median_filter_size_ = config.median_filter_size;

        this->update_distance_ = config.update_distance;
        ROS_INFO("update distance %f:", this->update_distance_);
        this->opts_pc_.radius_search_ = config.radius_search;
        this->opts_pc_.min_num_neighbors_ = config.min_num_neighbors;

        this->leaf_size_x = config.leaf_size_x;
        this->leaf_size_y = config.leaf_size_y;
        this->leaf_size_z = config.leaf_size_z;
        ROS_INFO("leaf size x %f:", this->leaf_size_x);

        //this->apply_color_mask = config.color_masking;
      }
    
};

/*
 * Load a set of events and poses from a rosbag,
 * compute the disparity space image (DSI),
 * extract a depth map (and point cloud) from the DSI,
 * and save to disk.
 */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "depth_estimation");
	ros::NodeHandle nh_;


  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InstallFailureSignalHandler();
  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;

  DepthEstimator depth_estimator(FLAGS_camera_info_topic, FLAGS_event_topic, FLAGS_pose_topic, 0.2,
                                FLAGS_dimX, FLAGS_dimY, FLAGS_dimZ, FLAGS_min_depth, FLAGS_max_depth, FLAGS_fov_deg,
                                FLAGS_adaptive_threshold_kernel_size, FLAGS_adaptive_threshold_c, FLAGS_median_filter_size,
                                FLAGS_radius_search, FLAGS_min_num_neighbors);

  
  ros::spin();


  return 0;
}
