#include <mapper_emvs/mapper_emvs.hpp>

#include <mapper_emvs/median_filtering.hpp>
#include <pcl/filters/radius_outlier_removal.h>
#include <geometry_msgs/PoseStamped.h>

namespace EMVS {

using namespace geometry_utils;

MapperEMVS::MapperEMVS(const image_geometry::PinholeCameraModel& cam,
                       const ShapeDSI& dsi_shape)
  : dvs_cam_(cam)
  , dsi_shape_(dsi_shape)
{
  cv::Size full_resolution = cam.fullResolution();
  width_ = full_resolution.width;//346
  height_ = full_resolution.height;//260
  K_ << dvs_cam_.fx(), 0.f, dvs_cam_.cx(),
       0.f, dvs_cam_.fy(), dvs_cam_.cy(),
       0.f, 0.f, 1.f;
  // K_ << (float) 206.29417, 0.f          , (float) 115.647026,
  //         0.f        , (float) 206.3225  , (float) 83.69464,
  //         0.f        , 0.f          ,  1.f;

  std::cout << K_ << std::endl;


  setupDSI();

  precomputeRectifiedPoints();
}


bool MapperEMVS::evaluateDSI(const std::vector<dvs_msgs::Event>& events,
                             const TrajectoryType& trajectory,
                             const geometry_utils::Transformation& T_rv_w)
{
  if(events.size() < packet_size_)
  {
    LOG(WARNING) << "Number of events ( " << events.size() << ") < packet size (" << packet_size_ << ")";
    return false;
  }

  // 2D coordinates of the events transferred to reference view using plane Z = Z_0.
  // We use Vector4f because Eigen is optimized for matrix multiplications with inputs whose size is a multiple of 4
  static std::vector<Eigen::Vector4f> event_locations_z0;
  event_locations_z0.clear();

  // List of camera centers
  static std::vector<Eigen::Vector3f> camera_centers;
  camera_centers.clear();

  // Loop through the events, grouping them in packets of frame_size_ events
  size_t current_event_ = 0;
  while(current_event_ + packet_size_ < events.size())
  {
    // Events in a packet are assigned the same timestamp (mid-point), for efficiency
    ros::Time frame_ts = events[current_event_ + packet_size_ / 2].ts;

    Transformation T_w_ev; // from event camera to world
    Transformation T_rv_ev; // from event camera to reference viewpoint
    if(!trajectory.getPoseAt(frame_ts, T_w_ev))
    {
      current_event_++;
      continue;
    }
    // from event camera to reference viewpoint
    T_rv_ev = T_rv_w * T_w_ev;

    const Transformation T_ev_rv = T_rv_ev.inverse();
    const Eigen::Matrix3f R = T_ev_rv.getRotationMatrix().cast<float>();
    const Eigen::Vector3f t = T_ev_rv.getPosition().cast<float>();

    // Optical center of the event camera in the coordinate frame of the reference view
    camera_centers.push_back(-R.transpose() * t);

    // Project the points on plane at distance z0
    const float z0 = raw_depths_vec_[0];

    // Planar homography  (H_z0)^-1 that maps a point in the reference view to the event camera through plane Z = Z0 (Eq. (8) in the IJCV paper)
    Eigen::Matrix3f H_z0_inv = R;
    H_z0_inv *= z0;
    H_z0_inv.col(2) += t;

    // Compute H_z0 in pixel coordinates using the intrinsic parameters
    Eigen::Matrix3f H_z0_inv_px = K_ * H_z0_inv * virtual_cam_.Kinv_;
    Eigen::Matrix3f H_z0_px = H_z0_inv_px.inverse();

    // Use a 4x4 matrix to allow Eigen to optimize the speed
    Eigen::Matrix4f H_z0_px_4x4;
    H_z0_px_4x4.block<3,3>(0,0) = H_z0_px;
    H_z0_px_4x4.col(3).setZero();
    H_z0_px_4x4.row(3).setZero();

    // For each packet, precompute the warped event locations according to Eq. (11) in the IJCV paper.
    for (size_t i=0; i < packet_size_; ++i)
    {
      const dvs_msgs::Event& e = events[current_event_++];
      Eigen::Vector4f p;

      p.head<2>() = precomputed_rectified_points_.col(e.y * width_ + e.x);
      p[2] = 1.;
      p[3] = 0.;

      p = H_z0_px_4x4 * p;
      p /= p[2];

      event_locations_z0.push_back(p);
    }
  }

  dsi_.resetGrid();
  // LOG(INFO) << "Fill voxel";
  fillVoxelGrid(event_locations_z0, camera_centers);

  return true;
}


void MapperEMVS::fillVoxelGrid(const std::vector<Eigen::Vector4f>& event_locations_z0,
                               const std::vector<Eigen::Vector3f>& camera_centers)
{
  // This function implements Step 2 of Algorithm 1 in the IJCV paper.
  // It maps events from plane Z0 to all the planes Zi of the DSI using Eq. (15)
  // and then votes for the corresponding voxel using bilinear voting.

  // For efficiency reasons, we split each packet into batches of N events each
  // which allows to better exploit the L1 cache
  static const int N = 5;
  typedef Eigen::Array<float, N, 1> Arrayf;

  const float z0 = raw_depths_vec_[0];

  // Parallelize over the planes of the DSI with OpenMP
  // (each thread will process a different depth plane)
  #pragma omp parallel for if (event_locations_z0.size() >= 20000)
  for(size_t depth_plane = 0; depth_plane < raw_depths_vec_.size(); ++depth_plane)
  {
    const Eigen::Vector4f* pe = &event_locations_z0[0];
    float *pgrid = dsi_.getPointerToSlice(depth_plane);
    // LOG(INFO) << "Get pointer";
    std::vector<Eigen::Vector3f>* vector_grid = dsi_.getPointerToVectorSlice(depth_plane);
    std::vector<Eigen::Vector3f>* camera_center_grid = dsi_.getPointerToCameraCenter(depth_plane);

    for (size_t packet=0; packet < camera_centers.size(); ++packet)
    {
      // Precompute coefficients for Eq. (15)
      const Eigen::Vector3f& C = camera_centers[packet];
      const float zi = static_cast<float>(raw_depths_vec_[depth_plane]),
          a = z0 * (zi - C[2]),
          bx = (z0 - zi) * (C[0] * virtual_cam_.fx_ + C[2] * virtual_cam_.cx_),
          by = (z0 - zi) * (C[1] * virtual_cam_.fy_ + C[2] * virtual_cam_.cy_),
          d = zi * (z0 - C[2]);

      // Update voxel grid now, N events per iteration
      for(size_t batch=0; batch < packet_size_ / N; ++batch, pe += N)
      {
        // Eq. (15)
        Arrayf X, Y;
        for (size_t i=0; i < N; ++i)
        {
          X[i] = pe[i][0];
          Y[i] = pe[i][1];
        }
        X = (X * a + bx) / d;
        Y = (Y * a + by) / d;

        for (size_t i=0; i < N; ++i)
        {
          // Bilinear voting
          // LOG(INFO) << "bilinear vote";
          dsi_.accumulateGridValueAt(X[i], Y[i], pe[i], camera_centers[packet], z0, pgrid, vector_grid, camera_center_grid, virtual_cam_.Kinv_);
          // LOG(INFO) << "bilinear vote out";
        }
      }
    }
  }
}


void MapperEMVS::setupDSI()
{
  CHECK_GT(dsi_shape_.min_depth_, 0.0);
  CHECK_GT(dsi_shape_.max_depth_ , dsi_shape_.min_depth_);

  depths_vec_ = TypeDepthVector(dsi_shape_.min_depth_, dsi_shape_.max_depth_, dsi_shape_.dimZ_);
  raw_depths_vec_ = depths_vec_.getDepthVector();

  dsi_shape_.dimX_ = (dsi_shape_.dimX_ > 0) ? dsi_shape_.dimX_ : dvs_cam_.fullResolution().width;
  dsi_shape_.dimY_ = (dsi_shape_.dimY_ > 0) ? dsi_shape_.dimY_ : dvs_cam_.fullResolution().height;
  
  float f_virtual_cam_;
  if (dsi_shape_.fov_ < 10.f)
  {
    LOG(INFO) << "Specified DSI FoV < 10 deg. Will use camera FoV instead.";
    f_virtual_cam_ = dvs_cam_.fx();
  }
  else
  {
    const float dsi_fov_rad = dsi_shape_.fov_ * CV_PI / 180.0;
    f_virtual_cam_ = 0.5 * (float) dsi_shape_.dimX_ / std::tan(0.5 * dsi_fov_rad);
  }
  LOG(INFO) << "Focal length of virtual camera: " << f_virtual_cam_ << " pixels";

  virtual_cam_ = PinholeCamera(dsi_shape_.dimX_, dsi_shape_.dimY_,
                               f_virtual_cam_, f_virtual_cam_,
                               0.5 * (float)dsi_shape_.dimX_, 0.5 * (float)dsi_shape_.dimY_);
    
  dsi_ = Grid3D(dsi_shape_.dimX_, dsi_shape_.dimY_, dsi_shape_.dimZ_);
}


void MapperEMVS::precomputeRectifiedPoints()
{
  // Create a lookup table that maps pixel coordinates to undistorted pixel coordinates
  precomputed_rectified_points_ = Eigen::Matrix2Xf(2, height_ * width_);
  for(int y=0; y < height_; y++)
  {
    for(int x=0; x < width_; ++x)
    {
      cv::Point2d rectified_point = dvs_cam_.rectifyPoint(cv::Point2d(x,y));//cv::Point2d(x,y);
      //cv::Point2d rectified_point = cv::Point2d(x,y);
      precomputed_rectified_points_.col(y * width_ + x) = Eigen::Vector2f(rectified_point.x, rectified_point.y);
    }
  }
}


void MapperEMVS::convertDepthIndicesToValues(const cv::Mat &depth_cell_indices, cv::Mat &depth_map)
{
  // Convert depth indices to depth values, for all pixels
  depth_map = cv::Mat(depth_cell_indices.rows, depth_cell_indices.cols, CV_32F);
  for(int y=0; y<depth_cell_indices.rows; ++y)
  {
    for(int x=0; x<depth_cell_indices.cols; ++x)
    {
      depth_map.at<float>(y,x) = depths_vec_.cellIndexToDepth(depth_cell_indices.at<uchar>(y,x));
    }
  }
}


void MapperEMVS::removeMaskBoundary(cv::Mat& mask, int border_size)
{
  for(int y=0; y<mask.rows; ++y)
  {
    for(int x=0; x<mask.cols; ++x)
    {
      if(x <= border_size || x >= mask.cols - border_size ||
         y <= border_size || y >= mask.rows - border_size)
      {
        mask.at<uchar>(y,x) = 0;
      }
    }
  }
}

void MapperEMVS::getDepthMapFromDSI(cv::Mat& depth_map, cv::Mat &confidence_map, cv::Mat &mask, const OptionsDepthMap &options_depth_map)
{
  // Reference: Section 5.2.3 in the IJCV paper.

  // Maximum number of votes along optical ray
  cv::Mat depth_cell_indices;
  dsi_.collapseMaxZSlice(&confidence_map, &depth_cell_indices);
  
  // Adaptive thresholding on the confidence map
  cv::Mat confidence_8bit;
  cv::normalize(confidence_map, confidence_8bit, 0.0, 255.0, cv::NORM_MINMAX);
  confidence_8bit.convertTo(confidence_8bit, CV_8U);
  
  cv::adaptiveThreshold(confidence_8bit,
                        mask,
                        1,
                        cv::ADAPTIVE_THRESH_GAUSSIAN_C,
                        cv::THRESH_BINARY,
                        options_depth_map.adaptive_threshold_kernel_size_,
                        -options_depth_map.adaptive_threshold_c_);


  // Clean up depth map using median filter (Section 5.2.5 in the IJCV paper)
  cv::Mat depth_cell_indices_filtered;
  huangMedianFilter(depth_cell_indices,
                    depth_cell_indices_filtered,
                    mask,
                    options_depth_map.median_filter_size_);

  // Remove the outer border to suppress boundary effects
  const int border_size = std::max(options_depth_map.adaptive_threshold_kernel_size_ / 2, 1);
  removeMaskBoundary(mask, border_size);

  // Convert depth indices to depth values
  convertDepthIndicesToValues(depth_cell_indices_filtered, depth_map);
}

void MapperEMVS::getIntersectionPointCloudFromDSI(const OptionsDepthMap &options_depth_map, const OptionsPointCloud &options_pc, PointCloud::Ptr &pc_)
{
  // Reference: Section 5.2.3 in the IJCV paper.
  std::vector<std::vector<Eigen::Vector3f>> start_point_vec, end_point_vec;
  start_point_vec.resize(width_  * height_);
  end_point_vec.resize(width_ * height_);

  // Maximum number of votes along optical ray
  cv::Mat depth_map, confidence_map, mask, depth_cell_indices;
  dsi_.collapseMaxZSlice(&confidence_map, &depth_cell_indices);
  
  // Adaptive thresholding on the confidence map
  cv::Mat confidence_8bit;
  cv::normalize(confidence_map, confidence_8bit, 0.0, 255.0, cv::NORM_MINMAX);
  confidence_8bit.convertTo(confidence_8bit, CV_8U);
  
  cv::adaptiveThreshold(confidence_8bit,
                        mask,
                        1,
                        cv::ADAPTIVE_THRESH_GAUSSIAN_C,
                        cv::THRESH_BINARY,
                        options_depth_map.adaptive_threshold_kernel_size_,
                        -options_depth_map.adaptive_threshold_c_);
  
  LOG(INFO) << "Get passing vectors";
  //Extract passing vectors
  for(int y=0; y<depth_cell_indices.rows; ++y)
  {
    for(int x=0; x<depth_cell_indices.cols; ++x)
    {
      start_point_vec[y*depth_cell_indices.cols + x] = dsi_.getVectorsAt(x, y, depth_cell_indices.at<uchar>(y, x));
      end_point_vec[y*depth_cell_indices.cols + x] = dsi_.getCameraCentersAt(x, y, depth_cell_indices.at<uchar>(y, x));
    }
  }
  
  //Check intersecting vectors at maximum confidence point
  // cv::Point minLoc, maxLoc; 
  // double minVal, maxVal;
  // cv::minMaxLoc(confidence_map, &minVal, &maxVal, &minLoc, &maxLoc);
  // std::vector<Eigen::Vector3f> best_intersecting_vectors = dsi_.getVectorsAt(maxLoc.x, maxLoc.y, depth_cell_indices.at<uchar>(maxLoc.y, maxLoc.x));
  // std::vector<Eigen::Vector3f> camera_centers = dsi_.getCameraCentersAt(maxLoc.x, maxLoc.y, depth_cell_indices.at<uchar>(maxLoc.y, maxLoc.x));
  // LOG(INFO) << "Best Intersecting vectors: ";
  // for (uint8_t i=0; i<10 ; i++)
  // {
  //   int j = rand() * best_intersecting_vectors.size() / RAND_MAX;
  //   LOG(INFO) << "Camera Center: ";
  //   LOG(INFO) << camera_centers[j];
  //   LOG(INFO) << "Vector: ";
  //   LOG(INFO) << best_intersecting_vectors[j];
  // }

  

  LOG(INFO) << "Clean depth map vectors";
  // Clean up depth map using median filter (Section 5.2.5 in the IJCV paper)
  cv::Mat depth_cell_indices_filtered;
  huangMedianFilter(depth_cell_indices,
                    depth_cell_indices_filtered,
                    mask,
                    options_depth_map.median_filter_size_);

  // Remove the outer border to suppress boundary effects
  const int border_size = std::max(options_depth_map.adaptive_threshold_kernel_size_ / 2, 1);
  removeMaskBoundary(mask, border_size);

  LOG(INFO) << "Get intercetions points";
  // Convert depth indices to depth values
  // BearingVector b_rv = virtual_cam_.projectPixelTo3dRay(Keypoint(maxLoc.x, maxLoc.y));
  // b_rv.normalize();
  // Eigen::Vector3d xyz_rv = (b_rv / b_rv[2] * depth_map.at<float>(maxLoc.y, maxLoc.x));
  // LOG(INFO) << xyz_rv;
  pc_->clear();
  for(int y=0; y<mask.rows; ++y)
  {
    for(int x=0; x<mask.cols; ++x)
    {
      if(mask.at<uint8_t>(y,x) > 0)
      {
        //LOG(INFO) << "Get specific intersection point";
        Eigen::Vector3f xyz_rv = this->getIntersectionPoint(start_point_vec[y*depth_cell_indices.cols + x], end_point_vec[y*depth_cell_indices.cols + x]);
        //LOG(INFO) << "Got specific intersection point";

        pcl::PointXYZI p_rv; // 3D point in reference view
        p_rv.x = xyz_rv.x();
        p_rv.y = xyz_rv.y();
        p_rv.z = xyz_rv.z();
        p_rv.intensity = 1.0 / p_rv.z;
        pc_->push_back(p_rv);
        //LOG(INFO) << "Contruct i'th point in pointcloud";
      }
    } 
  }
}


Eigen::Vector3f MapperEMVS::getIntersectionPoint(std::vector<Eigen::Vector3f> start_point_vectors, std::vector<Eigen::Vector3f> end_point_vectors)
{
  Eigen::MatrixXf start_points(3, start_point_vectors.size());
  Eigen::MatrixXf end_points(3, start_point_vectors.size());
  Eigen::MatrixXf PA(start_point_vectors.size(),3); //NOTE: initiate number of rows based on the number of vectors we have
  Eigen::MatrixXf PB(end_point_vectors.size(),3);
  Eigen::MatrixXf Si;
  Eigen::MatrixXf Si2;
  Eigen::MatrixXf Si_pow;
  Eigen::MatrixXf ni;
  Eigen::Matrix3f S;
  Eigen::VectorXf nx;
  Eigen::VectorXf ny;
  Eigen::VectorXf nz;
  Eigen::Vector3f C;
  Eigen::Vector3f P_intersect;

  //LOG(INFO) << "Convert to matrix";  
  for(int i=0; i<start_point_vectors.size(); i++)
  {
    start_points.col(i) = start_point_vectors[i];
    end_points.col(i) = end_point_vectors[i];
  }

  PA = start_points.transpose();
  PB = end_points.transpose();

  Si = PB - PA;
  Si2 = end_points - start_points;

  //LOG(INFO) << "Start code";  
  Si_pow = Eigen::sqrt(Eigen::square(Si.array()).rowwise().sum());

  ni = Si.array()/(Si_pow*Eigen::MatrixXf::Ones(1,3)).array();

  nx = ni.col(0);
  ny = ni.col(1); 
  nz = ni.col(2); 

  float SXX = (pow(nx.array(), 2) - 1).sum();
  float SYY = (pow(ny.array(), 2) - 1).sum();
  float SZZ = (pow(nz.array(), 2) - 1).sum();
  float SXY = (nx.array()*ny.array()).sum();
  float SXZ = (nx.array()*nz.array()).sum();
  float SYZ = (ny.array()*nz.array()).sum();

  S << SXX, SXY, SXZ,
      SXY, SYY, SYZ,
      SXZ, SYZ, SZZ;


  float CX = (PA.col(0).array() * (Eigen::square(nx.array()) - 1) + PA.col(1).array() * (nx.array()*ny.array()) +  PA.col(2).array() * (nx.array()*nz.array())).sum();
  float CY = (PA.col(0).array() * (nx.array()*ny.array()) + PA.col(1).array() * (Eigen::square(ny.array()) - 1) +  PA.col(2).array() * (ny.array()*nz.array())).sum();
  float CZ = (PA.col(0).array() * (nx.array()*nz.array()) + PA.col(1).array() * (ny.array()*nz.array()) +  PA.col(2).array() * (Eigen::square(nz.array()) - 1)).sum();

  C << CX,CY,CZ;

  //LOG(INFO) << "construct intersectionpoint";  
  //P_intersect = (S.completeOrthogonalDecomposition().pseudoInverse()*C).transpose();	
  P_intersect = S.ldlt().solve(C).transpose();
  
  //Find Distance between intersection point and lines
  //float N = start_point_vectors.size();
  float ui;
  Eigen::VectorXf distances(start_point_vectors.size(),1);

  for(int i=0; i < start_point_vectors.size(); i++)
  {
    ui = (P_intersect.transpose() - PA.row(i)) * (Si.row(i).transpose() / (Si.row(i)*Si.row(i).transpose()));
    distances(i) = (P_intersect.transpose()-PA.row(i)-ui*Si.row(i)).norm();
    if(distances(i) > 0.05 )
    {
      break; //TODO: Check with Abdulla
    }
    else
    {
      //LOG(INFO) << "Distances are : " << distances;
      return P_intersect;
    }
  }
}


void MapperEMVS::getPointcloud(const cv::Mat& depth_map,
                                const cv::Mat& mask,
                                const OptionsPointCloud &options_pc,
                                PointCloud::Ptr &pc_
                              )
{
  CHECK_EQ(depth_map.rows, mask.rows);
  CHECK_EQ(depth_map.cols, mask.cols);
  
  // Convert depth map to point cloud
  pc_->clear();
  for(int y=0; y<depth_map.rows; ++y)
  {
    for(int x=0; x<depth_map.cols; ++x)
    {
      if(mask.at<uint8_t>(y,x) > 0)
      {
        BearingVector b_rv = virtual_cam_.projectPixelTo3dRay(Keypoint(x,y));
        b_rv.normalize();
        Eigen::Vector3d xyz_rv = (b_rv / b_rv[2] * depth_map.at<float>(y,x));

        pcl::PointXYZI p_rv; // 3D point in reference view
        p_rv.x = xyz_rv.x();
        p_rv.y = xyz_rv.y();
        p_rv.z = xyz_rv.z();
        p_rv.intensity = 1.0 / p_rv.z;
        pc_->push_back(p_rv);
      }
    }
  }

  // Filter point cloud to remove outliers (Section 5.2.5 in the IJCV paper)
  PointCloud::Ptr cloud_filtered (new PointCloud);
  pcl::RadiusOutlierRemoval<PointType> outlier_rm;
  outlier_rm.setInputCloud(pc_);
  outlier_rm.setRadiusSearch(options_pc.radius_search_);
  outlier_rm.setMinNeighborsInRadius(options_pc.min_num_neighbors_);
  outlier_rm.filter(*cloud_filtered);

  pc_->swap(*cloud_filtered);
}

void MapperEMVS::PCtoVoxelGrid(PointCloud::Ptr cloud, PointCloud::Ptr cloud_filtered, float leaf_size_x, float leaf_size_y, float leaf_size_z)
{
 
  std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height 
       << " data points (" << pcl::getFieldsList (*cloud) << ")." << std::endl;

  // Create the filtering object
  pcl::VoxelGrid<PointType> sor;
  sor.setInputCloud (cloud);
  sor.setLeafSize (leaf_size_x, leaf_size_y, leaf_size_z);
  sor.filter (*cloud_filtered);

  std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height 
       << " data points (" << pcl::getFieldsList (*cloud_filtered) << ")." << std::endl;

}

}
