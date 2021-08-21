#include <iostream>
//#include <Eigen/Geometry> 
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>


int main()
{
  pcl::PointCloud<pcl::PointXYZ> cloud;

  // Fill in the cloud data
  cloud.width    = 8;
  cloud.height   = 1;
  cloud.is_dense = false;
  cloud.points.resize (cloud.width * cloud.height);

  Eigen::RowVectorXd X(8);
  Eigen::RowVectorXd Y(8);
  Eigen::RowVectorXd Z(8);

  X << 0, 10.9, 10.9, 0, 0, 10.9, 10.9, 0;
  Y << 0, 0, 0, 0, 6.9, 6.9, 6.9, 6.9;
  Z << 0, 0, 7.3, 7.3, 0, 0, 7.3, 7.3;
  
  int i=0;
  for (auto& point: cloud)
  {
    point.x = X[i]/100;
    point.y = Y[i]/100;
    point.z = Z[i]/100;
    i++;
  }

  pcl::io::savePCDFileASCII ("WhiteBox_pcd.pcd", cloud);
  std::cerr << "Saved " << cloud.size () << " data points to plate_pcd.pcd." << std::endl;

  for (const auto& point: cloud)
  {
    std::cerr << "    " << point.x << " " << point.y << " " << point.z << std::endl;
  }
  return (0);
}