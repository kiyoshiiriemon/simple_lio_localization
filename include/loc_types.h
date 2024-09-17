#pragma once
#include <pcl/io/pcd_io.h>

namespace simple_lio_localization
{

typedef pcl::PointCloud<pcl::PointXYZI> PointCloudPCL;
typedef Eigen::Isometry3d Pose3d;

}