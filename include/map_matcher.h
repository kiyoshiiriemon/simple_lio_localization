#pragma once

#include "loc_types.h"
#include <small_gicp/registration/registration_helper.hpp>

namespace simple_lio_localization
{

class Map
{
public:
    bool load_from_pcd(const std::string& pcd_file);

    std::vector<Eigen::Vector4f> pts_;
    std::shared_ptr<small_gicp::KdTree<small_gicp::PointCloud>> map_tree_ = nullptr;
    std::shared_ptr<small_gicp::PointCloud> map_cloud_ = nullptr;

    double DownsamplingResolution = 0.25;
    int NumNeighbors = 10;
    int NumThreads   = 4;
};

class MapMatcher
{
public:
    MapMatcher();
    ~MapMatcher();

    bool loadMap(const std::string& map_file);
    bool match_local(const PointCloudPCL& pc_local, const Pose3d& lio_pose, const Pose3d& pose_guess, Pose3d& out_map_pose, PointCloudPCL& out_registered_cloud);
    bool match_lioframe(const PointCloudPCL& pc_lioframe, const Pose3d& pose_guess, Pose3d& out_map_pose, PointCloudPCL& out_registered_cloud);
private:
    Map map_;

    double DownsamplingResolution = 0.25;
    int NumNeighbors = 10;
    int NumThreads   = 4;
};

}
