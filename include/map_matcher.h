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
    bool match(const PointCloudPCL& pc, const Pose3d& lio_pose, const Pose3d& pose_guess, Pose3d& out_map_pose);
    bool match(const std::vector<PointCloudPCL>& pc_vec, const std::vector<Pose3d>& lio_pose_vec, const Pose3d& pose_guess, Pose3d& out_map_pose);
private:
    Map map_;

    double DownsamplingResolution = 0.25;
    int NumNeighbors = 10;
    int NumThreads   = 4;
};

}
