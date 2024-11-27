#include "map_matcher.h"
#include <pcl/common/transforms.h>

using namespace small_gicp;
using namespace simple_lio_localization;

static std::vector<Eigen::Vector4f> pcl_to_vecarray(const PointCloudPCL &cloud)
{
    std::vector<Eigen::Vector4f> points;
    for (const auto &point: cloud.points) {
        points.emplace_back(point.x, point.y, point.z, 1.0f);
    }
    return points;
}

MapMatcher::MapMatcher()
{
}

MapMatcher::~MapMatcher()
{
}

bool MapMatcher::loadMap(const std::string& map_file)
{
    return map_.load_from_pcd(map_file);
}

bool MapMatcher::match_local(const PointCloudPCL& pc_local, const Pose3d& lio_pose, const Pose3d& pose_guess, Pose3d& out_map_pose, PointCloudPCL& out_registered_cloud)
{
    PointCloudPCL pc_registered;
    pcl::transformPointCloud(pc_local, pc_registered, lio_pose.matrix().cast<float>());
    return match_lioframe(pc_registered, pose_guess, out_map_pose, out_registered_cloud);
}

bool MapMatcher::match_lioframe(const PointCloudPCL& pc_lioframe, const Pose3d& pose_guess, Pose3d& out_map_pose, PointCloudPCL& out_registered_cloud)
{
    std::vector<Eigen::Vector4f> scan_pts = pcl_to_vecarray(pc_lioframe);
    auto [cloud, tree] = small_gicp::preprocess_points(scan_pts, DownsamplingResolution, NumNeighbors, NumThreads);

    RegistrationSetting setting;
    setting.type = RegistrationSetting::GICP;
    setting.voxel_resolution = DownsamplingResolution;
    setting.num_threads = NumThreads;
    setting.max_correspondence_distance = 1.0;

    RegistrationResult result = small_gicp::align(*map_.map_cloud_, *cloud, *map_.map_tree_, pose_guess, setting);

    if (result.converged) {
        out_map_pose = result.T_target_source;
        pcl::transformPointCloud(pc_lioframe, out_registered_cloud, result.T_target_source.matrix().cast<float>());
        std::cout << "ICP converged: " << result.T_target_source.translation().transpose() << std::endl;
    } else {
        pcl::transformPointCloud(pc_lioframe, out_registered_cloud, pose_guess.matrix().cast<float>());
        std::cout << "ICP NOT converged" << std::endl;
    }

    return result.converged;
}

bool Map::load_from_pcd(const std::string& pcd_file) {
    PointCloudPCL::Ptr pcl_cloud(new PointCloudPCL);

    if (pcl::io::loadPCDFile(pcd_file, *pcl_cloud) < 0) {
        return false;
    }
    pts_ = pcl_to_vecarray(*pcl_cloud);

    auto [cloud, tree] = small_gicp::preprocess_points(pts_, DownsamplingResolution, NumNeighbors, NumThreads);
    map_cloud_ = cloud;
    map_tree_ = tree;

    return true;
}
