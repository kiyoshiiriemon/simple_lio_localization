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

bool MapMatcher::match(const PointCloudPCL& pc_local, const Pose3d& lio_pose, const Pose3d& pose_guess, Pose3d& out_map_pose)
{
    PointCloudPCL pc_registered;
    pcl::transformPointCloud(pc_local, pc_registered, lio_pose.matrix().cast<float>());
    std::vector<Eigen::Vector4f> scan_pts = pcl_to_vecarray(pc_registered);
    auto [cloud, tree] = small_gicp::preprocess_points(scan_pts, DownsamplingResolution, NumNeighbors, NumThreads);

    RegistrationSetting setting;
    setting.type = RegistrationSetting::GICP;
    setting.voxel_resolution = DownsamplingResolution;
    setting.num_threads = NumThreads;
    setting.max_correspondence_distance = 1.0;

    RegistrationResult result = small_gicp::align(*map_.map_cloud_, *cloud, *map_.map_tree_, pose_guess, setting);

    if (result.converged) {
        out_map_pose = result.T_target_source;
        std::cout << "ICP converged: " << result.T_target_source.translation().transpose() << std::endl;
    } else {
        std::cout << "ICP not converged" << std::endl;
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
