#include "simple_lio_loc.h"

namespace simple_lio_localization
{

SimpleLIOLoc::SimpleLIOLoc()
{
}

SimpleLIOLoc::~SimpleLIOLoc()
{
}

bool SimpleLIOLoc::loadMap(const std::string& map_file)
{
    return map_matcher_.loadMap(map_file);
}

void SimpleLIOLoc::setInitialPose(const Pose3d& initial_pose)
{
    initial_pose_ = initial_pose;
    odom_to_map_ = initial_pose;
}

void SimpleLIOLoc::update(const PointCloudPCL& pc_local, const Pose3d& lio_pose)
{
    pc_buffer_.push_back(pc_local);
    odom_buffer_.push_back(lio_pose);

    if (pc_buffer_.size() < params_.update_interval) {
        std::cerr << "accumulating points" << std::endl;
    } else {
        Pose3d map_pose;
        Pose3d pose_guess = odom_to_map_;
        //if (map_matcher_.match(pc_local, lio_pose, pose_guess, map_pose)) {
        if (map_matcher_.match(pc_buffer_, odom_buffer_, pose_guess, map_pose)) {
            // success
            odom_to_map_ = map_pose;
        }
        pc_buffer_.clear();
        odom_buffer_.clear();
    }
    lio_pose_ = lio_pose;
}

void SimpleLIOLoc::setParams(const Params& params)
{
    params_ = params;
}

Eigen::Isometry3d SimpleLIOLoc::getPose()
{
    return odom_to_map_ * lio_pose_;
}

SimpleLIOLoc::LocalizationStatus SimpleLIOLoc::getStatus()
{
    return LocalizationStatus::OK;
}

Eigen::Isometry3d SimpleLIOLoc::getLIOToMap()
{
    return odom_to_map_;
}

}

