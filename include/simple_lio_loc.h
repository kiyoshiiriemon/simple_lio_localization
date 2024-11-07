#ifndef SIMPLE_LIO_LOCALIZATION_SIMPLE_LIO_LOC_H
#define SIMPLE_LIO_LOCALIZATION_SIMPLE_LIO_LOC_H

#include "loc_types.h"
#include "map_matcher.h"

namespace simple_lio_localization
{

struct Params
{
    int update_interval = 1;
};

class SimpleLIOLoc
{
public:
    // localization status
    enum class LocalizationStatus
    {
        OK,
        MAP_NOT_LOADED,
        INIT_POSE_NOT_GIVEN,
        LIO_FAILURE,
        MATCHING_FAILURE,
    };

    SimpleLIOLoc();
    ~SimpleLIOLoc();

    bool loadMap(const std::string& map_file);
    void setInitialPose(const Pose3d& initial_pose);
    void update(const PointCloudPCL& pc_local, const Pose3d& lio_pose);
    void setParams(const Params &params);
    Eigen::Isometry3d getPose();
    Eigen::Isometry3d getLIOToMap();
    LocalizationStatus getStatus();

private:
    MapMatcher map_matcher_;
    Pose3d initial_pose_;
    Pose3d odom_to_map_;
    Pose3d lio_pose_;
    LocalizationStatus status_;
    std::vector<PointCloudPCL> pc_buffer_;
    std::vector<Pose3d> odom_buffer_;
    Params params_;
};

}

#endif //SIMPLE_LIO_LOCALIZATION_SIMPLE_LIO_LOC_H
