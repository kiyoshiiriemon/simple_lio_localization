#ifndef SIMPLE_LIO_LOCALIZATION_SIMPLE_LIO_LOC_H
#define SIMPLE_LIO_LOCALIZATION_SIMPLE_LIO_LOC_H

#include <queue>
#include <thread>
#include <mutex>
#include <condition_variable>
#include "loc_types.h"
#include "map_matcher.h"

namespace simple_lio_localization
{

struct Params
{
    int registration_interval = 1;
    double min_update_distance = 0;
};

struct RegistrationResult
{
    bool converged;
    Eigen::Isometry3d trans;
    PointCloudPCL pc_registered;
    double elapsed_sec;
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
    bool initializeByRegistration(const PointCloudPCL& pc_local, const Pose3d& initial_pose_guess);
    void update(const PointCloudPCL& pc_local, const Pose3d& lio_pose);
    void setParams(const Params &params);
    void setRegistrationDoneCallback(std::function<void(const RegistrationResult &pose)> callback);
    void startAsynchronousRegistration();
    Eigen::Isometry3d getPose();
    Eigen::Isometry3d getLIOToMap();
    LocalizationStatus getStatus();
    void terminate();

private:
    MapMatcher map_matcher_;
    Pose3d initial_pose_;
    Pose3d odom_to_map_;
    Pose3d lio_pose_;
    LocalizationStatus status_;
    std::vector<PointCloudPCL> pc_buffer_;
    std::vector<Pose3d> odom_buffer_;
    Params params_;
    std::function<void(const RegistrationResult &)> registration_done_callback_ = nullptr;

    void registration_worker();
    struct RegistrationData
    {
        std::vector<PointCloudPCL> pc_buffer;
        std::vector<Pose3d> odom_buffer;
    } registration_data_;
    bool registration_working_ = true;
    std::thread registration_thread_;
    std::mutex registration_mutex_;
    std::mutex odom_to_map_mutex_;
    std::condition_variable registration_cv_;
    std::queue<RegistrationData> registration_queue_;
    bool asynchronous_registration_ = false;
    double distance_since_last_update_ = 0.0;
};

}

#endif //SIMPLE_LIO_LOCALIZATION_SIMPLE_LIO_LOC_H
