#include "simple_lio_loc.h"

namespace simple_lio_localization
{

SimpleLIOLoc::SimpleLIOLoc()
{
}

SimpleLIOLoc::~SimpleLIOLoc()
{
    registration_working_ = false;
    registration_thread_.join();
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

void SimpleLIOLoc::startAsynchronousRegistration()
{
    registration_working_ = true;
    asynchronous_registration_ = true;
    registration_thread_ = std::thread(&SimpleLIOLoc::registration_worker, this);
}

void SimpleLIOLoc::update(const PointCloudPCL& pc_local, const Pose3d& lio_pose)
{
    pc_buffer_.push_back(pc_local);
    odom_buffer_.push_back(lio_pose);
    distance_since_last_update_ += (lio_pose_.translation() - lio_pose.translation()).norm();

    if (pc_buffer_.size() < params_.registration_interval || distance_since_last_update_ < params_.min_update_distance) {
        std::cerr << "accumulating points [" << pc_buffer_.size() << "/" << params_.registration_interval << "]" << std::endl;
    } else {
        Pose3d map_pose;
        Pose3d pose_guess = getLIOToMap();
        if (asynchronous_registration_) {
            RegistrationData data;
            data.pc_buffer = pc_buffer_;
            data.odom_buffer = odom_buffer_;
            {
                std::lock_guard<std::mutex> lock(registration_mutex_);
                if (registration_queue_.size() > 0) {
                    std::cerr << "WARN: the previous registration is not finished yet " << registration_queue_.size() << std::endl;
                }
                registration_queue_.push(data);
                registration_cv_.notify_one();
            }
        } else {
            RegistrationResult result;

            std::chrono::system_clock::time_point t0 = std::chrono::system_clock::now();
            result.converged = map_matcher_.match(pc_buffer_, odom_buffer_, pose_guess, map_pose, result.pc_registered);
            std::chrono::system_clock::time_point t1 = std::chrono::system_clock::now();
            result.elapsed_sec = std::chrono::duration_cast<std::chrono::seconds>(t1-t0).count();
            if (result.converged) {
                // success
                odom_to_map_ = map_pose;
                result.trans = map_pose;
            }
            if (registration_done_callback_) {
                registration_done_callback_(result);
            }
        }
        pc_buffer_.clear();
        odom_buffer_.clear();
        distance_since_last_update_ = 0.0;
    }
    lio_pose_ = lio_pose;
}

void SimpleLIOLoc::setParams(const Params& params)
{
    params_ = params;
}

Eigen::Isometry3d SimpleLIOLoc::getPose()
{
    return getLIOToMap() * lio_pose_;
}

SimpleLIOLoc::LocalizationStatus SimpleLIOLoc::getStatus()
{
    return LocalizationStatus::OK;
}

Eigen::Isometry3d SimpleLIOLoc::getLIOToMap()
{
    std::lock_guard<std::mutex> lock(odom_to_map_mutex_);
    return odom_to_map_;
}

void SimpleLIOLoc::registration_worker() {
    std::cerr << "registration worker started" << std::endl;
    while (registration_working_) {
        RegistrationData data;
        if (registration_queue_.empty()) {
            std::unique_lock<std::mutex> lock(registration_mutex_);
            registration_cv_.wait(lock);
            if (!registration_working_ || registration_queue_.empty()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                continue;
            }
            data = registration_queue_.front();
            registration_queue_.pop();
        } else {
            std::lock_guard<std::mutex> lock(registration_mutex_);
            data = registration_queue_.front();
            registration_queue_.pop();
        }
        //std::cerr << "registration worker: start registration" << std::endl;
        Pose3d map_pose;
        Pose3d pose_guess = odom_to_map_;

        std::chrono::system_clock::time_point t0 = std::chrono::system_clock::now();
        PointCloudPCL::Ptr registered_cloud(new PointCloudPCL);
        RegistrationResult result;
        result.converged = map_matcher_.match(data.pc_buffer, data.odom_buffer, pose_guess, map_pose, *registered_cloud);
        if (result.converged) {
            std::lock_guard<std::mutex> lock(odom_to_map_mutex_);
            odom_to_map_ = map_pose;
        }
        std::chrono::system_clock::time_point t1 = std::chrono::system_clock::now();
        result.elapsed_sec = std::chrono::duration_cast<std::chrono::milliseconds>(t1-t0).count();
        if (registration_done_callback_) {
            registration_done_callback_(result);
        }
    }
    std::cerr << "registration worker stopped" << std::endl;
}

void SimpleLIOLoc::terminate() {
    registration_working_ = false;
    registration_cv_.notify_one();
}

void SimpleLIOLoc::setRegistrationDoneCallback(std::function<void(const RegistrationResult &)> callback) {
    registration_done_callback_ = callback;
}


}

