#include "simple_lio_loc.h"

using namespace simple_lio_localization;

std::string isometry3d_to_string(const Eigen::Isometry3d& pose) {
    Eigen::Quaterniond q(pose.rotation());
    Eigen::Vector3d t = pose.translation();
    std::stringstream ss;
    ss << t.x() << " " << t.y() << " " << t.z() << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w();
    return ss.str();
}

std::vector<Eigen::Isometry3d> read_rosbag_odom_csv(const std::string& filename) {
    std::vector<Eigen::Isometry3d> poses;
    std::ifstream ifs(filename);
    if (!ifs) {
        std::cerr << "Failed to open file: " << filename << std::endl;
        return poses;
    }
    // skip header
    std::string header;
    std::getline(ifs, header);

    while (!ifs.eof()) {
        std::string line;
        std::getline(ifs, line);
        if (line.empty()) {
            break;
        }
        std::stringstream ss(line);
        std::string token;
        std::vector<std::string> values;
        while (std::getline(ss, token, ',')) {
            values.push_back(token);
            //std::cout << "token: " << token << std::endl;
        }
        double x = std::stod(values[5]);
        double y = std::stod(values[6]);
        double z = std::stod(values[7]);
        double qx = std::stod(values[8]);
        double qy = std::stod(values[9]);
        double qz = std::stod(values[10]);
        double qw = std::stod(values[11]);

        std::cout << "x: " << x << ", y: " << y << ", z: " << z << ", qx: " << qx << ", qy: " << qy << ", qz: " << qz << ", qw: " << qw << std::endl;

        Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
        pose.translation() = Eigen::Vector3d(x, y, z);
        pose.rotate(Eigen::Quaterniond(qw, qx, qy, qz));
        poses.push_back(pose);
    }
    return poses;
}

int main(int argc, char** argv)
{
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <data_dir>" << std::endl;
        return 1;
    }
    std::string data_dir = argv[1];
    std::ofstream ofs("est_log.txt");
    SimpleLIOLoc loc;
    std::vector<Eigen::Isometry3d> poses = read_rosbag_odom_csv(data_dir + "/odometry.csv");
    loc.loadMap(data_dir + "/map.pcd");
    Eigen::Isometry3d init_pose_guess = Eigen::Isometry3d::Identity();
    init_pose_guess.translation() = Eigen::Vector3d(0.0, 0.0, 0.4);
    init_pose_guess.rotate(Eigen::AngleAxisd(0.3, Eigen::Vector3d(0, 1, 0)));
    std::cout << "init_pose_guess: " << isometry3d_to_string(init_pose_guess) << std::endl;
    loc.setInitialPose(init_pose_guess);

    // read pcd files
    for(int i = 0; i < poses.size(); i++) {
        char filename[256];
        sprintf(filename, "%s/cloud_%06d.pcd", data_dir.c_str(), i);
        PointCloudPCL pc;
        pcl::io::loadPCDFile(filename, pc);
        //std::cout << "loaded " << pc.size() << " points" << std::endl;
        loc.update(pc, poses[i]);
        std::cout << i << ": " << isometry3d_to_string(loc.getPose()) << std::endl;
        ofs << isometry3d_to_string(loc.getPose()) << " " << isometry3d_to_string(poses[i]) << std::endl;
    }

    Eigen::Isometry3d pose = loc.getPose();
    return 0;
}
