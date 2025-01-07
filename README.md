# simple_lio_localization

A lightweight library for map-based localization using LiDAR-Inertial Odometry (LIO)

Features:
- 3D map-based localization
- Integration with external LIO odometry for pose prediction
- Point cloud registration using [small_gicp](https://github.com/koide3/small_gicp) for map-matching
- Support for multi-threaded, asynchronous registration

## Installation
#### Prerequisites:
- C++17 or higher
- PCL (Point Cloud Library)
- Eigen
- Boost

#### Build:
```bash
git clone --recursive https://github.com/kiyoshiiriemon/simple_lio_localization/
cd simple_lio_localization
cmake -Bbuild .
cmake --build build
```

## API
### Constructor
```cpp
SimpleLIOLoc();
```

Initializes the localization system.

### Load Map
```cpp
bool loadMap(const std::string& map_file);
```

Loads a map file for localization.

#### Parameters:
- map_file: Path to the map file (PointCloud Library PCD file).

### Set Initial Pose
```cpp
void setInitialPose(const Pose3d& initial_pose);
```

Sets the initial pose of the system.
#### Parameters:
- initial_pose: The starting position and orientation.

### Update Estimation
```cpp
update(const PointCloudPCL& pc, const Pose3d& lio_pose, CoordinateFrame frame=CoordinateFrame::LOCAL);
```

Updates the estimated position using LIO outputs.

#### Parameters:
- pc: Point cloud from the LiDAR sensor.
- lio_pose: Current pose estimate from LIO.
- frame: Reference frame of the input data
   - LOCAL: robot local coordinate frame
   - LIO: LIO frame (for LIO-registered point cloud)

### Get Current Pose
```cpp
Eigen::Isometry3d getPose();
```

### Using Asynchronous Registration
```cpp
void startAsynchronousRegistration();
```

#### Description:
Starts the registration process in a separate thread.
After calling this function, registration is performed in a non-blocking manner (update() returns immediately).

#### Receiving Registration Done Notification
```cpp
void setRegistrationDoneCallback(std::function<void(const RegistrationResult &pose)> callback);
```

Registers a callback function that is invoked when the asynchronous registration completes.

Example:
```cpp
loc.setRegistrationDoneCallback([](const RegistrationResult& result) {
    if (result.converged) {
        std::cout << "Registration succeeded. Pose: " << result.trans.matrix() << std::endl;
    } else {
        std::cerr << "Registration failed." << std::endl;
    }
});
```

### Set Parameters
```cpp
void setParams(const Params &params);
```
#### Parameters:
| Parameter                | Type   | Default | Description                                |
|--------------------------|--------|---------|--------------------------------------------|
| `frames_accumulate`      | int    | 1       | Number of frames to accumulate before registration |
| `min_registration_distance` | double | 0.0     | Minimum distance threshold for registration updates |
| `max_accumulate_frames`  | int    | 100     | Maximum number of frames to accumulate    |


