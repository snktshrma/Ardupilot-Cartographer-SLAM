# Cartographer-documentation

## Cartographer SLAM

### Overview: 
Cartographer slam is a combination of two connected subsystem, Local SLAM and Global SLAM. Local SLAM build successive submaps.  Global SLAM’s main work is to find loop closure constraints between nodes and submaps and then optimizing it. The core concept on SLAM is pose graph optimization. 

### Inputs: 
LIDAR for depth information of the environment, IMU, GPS(optional)

### Local SLAM: 
`<install_isolated/share/cartographer/configuration_files/trajectory_builder_2d.lua>`

Local SLAM tries to insert a new node into the current submap created as a result of multiple past filtered scans. It does so by scan matching using initial guess from extrapolating the pose to predict future pose. 
Scan matching is done by:

1. CeresScanMatcher: It interpolate the already created submap and through initial guess it made, it finds a place where scan match fits.
If sensors are trustworthy enough, then this method is preferred.

2. RealTimeCorrelativeScanMatcher: It can be enabled if we don’t have much trust on our sensors. It matches scan to current submap using loop closure. Then the best match is used in CeresScanMatcher.


Many filters like motion filter, voxel filter, bandpass filter, etc to refine the inputs to the SLAM model.

### Global SLAM: 
 `<install_isolated/share/cartographer/configuration_files/pose_graph.lua>`

Global SLAM arranges the submaps from local SLAM to form a coherent global map. Global SLAM is a pose graph optimization technique, trying to find optimum loop closure to form the map. Global SLAM uses subsampled set of nodes to limit the constraints and computational use.

When a node and a submap are considered for constraint building, they go through a scan matcher called the FastCorrelativeScanMatcher. It works on an exploration tree whose depth can be controlled. Then its output is fed into CeresScanMatcher to refine the data.
Over the top, global slam uses scan data, submaps from local SLAM and IMU to stitch through the submaps to generate pose graph.
More the data, more optimized the graph will be.


### Hector SLAM:

Overview: Hector scan matcher take in LIDAR data, match the scans 
between two time stamps while registering the map and update the car pose at each stage. The change in pose estimate is evaluated at each stage to calculate the new pose and the scans are aligned to the new pose. We observe that once we have the change in pose estimate we transform the scan to global frame and update the global map with these scans.

Out of both, hector slam and cartographer SLAM, the latter is more efficient and optimized than the former. 
The cartographer slams has many optimization algorithm and uses other loop closure whereas in hector SLAM, there is  and no loop closure to ensure the validity of constructed map. Though both are highly used and state of the art, but due to the advantage of loop closure, which ensure a little more efficient and optimized graph, cartographer is a bit more preferable than hector SLAM.


### Tuning Cartographer SLAM:

1. Turn of global SLAM:
Turn off global SLAM to concentrate of local SLAM only for tuning pupose. <POSE_GRAPH.optimize_every_n_nodes = 0>




2. Tuning size of submaps:
If size of submaps is not within the constraints set, then using <TRAJECTORY_BUILDER_2D.submaps.num_range_data> parameter, we can set the size of submaps.

3. Tuning CeresScanMatcher:
To tune this parameter, we have <TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight> and <rotation_weight>. We use them to penalize the situation by making scan to deviate more from the prior. Concepts of CeresScanMatcher are described above in local SLAM.
We can set them accordingly to tune CeresScanMatcher to get a reasonable result.

4. Verification:
To verify the tuning, we check our SLAM working of new configuration with the older one to see the difference and will fine tune the above parameters in case of some slippage or any other issue.
Major tuning is required in CeresScanMatcher which can be tuned by above mention parameters.


### Cartographer SLAM using drone:

To use cartographer on drone, we used a modified ArduCopter UAV for the same. We added a LiDAR on it so as to map the area.

#### To start with the environment and setups:

1. Setup the ardupilot environment and simulation to start with. (Already uploaded on github)

2. Install the cartographer and cartographer-ros package.

3. Build the environment and source it.

#### To start with the configurations of drone:

1. The cartographer SLAM configuration, which we are using, will publish both, map frame and odom frame.

2. To setup our drone, we will modify the node.launch file under the same folder. After ` <rosparam command=”load” file=”$(arg config_yaml)” >` line, add `<remap from=”/mavros/vision_pose/pose” to=”/robot_pose” />`.

3. Now we have set those parameters, we can launch our ardupilot simulation(as per github) and can visualize tf tree using: `rosrun rqt_tf_tree rqt_tf_tree.`

*4. In the gazebo launch file, that is already uploaded on github, we have published a static transform from base_link to laser frame to have a connected tf-tree. 

#### To start with the cartographer-ros package:

To build and install the package, execute the following commands:

    sudo apt-get update

    sudo apt-get install -y python-wstool python-rosdep ninja-build stow

    cd ardupilot_ws
    wstool init src
    wstool merge -t src https://raw.githubusercontent.com/googlecartographer/cartographer_ros/master/cartographer_ros.rosinstall

    wstool update -t src
    src/cartographer/scripts/install_proto3.sh
    sudo rosdep init   
    rosdep update

    rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y
    cd ardupilot_ws/src
    git clone https://github.com/GT-RAIL/robot_pose_publisher.git
    For detailed instructions, https://ardupilot.org/dev/docs/ros-cartographer-slam.html
1. Now edit robot_pose_publised package as:
Modify the robot_pose_publisher.cpp file

    cd $HOME/ardupilot_ws/src/robot_pose_publisher/src
    gedit robot_pose_publisher.cpp

modify line 40 to look like below ("false" has been changed to "true")
`nh_priv.param<bool>("is_stamped", is_stamped, true);`

2. Edit the cartgrapher_ros package:
Create the cartographer_ros launch file:

    cd $HOME/ardupilot_ws/src/cartographer_ros/cartographer_ros/launch
    gedit cartographer.launch
Copy-paste the contents below into the file

    <?xml version="1.0"?>
       <launch>
          <param name="/use_sim_time" value="true" />
          <node name="cartographer_node"
                pkg="cartographer_ros"
                type="cartographer_node"
                args="-configuration_directory $(find cartographer_ros)/configuration_files -configuration_basename cartographer.lua"
                output="screen">
        <remap from=”odom” to “/mavros/local_position/odom” />
        <remap from=”imu” to “/mavros/imu/data” />
          </node>
          <node name="cartographer_occupancy_grid_node"
                pkg="cartographer_ros"
                type="cartographer_occupancy_grid_node" />
          <node name="robot_pose_publisher"
                pkg="robot_pose_publisher"
                type="robot_pose_publisher"
                respawn="false"
                output="screen" />
       </launch>


3: Edit cartographer configuration file, cartographer.lua:
<Currently under progress> 
  
    include "map_builder.lua"
    include "trajectory_builder.lua"
    options = {
      map_builder = MAP_BUILDER,
      trajectory_builder = TRAJECTORY_BUILDER,
      map_frame = "map",
      tracking_frame = "base_link",
      published_frame = "base_link",
      odom_frame = "odom",
      provide_odom_frame = true,
      publish_frame_projected_to_2d = false,
      use_odometry = false,
      use_nav_sat = false,
      use_landmarks = false,
      num_laser_scans = 1,
      num_multi_echo_laser_scans = 0,
      num_subdivisions_per_laser_scan = 1,
      num_point_clouds = 0,
      lookup_transform_timeout_sec = 0.2,
      submap_publish_period_sec = 0.3,
      pose_publish_period_sec = 5e-3,
      trajectory_publish_period_sec = 30e-3,
      rangefinder_sampling_ratio = 1.,
      odometry_sampling_ratio = 1.,
      fixed_frame_pose_sampling_ratio = 1.,
      imu_sampling_ratio = 1.,
      landmarks_sampling_ratio = 1.,
    }
    MAP_BUILDER.use_trajectory_builder_2d = true

    TRAJECTORY_BUILDER_2D.min_range = 0.05
    TRAJECTORY_BUILDER_2D.max_range = 8.
    TRAJECTORY_BUILDER_2D.missing_data_ray_length = 8.5
    TRAJECTORY_BUILDER_2D.use_imu_data = true
    TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 5e15
    TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 5e15
    TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
    TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.1
    TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 100.
    TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 100e100
    TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.2)
    -- for current lidar only 1 is good value
    TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1

    TRAJECTORY_BUILDER_2D.min_z = -0.5
    TRAJECTORY_BUILDER_2D.max_z = 0.5

    POSE_GRAPH.constraint_builder.min_score = 0.65
    POSE_GRAPH.constraint_builder.global_localization_min_score = 0.65
    POSE_GRAPH.optimization_problem.huber_scale = 1e2
    POSE_GRAPH.optimize_every_n_nodes = 35

    return options


### Archive:
Earlier documentations: https://github.com/snktshrma/Cartographer-documentation/tree/past_updates
