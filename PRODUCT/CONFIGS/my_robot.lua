include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  
  -- [[ SYSTEM FRAMES & TRACKING ]]
  map_frame = "map",                                -- The ROS frame ID of the map, used to publish poses
  tracking_frame = "imu_link",                      -- The ROS frame ID of the frame that is tracked by the SLAM algorithm
  published_frame = "base_link",                    -- The ROS frame ID to use as the child frame for publishing poses
  odom_frame = "odom",                              -- The ROS frame ID to use for publishing odometry
  provide_odom_frame = false,                       -- If true, publishes the local, non-loop-closed pose as odom_frame
  publish_frame_projected_to_2d = false,            -- If true, publishes the published_frame projected to the ground (2D)
  use_pose_extrapolator = true,                     -- If true, uses the pose extrapolator to predict the next pose
  
  -- [[ SENSOR USAGE ]]
  use_odometry = false,                             -- If true, subscribes to nav_msgs/Odometry on the "odom" topic
  use_nav_sat = false,                              -- If true, subscribes to sensor_msgs/NavSatFix on "fix" topic
  use_landmarks = false,                            -- If true, subscribes to cartographer_ros_msgs/LandmarkList
  num_laser_scans = 1,                              -- Number of laser scan topics to subscribe to (sensor_msgs/LaserScan)
  num_multi_echo_laser_scans = 0,                   -- Number of multi-echo laser scan topics
  num_subdivisions_per_laser_scan = 1,              -- Number of point clouds to split each laser scan into
  num_point_clouds = 0,                             -- Number of point cloud topics to subscribe to
  
  -- [[ TIMEOUTS & SAMPLING ]]
  lookup_transform_timeout_sec = 0.2,               -- Timeout in seconds for looking up transforms using tf2
  submap_publish_period_sec = 0.3,                  -- Interval in seconds to publish the submap list
  pose_publish_period_sec = 5e-3,                   -- Interval in seconds to publish poses
  trajectory_publish_period_sec = 30e-3,            -- Interval in seconds to publish trajectory markers
  rangefinder_sampling_ratio = 1.,                  -- Fixed ratio sampling for rangefinder messages (1.0 = use all)
  odometry_sampling_ratio = 1.,                     -- Fixed ratio sampling for odometry messages
  fixed_frame_pose_sampling_ratio = 1.,             -- Fixed ratio sampling for fixed frame messages
  imu_sampling_ratio = 1.,                          -- Fixed ratio sampling for IMU messages
  landmarks_sampling_ratio = 1.,                    -- Fixed ratio sampling for landmark messages
}

MAP_BUILDER.use_trajectory_builder_2d = true        -- Enables the 2D SLAM trajectory builder

-- ==========================================================
-- [[ LOCAL SLAM ]]
-- ==========================================================
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 2               -- Number of messages to accumulate into a single point cloud
TRAJECTORY_BUILDER_2D.min_range = 0.1                              -- Minimum valid range for the sensor in meters
TRAJECTORY_BUILDER_2D.max_range = 12.0                             -- Maximum valid range for the sensor in meters
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true  -- Solves online scan matching first before Ceres
TRAJECTORY_BUILDER_2D.use_imu_data = true                          -- Whether to use IMU data for 2D trajectory building
TRAJECTORY_BUILDER_2D.imu_gravity_time_constant = 1.0              -- Time constant for the IMU gravity vector observer

-- THE "WALKING" BALANCE (Weights)
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 2000.  -- Weight for the scan matching against the map features
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 1.        -- Weight for trusting the prior (IMU/Odom) translation
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 10.          -- Weight for trusting the prior (IMU/Odom) rotation

-- SCAN MATCHER WINDOW
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.2             -- Linear search window size in meters
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(15.)  -- Angular search window size in radians
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 10.    -- Weight of translation deviation from prior
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1e-1      -- Weight of rotation deviation from prior

-- ==========================================================
-- [[ GLOBAL SLAM (POSE GRAPH) ]]
-- ==========================================================
POSE_GRAPH.optimize_every_n_nodes = 10         -- Number of nodes after which to perform global loop closure optimization
POSE_GRAPH.constraint_builder.min_score = 0.5  -- Minimum score threshold for considering a loop closure match valid


-- ==========================================================
-- [[ 2.0 PARAMS ]]
-- ==========================================================
TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.01
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 5.
POSE_GRAPH.constraint_builder.max_constraint_distance = 10.0


return options
