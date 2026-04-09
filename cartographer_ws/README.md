sudo rm /opt/ros/jazzy/share/cartographer_ros/configuration_files/backpack_2d.lua
sudo nano /opt/ros/jazzy/share/cartographer_ros/configuration_files/backpack_2d.lua

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
  use_pose_extrapolator = true,
  use_odometry = false,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,                -- 关键：启用1路普通单回波激光雷达（T-MINI）
  num_multi_echo_laser_scans = 0,     -- 关键：关闭多回波（T-MINI无多回波）
  num_subdivisions_per_laser_scan = 1,-- 关键：细分次数设为1（适配普通雷达）
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

-- 启用2D建图
MAP_BUILDER.use_trajectory_builder_2d = true

-- 适配T-MINI雷达的核心参数
TRAJECTORY_BUILDER_2D.min_range = 0.08          -- T-MINI最小测距（0.08米）
TRAJECTORY_BUILDER_2D.max_range = 12.0          -- T-MINI最大测距（12米）
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 1.0  -- 缺失数据补全长度
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1  -- 累积帧数设为1（实时性更好）
TRAJECTORY_BUILDER_2D.use_imu_data = false           -- 无IMU则关闭（T-MINI无IMU）
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true  -- 启用在线扫描匹配

-- 优化建图精度（可选，适合T-MINI）
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.1
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 10.
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1e-1

-- 位姿图优化参数（避免地图漂移）
POSE_GRAPH.optimization_problem.huber_scale = 1e2
POSE_GRAPH.optimize_every_n_nodes = 30
POSE_GRAPH.constraint_builder.min_score = 0.65  -- 匹配阈值（过低易漂移，过高易断连）

return options

--------------------------------------------------------------------------

cd ~/cartographer_ws
colcon build --packages-select tmini_cartographer_py

cd ~/cartographer_ws
source install/setup.bash 
ros2 launch tmini_cartographer_py tmini_cartographer.launch.py

