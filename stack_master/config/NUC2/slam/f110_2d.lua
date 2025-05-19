include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "imu", -- IMUがある場合はIMUのフレームを指定するのが一般的
  published_frame = "base_link",  -- map_frame -> published_frameをtfでpublishする
  odom_frame = "odom",
  provide_odom_frame = false, -- Trueにすると map_frame -> odom_frameをtfでpublishする
  publish_frame_projected_to_2d = true, -- 出力される姿勢のz軸、roll, pitchを0にする
  use_odometry = true,
  use_nav_sat = false,
  use_landmarks = false, -- ランドマーク（既知の目印）を使うかどうか
  num_laser_scans = 1, -- sensor_msgs/LaserScan型のトピック数
  num_multi_echo_laser_scans = 0, -- マルチエコーを使うトピック数（室内は反射ノイズが少ないので使わなくていい）
  num_subdivisions_per_laser_scan = 1, -- 1回のスキャンを何分割するか、スキャン中に移動すると位置がずれるので分割して補正する
  num_point_clouds = 0, -- sensor_msgs/PointCloud2型のトピック数
  lookup_transform_timeout_sec = 0.2, -- tfの変換を待つ時間
  submap_publish_period_sec = 0.3, -- サブマップをpublishする周期、精度に影響はない
  pose_publish_period_sec = 5e-3, -- ロボットの姿勢をpublishする周期、精度に影響はない
  trajectory_publish_period_sec = 30e-3, -- trajectoryをpublishする周期、精度に影響はない
  rangefinder_sampling_ratio = 1., -- LaserScanのサンプリング比率、1.0で全てのデータを使う
  odometry_sampling_ratio = 1.0, -- odometryのサンプリング比率
  fixed_frame_pose_sampling_ratio = 1., -- GNSSなどの絶対位置情報のサンプリング比率
  imu_sampling_ratio = 1., -- IMUのサンプリング比率
  landmarks_sampling_ratio = 1., -- ランドマークのサンプリング比率
  publish_to_tf = true, -- tfをpublishするかどうか
  publish_tracked_pose = true, -- トラッキングした姿勢をpublishするかどうか
}

MAP_BUILDER.use_trajectory_builder_2d = true
MAP_BUILDER.use_trajectory_builder_3d = false
MAP_BUILDER.num_background_threads = 3.0
MAP_BUILDER.pose_graph = POSE_GRAPH

TRAJECTORY_BUILDER_2D.use_imu_data = true
TRAJECTORY_BUILDER_2D.max_range = 25.0
TRAJECTORY_BUILDER_2D.min_range = 0.1

-- might be able to optimize these parameters
-- see: http://google-cartographer-ros.readthedocs.io/en/latest/tuning.html
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 100
POSE_GRAPH.optimize_every_n_nodes = 20

TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 0.2 * TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight

TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 0.2 * TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight

POSE_GRAPH.optimization_problem.odometry_rotation_weight = 0
POSE_GRAPH.optimization_problem.odometry_translation_weight = 0

return options