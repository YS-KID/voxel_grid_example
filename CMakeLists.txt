cmake_minimum_required(VERSION 3.5)
project(voxel_grid_example)

# 必要なパッケージを見つける
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

# 実行ファイルを追加する (ターゲット名を指定)
add_executable(voxel_grid_filter_node src/voxel_grid_node.cpp)

# 依存関係を指定する
ament_target_dependencies(voxel_grid_filter_node
  "rclcpp"
  "sensor_msgs"
  "pcl_conversions"
  "pcl_ros"
  "visualization_msgs"
)


# インストール設定
install(TARGETS voxel_grid_filter_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
