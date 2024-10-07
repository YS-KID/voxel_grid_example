#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>

class VoxelGridMarkerNode : public rclcpp::Node
{
public:
  VoxelGridMarkerNode() : Node("voxel_grid_marker_node")
  {
    // サブスクライバ: 点群データを受信
    point_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/livox/lidar_192_168_1_100", 10, std::bind(&VoxelGridMarkerNode::pointCloudCallback, this, std::placeholders::_1));

    // パブリッシャ: マーカーを送信
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/voxel_grid_markers", 10);
  }

private:
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);

    // Voxel Gridフィルタを適用
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
    voxel_grid.setInputCloud(cloud);
    voxel_grid.setLeafSize(0.05f, 0.05f, 0.05f); // ボクセルサイズを設定
    voxel_grid.filter(*filtered_cloud);

    // マーカーアレイを作成
    visualization_msgs::msg::MarkerArray marker_array;
    int id = 0;

    for (const auto &point : filtered_cloud->points)
    {
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = msg->header.frame_id;  // フレームIDを確認
      marker.header.stamp = this->now();
      marker.ns = "voxel_grid";
      marker.id = id++;
      marker.type = visualization_msgs::msg::Marker::CUBE;
      marker.action = visualization_msgs::msg::Marker::ADD;

      // ボックスの位置を設定
      marker.pose.position.x = point.x;
      marker.pose.position.y = point.y;
      marker.pose.position.z = point.z;
      marker.pose.orientation.w = 1.0;

      // ボックスのサイズを設定
      marker.scale.x = 0.2;  // voxel gridのリーフサイズに対応
      marker.scale.y = 0.2;
      marker.scale.z = 0.2;

      // 色を設定（例: 赤）
      marker.color.r = 1.0f;
      marker.color.g = 0.0f;
      marker.color.b = 0.0f;
      marker.color.a = 1.0f;  // 不透明

      // 一定の寿命を設定
      marker.lifetime = rclcpp::Duration::from_seconds(5.0);

      marker_array.markers.push_back(marker);
    }

    // マーカーをパブリッシュ
    marker_pub_->publish(marker_array);
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VoxelGridMarkerNode>());
  rclcpp::shutdown();
  return 0;
}
