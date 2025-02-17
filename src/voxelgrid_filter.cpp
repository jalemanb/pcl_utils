#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

namespace pcl_utils {

class VoxelGridFilter : public rclcpp::Node {
public:
  explicit VoxelGridFilter(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
    : Node("voxel_grid_filter", options), voxel_filter_()
  {
    // Declare parameters
    this->declare_parameter<std::string>("input_topic", "/lidar/points");
    this->declare_parameter<std::string>("output_topic", "/filtered/points");
    this->declare_parameter<double>("voxel_size", 0.1);

    // Retrieve parameters
    this->get_parameter("input_topic", input_topic_);
    this->get_parameter("output_topic", output_topic_);
    this->get_parameter("voxel_size", voxel_size_);

    // Configure voxel filter
    voxel_filter_.setLeafSize(static_cast<float>(voxel_size_), static_cast<float>(voxel_size_), static_cast<float>(voxel_size_));

    // Create a subscriber and publisher
    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, rclcpp::SensorDataQoS(),
      std::bind(&VoxelGridFilter::callback, this, std::placeholders::_1));
    pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, 10);

    RCLCPP_INFO(this->get_logger(), "VoxelGridFilter initialized with:");
    RCLCPP_INFO(this->get_logger(), "  Input Topic: %s", input_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "  Output Topic: %s", output_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "  Voxel Size: %.3f", voxel_size_);
  }

private:
  void callback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud) {
    if (!cloud) {
      RCLCPP_WARN(this->get_logger(), "Received null cloud message!");
      return;
    }

    // Convert ROS message to PCL format
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*cloud, *pcl_cloud);

    // Apply voxel grid filter
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    voxel_filter_.setInputCloud(pcl_cloud);
    voxel_filter_.filter(*pcl_filtered);

    // Convert back to ROS message
    sensor_msgs::msg::PointCloud2 output;
    pcl::toROSMsg(*pcl_filtered, output);
    output.header = cloud->header;

    // Publish
    pub_->publish(output);
    // RCLCPP_INFO(this->get_logger(), "Published downsampled pointcloud with %zu points", pcl_filtered->size());
  }

  // Parameters
  std::string input_topic_;
  std::string output_topic_;
  double voxel_size_;

  // Voxel Grid Filter
  pcl::VoxelGrid<pcl::PointXYZ> voxel_filter_;

  // Subscriber and Publisher
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
};

}  // namespace pcl_utils

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(pcl_utils::VoxelGridFilter)
