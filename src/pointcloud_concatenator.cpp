
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_ros/transforms.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <rmw/qos_profiles.h>  // For rmw_qos_profile_*

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace pcl_utils {

class PointCloudConcatenator : public rclcpp::Node {
public:
  explicit PointCloudConcatenator(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
    : Node("pointcloud_concatenator", options),
      tf_buffer_(this->get_clock()),
      tf_listener_(tf_buffer_)
  {
    // Declare parameters (with default values)
    this->declare_parameter<std::string>("input_topic1", "/front/rslidar_points");
    this->declare_parameter<std::string>("input_topic2", "/rear/rslidar_points");
    this->declare_parameter<std::string>("output_topic", "/base/rslidar_points");
    this->declare_parameter<std::string>("output_frame", "base_link");

    // Retrieve parameters
    this->get_parameter("input_topic1", input_topic1_);
    this->get_parameter("input_topic2", input_topic2_);
    this->get_parameter("output_topic", output_topic_);
    this->get_parameter("output_frame", output_frame_);

    // Create a publisher (can still use rclcpp::QoS here)
    pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      output_topic_, 10 /* or rclcpp::SensorDataQoS() etc. */
    );

    // Prepare rmw QoS profiles for the subscribers
    // We'll base them on the sensor_data QoS, but you can customize as needed
    rmw_qos_profile_t sub_qos = rmw_qos_profile_sensor_data;
    sub_qos.depth = 10;  // example customization

    // Create the message_filters subscribers
    // NOTE: The constructor signature is: Subscriber(NodeType* node, const std::string& topic, rmw_qos_profile_t)
    sub1_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>(
      this, input_topic1_, sub_qos);

    sub2_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>(
      this, input_topic2_, sub_qos);

    // Initialize the synchronizer (ApproximateTime with queue size 10)
    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
      SyncPolicy(10), *sub1_, *sub2_
    );
    sync_->registerCallback(
      std::bind(&PointCloudConcatenator::syncCallback, this,
                std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(
      this->get_logger(),
      "PointCloudConcatenator initialized:\n"
      "  input_topic1: %s\n"
      "  input_topic2: %s\n"
      "  output_topic: %s\n"
      "  output_frame: %s",
      input_topic1_.c_str(), input_topic2_.c_str(),
      output_topic_.c_str(), output_frame_.c_str()
    );
  }

private:
  // ApproximateTime sync for two PointCloud2 messages
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
      sensor_msgs::msg::PointCloud2,
      sensor_msgs::msg::PointCloud2>;

  // Synchronized callback
  void syncCallback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud1,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud2)
  {

    // Transform to the desired output_frame_ using the tf_buffer_
    sensor_msgs::msg::PointCloud2 pcl_tf1, pcl_tf2, pcl_out;

    try {
      pcl_ros::transformPointCloud(output_frame_, *cloud1, pcl_tf1, tf_buffer_);
      pcl_ros::transformPointCloud(output_frame_, *cloud2, pcl_tf2, tf_buffer_);
    } catch (tf2::TransformException &ex) {
      RCLCPP_WARN(this->get_logger(), "Transform failed: %s", ex.what());
      return;
    }

    // COncatenate both pointclouds
    pcl::concatenatePointCloud(pcl_tf1, pcl_tf2, pcl_out);
    pcl_out.header.frame_id = output_frame_;
    pcl_out.header.stamp    = this->now();

    // Publish
    pub_->publish(pcl_out);

    // RCLCPP_INFO(this->get_logger(),"Published concatenated pointcloud");
  }

  // Parameters
  std::string input_topic1_;
  std::string input_topic2_;
  std::string output_topic_;
  std::string output_frame_;

  // Publisher
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;

  // Subscribers + sync
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>> sub1_;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>> sub2_;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

  // TF
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

}  // namespace pcl_utils

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(pcl_utils::PointCloudConcatenator)
