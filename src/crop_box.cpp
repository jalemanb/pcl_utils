#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/filters/crop_box.h>
#include <pcl_conversions/pcl_conversions.h>
#include <Eigen/Dense>
#include <pcl/common/common.h>  // For getMinMax3D()

namespace pcl_utils
{
    class CropBox : public rclcpp::Node
    {
    public:
        explicit CropBox(const rclcpp::NodeOptions &options)
            : Node("crop_box_node", options)
        {
            // Declare parameters with default values
            this->declare_parameter("input_topic", "input_cloud");
            this->declare_parameter("output_topic", "filtered_cloud");
            this->declare_parameter("xmin", -1.0);
            this->declare_parameter("ymin", -1.0);
            this->declare_parameter("zmin", -1.0);
            this->declare_parameter("xmax", 1.0);
            this->declare_parameter("ymax", 1.0);
            this->declare_parameter("zmax", 1.0);
            this->declare_parameter("negative", false);

            // Get parameter values
            this->get_parameter("input_topic", input_topic_);
            this->get_parameter("output_topic", output_topic_);
            this->get_parameter("xmin", xmin_);
            this->get_parameter("ymin", ymin_);
            this->get_parameter("zmin", zmin_);
            this->get_parameter("xmax", xmax_);
            this->get_parameter("ymax", ymax_);
            this->get_parameter("zmax", zmax_);
            this->get_parameter("negative", negative_);

            // Subscriber & Publisher
            cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                input_topic_, rclcpp::SensorDataQoS(),
                std::bind(&CropBox::cloud_callback, this, std::placeholders::_1));

            cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, rclcpp::QoS(10).reliable());
            RCLCPP_INFO(this->get_logger(), "CropBox initialized! Subscribing to: %s, Publishing to: %s", input_topic_.c_str(), output_topic_.c_str());
        }

    private:
        void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr input_cloud_msg)
        {
            pcl::PCLPointCloud2::Ptr pcl_cloud(new pcl::PCLPointCloud2());
            pcl_conversions::toPCL(*input_cloud_msg, *pcl_cloud);

            // Use PCL's getMinMax3D to find min/max values efficiently
            pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_cloud(new pcl::PointCloud<pcl::PointXYZ>());
            pcl::fromPCLPointCloud2(*pcl_cloud, *xyz_cloud);
            pcl::PointXYZ min_pt, max_pt;
            pcl::getMinMax3D(*xyz_cloud, min_pt, max_pt);
            // Print the minimum Y value
            RCLCPP_INFO(this->get_logger(), "Min Z value: %f", min_pt.z);


            pcl::CropBox<pcl::PCLPointCloud2> crop_filter;
            crop_filter.setInputCloud(pcl_cloud);

            Eigen::Vector4f min_point(xmin_, ymin_, zmin_, 1.0);
            Eigen::Vector4f max_point(xmax_, ymax_, zmax_, 1.0);
            crop_filter.setMin(min_point);
            crop_filter.setMax(max_point);
            crop_filter.setNegative(negative_);  // Keep points inside or outside

            pcl::PCLPointCloud2::Ptr cropped_cloud(new pcl::PCLPointCloud2());
            crop_filter.filter(*cropped_cloud);

            // Convert back to ROS message
            sensor_msgs::msg::PointCloud2 output_msg;
            pcl_conversions::fromPCL(*cropped_cloud, output_msg);
            output_msg.header = input_cloud_msg->header;  // Preserve timestamp and frame

            cloud_pub_->publish(output_msg);
        }

        // ROS 2 Parameters
        std::string input_topic_;
        std::string output_topic_;
        double xmin_, ymin_, zmin_, xmax_, ymax_, zmax_;
        bool negative_;

        // ROS 2 Communication
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
    };
} // namespace pcl_utils

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(pcl_utils::CropBox)
