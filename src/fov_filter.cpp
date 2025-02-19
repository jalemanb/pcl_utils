#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/extract_indices.h>
#include <Eigen/Dense>
#include <chrono>  // For timing

constexpr float RAD2DEG = 180.0f / M_PI;
constexpr float DEG2RAD = M_PI / 180.0f;

// Function to check if a value is valid (not NaN or Inf)
inline bool isValidPoint(float x, float y, float z) {
    return std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
}

// Function to wrap an angle to [0, 2π]
inline float wrapTo2Pi(float angle) {
    while (angle < 0) angle += 2 * M_PI;
    while (angle >= 2 * M_PI) angle -= 2 * M_PI;
    return angle;
}

// Function to convert Cartesian to Spherical coordinates with NaN handling
bool cartesianToSpherical(float x, float y, float z, float& radius, float& polar, float& azimuth) {
    if (!isValidPoint(x, y, z)) {
        return false; // Return false if the input contains NaN
    }

    radius = std::sqrt(x * x + y * y + z * z);
    if (radius == 0.0f || !std::isfinite(radius)) {
        return false; // Avoid division by zero and NaN cases
    }

    polar = std::acos(z / radius);  // θ (Polar Angle)
    azimuth = std::atan2(y, x);     // φ (Azimuth Angle)

    // Wrap angles to [0, 2π] range
    polar = wrapTo2Pi(polar);
    azimuth = wrapTo2Pi(azimuth);

    return std::isfinite(polar) && std::isfinite(azimuth); // Ensure valid angles
}

// Function to check if an angle is in the correct range (handles wrap-around cases)
bool isAngleInRange(float angle, float min_angle, float max_angle) {
    if (min_angle <= max_angle) {
        return angle >= min_angle && angle <= max_angle;
    } else {
        // Handle wrap-around case (e.g., min_azimuth = 350°, max_azimuth = 10°)
        return (angle >= min_angle && angle <= 2 * M_PI) || (angle >= 0 && angle <= max_angle);
    }
}

// Function to get indices of points within specified polar & azimuth angle ranges
pcl::PointIndices filterSphericalIndices(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, 
    float min_polar_deg, float max_polar_deg, 
    float min_azimuth_deg, float max_azimuth_deg) 
{
    pcl::PointIndices indices;

    // Convert degrees to radians and wrap them to [0, 2π]
    float min_polar = wrapTo2Pi(DEG2RAD * min_polar_deg);
    float max_polar = wrapTo2Pi(DEG2RAD * max_polar_deg);
    float min_azimuth = wrapTo2Pi(DEG2RAD * min_azimuth_deg);
    float max_azimuth = wrapTo2Pi(DEG2RAD * max_azimuth_deg);

    // Parallelized loop over the point cloud
    std::vector<int> valid_indices;

    #pragma omp parallel
    {
        std::vector<int> local_indices;

        #pragma omp for nowait
        for (size_t i = 0; i < cloud->size(); ++i) {
            float radius, polar, azimuth;

            if (cartesianToSpherical(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z, radius, polar, azimuth)) {
                if (isAngleInRange(polar, min_polar, max_polar) && isAngleInRange(azimuth, min_azimuth, max_azimuth)) {
                    local_indices.push_back(i);
                }
            }
        }

        #pragma omp critical
        valid_indices.insert(valid_indices.end(), local_indices.begin(), local_indices.end());
    }

    indices.indices = std::move(valid_indices);
    return indices;
}

// Function to merge two pcl::PointIndices while avoiding duplicates
pcl::PointIndices mergePointIndices(const pcl::PointIndices& indices1, const pcl::PointIndices& indices2) {
    std::unordered_set<int> uniqueIndices; // Stores unique indices
    
    // Insert indices from the first set
    uniqueIndices.insert(indices1.indices.begin(), indices1.indices.end());
    
    // Insert indices from the second set
    uniqueIndices.insert(indices2.indices.begin(), indices2.indices.end());

    // Create the merged PointIndices
    pcl::PointIndices mergedIndices;
    mergedIndices.indices.assign(uniqueIndices.begin(), uniqueIndices.end());

    // Optional: Sort indices for consistency
    std::sort(mergedIndices.indices.begin(), mergedIndices.indices.end());

    return mergedIndices;
}

namespace pcl_utils
{
    class FOVFilter : public rclcpp::Node
    {
    public:
        explicit FOVFilter(const rclcpp::NodeOptions &options)
            : Node("index_filter_node", options)
        {
            declare_parameters();
            get_parameters();

            // Subscriber to the raw PointCloud2 topic
            cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                input_topic_, rclcpp::SensorDataQoS(),
                std::bind(&FOVFilter::cloud_callback, this, std::placeholders::_1));

            // Publisher for the filtered PointCloud2
            cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, 10);

            RCLCPP_INFO(this->get_logger(), "FOVFilter Node started! Using pcl::ExtractIndices.");

            indices = pcl::PointIndices::Ptr(new pcl::PointIndices());

        }

    private:

        void declare_parameters() {
            this->declare_parameter<std::string>("input_topic", "/rear/rslidar_points");
            this->declare_parameter<std::string>("output_topic", "/filtered_cloud");
            this->declare_parameter<float>("polar_min_lateral", 80.0);
            this->declare_parameter<float>("polar_max_lateral", 115.0);
            this->declare_parameter<float>("azimuth_min_lateral", 1.0);
            this->declare_parameter<float>("azimuth_max_lateral", 360.0);
            this->declare_parameter<float>("polar_min_front", 0.0);
            this->declare_parameter<float>("polar_max_front", 180.0);
            this->declare_parameter<float>("azimuth_min_front", 330.0);
            this->declare_parameter<float>("azimuth_max_front", 30.0);
        }

        void get_parameters() {
            input_topic_ = this->get_parameter("input_topic").as_string();
            output_topic_ = this->get_parameter("output_topic").as_string();
            polar_min_lateral_ = this->get_parameter("polar_min_lateral").as_double();
            polar_max_lateral_ = this->get_parameter("polar_max_lateral").as_double();
            azimuth_min_lateral_ = this->get_parameter("azimuth_min_lateral").as_double();
            azimuth_max_lateral_ = this->get_parameter("azimuth_max_lateral").as_double();
            polar_min_front_ = this->get_parameter("polar_min_front").as_double();
            polar_max_front_ = this->get_parameter("polar_max_front").as_double();
            azimuth_min_front_ = this->get_parameter("azimuth_min_front").as_double();
            azimuth_max_front_ = this->get_parameter("azimuth_max_front").as_double();
        }

        void cloud_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr input_cloud_msg)
        {
            if (input_cloud_msg->height <= 1)
            {
                RCLCPP_WARN(this->get_logger(), "Received unorganized point cloud. Skipping filtering.");
                return;
            }

            // 🔹 FIXED: Correct width and height
            int height = input_cloud_msg->width;  // 32 beams
            int width = input_cloud_msg->height;  // 1800 scan points
            size_t total_points = height * width;

            // Convert ROS2 PointCloud2 to PCL format
            pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>());
            pcl::fromROSMsg(*input_cloud_msg, *pcl_cloud);

            if (pcl_cloud->size() != total_points)
            {
                RCLCPP_ERROR(this->get_logger(), "Mismatch in PointCloud size: expected %zu, got %zu",
                             total_points, pcl_cloud->size());
                return;
            }

            auto start_time = std::chrono::steady_clock::now();

            // Front Indices
            pcl::PointIndices indices_front = filterSphericalIndices(pcl_cloud, polar_min_front_, polar_max_front_, azimuth_min_front_, azimuth_max_front_);
            // Lateral Indices
            pcl::PointIndices indices_lat = filterSphericalIndices(pcl_cloud, polar_min_lateral_, 
                                                                   polar_max_lateral_, 
                                                                   azimuth_min_lateral_, 
                                                                   azimuth_max_lateral_);
            // Merge Indices and Avoid Duplicates
            *indices = mergePointIndices(indices_front, indices_lat);

            // *indices = indices_front;
            extract.setIndices(indices);
            // 🔹 Delete the selected indices if true
            extract.setNegative(false); 
            // Indices Prepared 

            // Apply PCL ExtractIndices filter and measure execution time
            extract.setInputCloud(pcl_cloud);

            pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>());

            // Start timer
            extract.filter(*filtered_cloud);

            auto end_time = std::chrono::steady_clock::now();

            // Calculate execution time in microseconds
            auto duration_microseconds = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count();
            auto duration_milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();

            // Print execution time
            // RCLCPP_INFO(this->get_logger(), "Filter execution time: %ld µs (%ld ms)", duration_microseconds, duration_milliseconds);

            // Convert back to ROS2 message
            sensor_msgs::msg::PointCloud2 output_msg;
            pcl::toROSMsg(*filtered_cloud, output_msg);
            output_msg.header = input_cloud_msg->header; // Preserve timestamp and frame

            // Publish filtered point cloud
            cloud_pub_->publish(output_msg);
        }

        // Indices Extractor
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        pcl::PointIndices::Ptr indices;


        std::string input_topic_;
        std::string output_topic_;
        float polar_min_lateral_, polar_max_lateral_, azimuth_min_lateral_, azimuth_max_lateral_;
        float polar_min_front_, polar_max_front_, azimuth_min_front_, azimuth_max_front_;

        // ROS 2 Communication
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
    };

} // namespace pcl_utils

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(pcl_utils::FOVFilter)
