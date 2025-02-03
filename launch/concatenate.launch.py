from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    return LaunchDescription([
        ComposableNodeContainer(
            name="pointcloud_container",
            namespace="",
            package="rclcpp_components",
            executable="component_container",
            composable_node_descriptions=[
                ComposableNode(
                    package="pcl_utils",
                    plugin="pcl_utils::PointCloudConcatenator",
                    name="pointcloud_concatenator",
                    parameters=[{
                        "input_topic1": "/front/rslidar_points",
                        "input_topic2": "/rear/rslidar_points",
                        "output_topic": "/base/rslidar_points",
                        "output_frame": "base_link",
                        "use_sim_time": True,
                    }]
                )
            ],
            output="screen"
        )
    ])

