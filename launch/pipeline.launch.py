from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name="use_sim_time",
        default_value="True",
        description="Use simulation (Gazebo) clock if true",
    )

    container = ComposableNodeContainer(
        name='pointcloud_processing_container',
        namespace='filtered_pcl',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='pcl_utils',
                plugin='pcl_utils::FOVFilter',
                name='fov_filter_rear',
                parameters=[
                    {'input_topic': '/rear/rslidar_points'},
                    {'output_topic': '/filtered_cloud_rear'},
                    {'polar_min_lateral': 80.0},
                    {'polar_max_lateral': 115.0},
                    {'azimuth_min_lateral': 1.0},
                    {'azimuth_max_lateral': 360.0},
                    {'polar_min_front': 0.0},
                    {'polar_max_front': 180.0},
                    {'azimuth_min_front': 330.0},
                    {'azimuth_max_front': 30.0},
                    {'use_sim_time': use_sim_time},
                ]
            ),
            ComposableNode(
                package='pcl_utils',
                plugin='pcl_utils::FOVFilter',
                name='fov_filter_front',
                parameters=[
                    {'input_topic': '/front/rslidar_points'},
                    {'output_topic': '/filtered_cloud_front'},
                    {'polar_min_lateral': 80.0},
                    {'polar_max_lateral': 115.0},
                    {'azimuth_min_lateral': 1.0},
                    {'azimuth_max_lateral': 360.0},
                    {'polar_min_front': 0.0},
                    {'polar_max_front': 180.0},
                    {'azimuth_min_front': 330.0},
                    {'azimuth_max_front': 30.0},
                    {'use_sim_time': use_sim_time},
                ]
            ),
            ComposableNode(
                package="pcl_utils",
                plugin="pcl_utils::PointCloudConcatenator",
                name="pointcloud_concatenator",
                parameters=[{
                    "input_topic1": "/filtered_cloud_front",
                    "input_topic2": "/filtered_cloud_rear",
                    "output_topic": "/base/rslidar_points",
                    "output_frame": "base_link",
                    "use_sim_time": use_sim_time,
                }]
            ),
            ComposableNode(
                package='pcl_utils',
                plugin='pcl_utils::CropBox',
                name='crop_box_filter_node',
                parameters=[
                    {   'input_topic': '/base/rslidar_points',
                        'output_topic': '/cloud',
                        'xmin': -150.0,
                        'xmax':  150.0,
                        'ymin': -150.0,
                        'ymax':  150.0,
                        'zmin':  -0.45,
                        'zmax':  -0.20,
                        'negative': True,
                        "use_sim_time": use_sim_time,
                    }
                ]
            ),
            # ComposableNode(
            #     package='pcl_utils',
            #     plugin='pcl_utils::VoxelGridFilter',
            #     name='voxelgrid_filter_node',
            #     parameters=[
            #         {   'input_topic': '/cloud_no_floor',
            #             'output_topic': '/cloud',
            #             'voxel_size': 0.2,
            #             "use_sim_time": use_sim_time,
            #         }
            #     ]
            # )
        ],
        output='screen'
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(container)

    return ld
