# boxcrop.launch.py

from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    # Create a container to run the composable node
    container = ComposableNodeContainer(
        name='crop_box_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',  # or 'component_container'
        composable_node_descriptions=[
            ComposableNode(
                package='pcl_utils',
                plugin='pcl_utils::CropBox',
                name='crop_box_filter_node',
                # Example remappings:
                # remappings=[('input', '/input_cloud'), ('output', '/filtered_cloud')],
                parameters=[
                    {   'input_topic': '/rear/rslidar_points',
                        'xmin': -0.3,
                        'xmax':  0.3,
                        'ymin': -0.3,
                        'ymax':  0.3,
                        'zmin':  -0.3,
                        'zmax':  0.3,
                        'negative': True,
                        "use_sim_time": True,
                    }
                ]
            )
        ],
        output='screen'
    )

    return LaunchDescription([container])
