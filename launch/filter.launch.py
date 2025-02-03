import launch
import launch_ros.actions


def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='pcl_utils',  # Replace with your actual package name
            executable='pcl_utils_index_filter_node',
            name='filter_rear',
            output='screen',
            parameters=[
                {'input_topic': '/rear/rslidar_points'},
                {'output_topic': '/filtered_cloud'},
                {'polar_min_lateral': 80.0},
                {'polar_max_lateral': 115.0},
                {'azimuth_min_lateral': 1.0},
                {'azimuth_max_lateral': 360.0},
                {'polar_min_front': 0.0},
                {'polar_max_front': 180.0},
                {'azimuth_min_front': 330.0},
                {'azimuth_max_front': 30.0},
            ]
        )
    ])
