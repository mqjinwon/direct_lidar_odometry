from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition   
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    current_pkg = FindPackageShare('direct_lidar_odometry')

    # Set default arguments
    ns = LaunchConfiguration('robot_namespace', default='robot')
    
    rviz = LaunchConfiguration('rviz', default='false')
    pointcloud_topic = LaunchConfiguration('pointcloud_topic', default='/ouster/points')
    imu_topic = LaunchConfiguration('imu_topic', default='/ouster/imu')
    
    # Define arguments
    declare_rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value=rviz,
        description='Launch RViz'
    )
    declare_pointcloud_topic_arg = DeclareLaunchArgument(
        'pointcloud_topic',
        default_value=pointcloud_topic,
        description='Pointcloud topic name'
    )
    declare_imu_topic_arg = DeclareLaunchArgument(
        'imu_topic',
        default_value=imu_topic,
        description='IMU topic name'
    )

    # Load parameters
    dlo_yaml_path = PathJoinSubstitution([current_pkg, 'cfg', 'dlo.yaml'])
    dlo_params_yaml_path = PathJoinSubstitution([current_pkg, 'cfg', 'params.yaml'])
    
    # DLO Odometry Node
    dlo_odom_node = Node(
        package='direct_lidar_odometry',
        executable='dlo_odom_node',
        output='screen',
        parameters=[dlo_yaml_path, dlo_params_yaml_path],
        # namespace=ns,
        remappings=[
            ('pointcloud', pointcloud_topic),
            ('imu', imu_topic),
            ('odom', 'dlo/odom_node/odom'),
            ('pose', 'dlo/odom_node/pose'),
            ('kfs', 'dlo/odom_node/odom/keyframe'),
            ('keyframe', 'dlo/odom_node/pointcloud/keyframe'),
        ],
    )

    # DLO Mapping Node
    dlo_map_node = Node(
        package='direct_lidar_odometry',
        executable='dlo_map_node',
        output='screen',
        parameters=[dlo_yaml_path, dlo_params_yaml_path],
        # namespace=ns,
        remappings=[
            ('keyframe', 'dlo/odom_node/pointcloud/keyframe'),
            ('map', 'dlo/map_node/map')
        ],
    )

    # Rviz node
    rviz_config_path = PathJoinSubstitution([current_pkg, 'launch', 'dlo.rviz'])
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='dlo_rviz',
        arguments=['-d', rviz_config_path],
        output='screen',
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    return LaunchDescription([
        declare_rviz_arg,
        declare_pointcloud_topic_arg,
        declare_imu_topic_arg,
        dlo_odom_node,
        dlo_map_node,
        rviz_node
    ])
