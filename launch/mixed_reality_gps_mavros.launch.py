import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description(): 
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': '-r ' + os.path.join(get_package_share_directory('mixed_sense'), 'worlds', 'AbuDhabi_MR.sdf')
        }.items(),
    )
    
    mavros_gps_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('px4_gps'), 'launch', 'px4_gps_mavros.launch.py')),
        launch_arguments={
            'gz_world_name': "AbuDhabi",
            'gz_model_name': "x500_1",
            'gz_spoofer_model_name': "spoofer",
            'gps_delay': '0.0'
        }.items(),
    )

    entity_service = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/world/AbuDhabi/set_pose@ros_gz_interfaces/srv/SetEntityPose']
    )

    qualisys_mocap = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('qualisys_mocap'), 'launch', 'qualisys_mocap_tracking.launch.py')),
        launch_arguments={
            'mocap_server_ip': "192.168.123.2",
        }.items(),
    )

    mocap_gz_bridge = Node(
           package='mixed_sense',
           executable='mocap_gz_bridge',
           name='mocap_gz_bridge',
           parameters = [
                {'mocap_rigid_body': 'drone162'},
                {'gz_entity': 'x500_1'},
            ]
    ) 
    
    onboard_camera_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=['/camera'],
        output='screen'
    )

    return LaunchDescription([
        gz_sim,
        mavros_gps_bridge,
        entity_service,
        mocap_gz_bridge,
        qualisys_mocap,
        onboard_camera_bridge,
    ])
