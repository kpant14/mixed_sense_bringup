import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description(): 
    xrce_gps_bridge_1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('px4_gps'), 'launch', 'px4_gps_xrce.launch.py')),
        launch_arguments={
            'px4_ns': "px4_1",
            'gz_world_name': "AbuDhabiSwarm",
            'gz_model_name': "x500_1",
            'gz_spoofer_model_name': "spoofer",
            'gps_delay': '0.0'
        }.items(),
    )
    xrce_gps_bridge_2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('px4_gps'), 'launch', 'px4_gps_xrce.launch.py')),
        launch_arguments={
            'px4_ns': "px4_2",
            'gz_world_name': "AbuDhabiSwarm",
            'gz_model_name': "x500_2",
            'gz_spoofer_model_name': "spoofer",
            'gps_delay': '0.0'
        }.items(),
    )
    
    entity_service = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/world/AbuDhabiSwarm/set_pose@ros_gz_interfaces/srv/SetEntityPose']
    )

    qualisys_mocap = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('qualisys_mocap'), 'launch', 'qualisys_mocap_tracking.launch.py')),
        launch_arguments={
            'mocap_server_ip': "192.168.123.2",
        }.items(),
    )

    mocap_gz_bridge_1 = Node(
        package='mixed_sense_bringup',
        executable='mocap_gz_bridge',
        name='mocap_gz_bridge',
        parameters=[
                {'gz_entity': 'cf1'},
                {'gz_world_name': "AbuDhabiSwarm"},
                {'mocap_rigid_body': 'cf1'},
            ]
    ) 
    mocap_gz_bridge_2 = Node(
        package='mixed_sense_bringup',
        executable='mocap_gz_bridge',
        name='mocap_gz_bridge',
        parameters=[
                {'gz_entity': 'cf2'},
                {'gz_world_name': "AbuDhabiSwarm"},
                {'mocap_rigid_body': 'cf2'},
            ]
    ) 
    mocap_gz_bridge_3 = Node(
        package='mixed_sense_bringup',
        executable='mocap_gz_bridge',
        name='mocap_gz_bridge',
        parameters=[
                {'gz_entity': 'cf3'},
                {'gz_world_name': "AbuDhabiSwarm"},
                {'mocap_rigid_body': 'cf3'},
            ]
    ) 

    return LaunchDescription([
        xrce_gps_bridge_1,
        xrce_gps_bridge_2,
        entity_service,
        mocap_gz_bridge_1,
        mocap_gz_bridge_2,
        mocap_gz_bridge_3,
        qualisys_mocap,
    ])
