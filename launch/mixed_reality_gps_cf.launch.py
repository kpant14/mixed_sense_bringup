import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description(): 
    fdir_mixed = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('mas_fdir'), 'launch', 'fdir_mixed_launch.py')),
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
                {'gz_entity': 'cf_1'},
                {'gz_world_name': "AbuDhabiSwarm"},
                {'mocap_rigid_body': 'cf1'},
            ]
    ) 
    mocap_gz_bridge_2 = Node(
        package='mixed_sense_bringup',
        executable='mocap_gz_bridge',
        name='mocap_gz_bridge',
        parameters=[
                {'gz_entity': 'cf_2'},
                {'gz_world_name': "AbuDhabiSwarm"},
                {'mocap_rigid_body': 'cf2'},
            ]
    ) 
    mocap_gz_bridge_3 = Node(
        package='mixed_sense_bringup',
        executable='mocap_gz_bridge',
        name='mocap_gz_bridge',
        parameters=[
                {'gz_entity': 'cf_3'},
                {'gz_world_name': "AbuDhabiSwarm"},
                {'mocap_rigid_body': 'cf3'},
            ]
    ) 

    mocap_gz_bridge_4 = Node(
        package='mixed_sense_bringup',
        executable='mocap_gz_bridge',
        name='mocap_gz_bridge',
        parameters=[
                {'gz_entity': 'cf_4'},
                {'gz_world_name': "AbuDhabiSwarm"},
                {'mocap_rigid_body': 'cf4'},
            ]
    ) 

    return LaunchDescription([
        fdir_mixed,
        entity_service,
        mocap_gz_bridge_1,
        mocap_gz_bridge_2,
        mocap_gz_bridge_3,
        mocap_gz_bridge_4,
        qualisys_mocap,
    ])
