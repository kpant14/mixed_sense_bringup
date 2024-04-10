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
            'gz_args': '-r ' + os.path.join(get_package_share_directory('mixed_sense_bringup'), 'worlds', 'AbuDhabi_MR.sdf')
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
        package='mixed_sense_bringup',
        executable='mocap_gz_bridge',
        name='mocap_gz_bridge',
        parameters=[
                {'gz_entity': 'cf1'},
                {'mocap_rigid_body': 'cf1'},
            ]
    ) 


    return LaunchDescription([
        gz_sim,
        entity_service,
        mocap_gz_bridge,
        qualisys_mocap,
    ])
