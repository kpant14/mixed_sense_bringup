import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource

def generate_launch_description():

    xrce_gps_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('px4_gps'), 'launch', 'px4_gps_xrce.launch.py')),
        launch_arguments={
            'px4_ns': "px4_1",
            'gz_world_name': "swarm",
            'gz_model_name': "x500_1",
            'gz_spoofer_model_name': "spoofer",
            'gps_delay': '0.0'
        }.items(),
    )
    
   
    return LaunchDescription([
        xrce_gps_bridge,
    ])
