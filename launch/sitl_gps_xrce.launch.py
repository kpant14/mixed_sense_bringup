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
            'gz_world_name': "AbuDhabi",
            'gz_model_name': "x500_1",
            'gz_spoofer_model_name': "spoofer",
            'gps_delay': '0.0'
        }.items(),
    )

    ros_xrce_agent = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('micro_ros_agent'), 'launch', 'micro_ros_agent_launch.py')),
    )
    
    offboard_detector = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('offboard_detector'),'launch', 'offboard_detector_sitl.launch.py')),
            launch_arguments={
                'px4_ns': "px4_1",
                'gz_world_name': "AbuDhabi",
                'gz_model_name': "x500_1",
            }.items(),
    )

    gnss_attack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gnss_meaconing_attack'),'launch', 'launch_spoofing_attack.launch.py')),
            launch_arguments={
                'px4_ns': "px4_1",
                'gz_world_name': "AbuDhabi",
            }.items(),
    )

    foxglove_studio = ExecuteProcess(cmd=["foxglove-studio"])
    foxglove_bridge = IncludeLaunchDescription(XMLLaunchDescriptionSource(
        os.path.join(
        get_package_share_directory("mixed_sense_bringup"),
        "launch/foxglove_bridge.launch",))
    )

    return LaunchDescription([
        xrce_gps_bridge,
        ros_xrce_agent,
        offboard_detector,
        gnss_attack, 
        foxglove_bridge,
        foxglove_studio,
    ])
