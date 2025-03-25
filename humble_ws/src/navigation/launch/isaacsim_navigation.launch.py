import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    use_sim_time = LaunchConfiguration("use_sim_time", default="True")

    map_dir = LaunchConfiguration(
        "map",
        default=os.path.join(
            get_package_share_directory("navigation"), "maps", "extended_outdoor_navigation.yaml"
        ),
    )

    param_dir = LaunchConfiguration(
        "params_file",
        default=os.path.join(
            get_package_share_directory("navigation"), "params", "global_navigation_params.yaml"
        ),
    )


    nav2_bringup_launch_dir = os.path.join(get_package_share_directory("nav2_bringup"), "launch")

    rviz_config_dir = os.path.join(get_package_share_directory("navigation"), "rviz2", "global_navigation.rviz")

    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(get_package_share_directory("isaacsim"), "launch", 
                                              "run_isaacsim.launch.py")),
                                              launch_arguments={"gui": "/home/dueiras/Personal/projects/agriculture_bot/assets/Collected_world_flat/world_flat.usd", 
                                                                "play_sim_on_start": "true",
                                                                "install_path": "/home/dueiras/isaacsim/isaac-sim-standalone@4.5.0-rc.36+release.19112.f59b3005.gl.linux-x86_64.release"}.items(),
            ),
            DeclareLaunchArgument("map", default_value=map_dir, description="Full path to map file to load"),
            DeclareLaunchArgument(
                "params_file", default_value=param_dir, description="Full path to param file to load"
            ),
            DeclareLaunchArgument(
                "use_sim_time", default_value="true", description="Use simulation (Omniverse Isaac Sim) clock if true"
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(nav2_bringup_launch_dir, "rviz_launch.py")),
                launch_arguments={"namespace": "", "use_namespace": "False","rviz_config": rviz_config_dir}.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([nav2_bringup_launch_dir, "/bringup_launch.py"]),
                launch_arguments={"map": map_dir, "use_sim_time": use_sim_time, "params_file": param_dir}.items(),
            ),

        ]
    )
