import os
import yaml
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

    isaac_config_path = os.path.join(get_package_share_directory("navigation"), "params", "isaacsim.yaml")
    with open(isaac_config_path, "r") as file:
        isaac_config = yaml.safe_load(file)

    gui_path = isaac_config.get("gui_path", "/root/assets/")
    install_path = isaac_config.get("install_path", "/isaacsim/")
    play_sim_on_start = str(isaac_config.get("play_sim_on_start", "true")).lower()  # Convert bool to string
    custom_args = isaac_config.get("custom_args", "--allow-root")


    nav2_bringup_launch_dir = os.path.join(get_package_share_directory("nav2_bringup"), "launch")

    rviz_config_dir = os.path.join(get_package_share_directory("navigation"), "rviz2", "global_navigation.rviz")

    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(get_package_share_directory("isaacsim"), "launch", 
                                              "run_isaacsim.launch.py")),
                                              launch_arguments={"gui": gui_path, 
                                                                "play_sim_on_start": play_sim_on_start,
                                                                "install_path": install_path,
                                                                "custom_args": custom_args}.items(),
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
