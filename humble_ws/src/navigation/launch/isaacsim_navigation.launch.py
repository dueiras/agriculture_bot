import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessIO

def execute_second_node_if_condition_met(event, second_node_action):
    """
    Function taken from NVIDIA as a way to just launch ROS is isaacsim loads
    Otherwise nav2 goes time out
    """
    output = event.text.decode().strip()
    # Look for fully loaded message from Isaac Sim. Only applicable in Gui mode.
    if "Stage loaded and simulation is playing." in output:
        # Log a message indicating the condition has been met
        print("Condition met, launching the second node.")
        
        # If Nav2 takes additional time to initialize, uncomment the lines below to add a delay of 10 seconds (or any desired duration) before launching the second_node_action
        import time
        time.sleep(10)
        return second_node_action

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

    goal_text_file = LaunchConfiguration(
        "goal_text_file_path",
        default=os.path.join(get_package_share_directory("navigation"), "params", "goals.txt"),
    )

    nav2_bringup_launch_dir = os.path.join(get_package_share_directory("nav2_bringup"), "launch")

    rviz_config_dir = os.path.join(get_package_share_directory("navigation"), "rviz2", "global_navigation.rviz")

    ld_automatic_goal = Node(
                        name="set_navigation_goals",
                        package="navigation",
                        executable="navigate_through_poses",
                        parameters=[
                            {
                                "iteration_count": 3,
                                "action_server_name": "navigate_through_poses",
                                "obstacle_search_distance_in_meters": 0.2,
                                "goal_text_file_path": goal_text_file,
                                "initial_pose": [2.4, -1.0, 0.0, 0.0, 0.0, 0.0, 1.0],
                            }
                        ],
                        output="screen",
                    )

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
            Node(
                name="set_navigation_goals",
                package="navigation",
                executable="navigate_through_poses",
                parameters=[
                    {
                        "iteration_count": 3,
                        "action_server_name": "navigate_through_poses",
                        "obstacle_search_distance_in_meters": 0.2,
                        "goal_text_file_path": goal_text_file,
                        "initial_pose": [2.4, -1.0, 0.0, 0.0, 0.0, 0.0, 1.0],
                    }
                ],
                output="screen",
            ),
            # Launch automatic goal generator node when Isaac Sim has finished loading.
            RegisterEventHandler(
                OnProcessIO(
                    on_stdout=lambda event: execute_second_node_if_condition_met(event, ld_automatic_goal)
                )
            ),

        ]
    )
