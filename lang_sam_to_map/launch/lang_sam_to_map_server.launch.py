from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

import os

def generate_launch_description():
    # RViz起動オプション
    use_rviz = LaunchConfiguration('use_rviz')
    
    declare_arg_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Set "true" to launch rviz.')

    # パッケージ共有ディレクトリ取得
    rs_pkg = get_package_share_directory('realsense2_camera')
    pkg_dir = get_package_share_directory('lang_sam_to_map')

    load_nodes = GroupAction(
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(rs_pkg, 'launch', 'rs_launch.py')
                ),
                launch_arguments={
                    'depth.enable': 'true',
                    'color.enable': 'true',
                    'enable_depth': 'true', 
                    'align_depth.enable': 'true',
                    'pointcloud.enable': 'false',
                    'pointcloud.ordered_pc': 'false', 
                }.items()
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_dir, 'launch', 'lang_sam_to_map_wo_camera.launch.py')
                ),
                launch_arguments={
                    'use_rviz': use_rviz
                }.items()
            )
        ]
    )
    ld = LaunchDescription()
    ld.add_action(declare_arg_use_rviz)
    ld.add_action(load_nodes)
    return ld
