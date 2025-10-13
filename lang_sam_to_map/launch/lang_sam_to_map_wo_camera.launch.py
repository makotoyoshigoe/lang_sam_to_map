from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

import os

def generate_launch_description():
    use_rviz = LaunchConfiguration('use_rviz')
    
    declare_arg_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Set "true" to launch rviz.')
    
    # パッケージ共有ディレクトリ取得
    pkg_dir = get_package_share_directory('lang_sam_to_map')
    lsa_pkg = get_package_share_directory('ros2_lang_sam')
    rs_pkg = get_package_share_directory('realsense2_camera')

    # RViz設定ファイルパス
    rviz_config = os.path.join(pkg_dir, 'rviz', 'rviz.rviz')
    
    # パラメータファイルパス
    param_dir = os.path.join(pkg_dir, 'param', 'param.yaml')
    
    load_nodes = GroupAction(
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(lsa_pkg, 'launch', 'server.launch.py')
                ), 
                launch_arguments={
                    'sam_type': 'sam2.1_hiera_large'
                }.items()
            ),
            TimerAction(
                period=1.0,
                actions=[
                    Node(
                        package='lang_sam_to_map',
                        executable='lang_sam_to_map',
                        name='lang_sam_to_map',
                        output='screen',
                        parameters=[param_dir], 
                        # remappings=[('/camera/camera/aligned_depth_to_color/image_raw')]
                    )
                ]
            )
        ])
    rviz2 = Node(package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config, '--ros-args', '--log-level', 'warn'],
        condition=IfCondition(use_rviz))
    ld = LaunchDescription()
    ld.add_action(declare_arg_use_rviz)
    ld.add_action(load_nodes)
    ld.add_action(rviz2)
    return ld
