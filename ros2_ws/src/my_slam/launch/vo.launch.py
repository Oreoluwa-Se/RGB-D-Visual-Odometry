import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    container = ComposableNodeContainer(
        name="VisualOdom_Container",
        namespace="", package="my_slam",
        executable="run_odom",
        parameters=[{'use_sim_time': True}],
        composable_node_descriptions=[
            ComposableNode(package="my_slam",
                           plugin="my_slam::Frontend",
                           name="frontend_node"),
            ComposableNode(package="my_slam",
                           plugin="dataset::Reader",
                           name="data_reader_node"),
            ComposableNode(package="my_slam",
                           plugin="my_slam::VisualOdom",
                           name="visual_odom_node"),
            ComposableNode(package="my_slam",
                           plugin="my_slam::Viewer",
                           name="slam_viewer_node")
        ],
        # prefix=['xterm -e gdb -ex run --args'],
        output="screen")

    rviz_node = Node(package='rviz2',
                     executable='rviz2',
                     arguments=['-d', "/home/user/ros2_ws/src/my_slam/rviz/slam.rviz"])

    return LaunchDescription([container, rviz_node])
