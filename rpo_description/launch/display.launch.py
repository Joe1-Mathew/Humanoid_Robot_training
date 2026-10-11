import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory('rpo_description')
    model = LaunchConfiguration('model').perform(context)

    urdf_path = os.path.join(pkg_share, 'models', model, 'urdf', f'{model}.urdf')
    # ----- Uncomment to add your RViz config -----
    # rviz_path = os.path.join(pkg_share, 'rviz', 'display.rviz')

    with open(urdf_path, 'r') as urdf_file:
        robot_description = urdf_file.read()

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_joint_state_gui')),
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        # ----- Uncomment to add your RViz config -----
        # arguments=['-d', rviz_path],
    )

    return [
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node,
    ]


def generate_launch_description():
    declare_model = DeclareLaunchArgument(
        'model',
        default_value='rpo',
        description='Exported URDF to display (gripper, left_leg, hip, lower_body, full_assembly, rpo)'
    )
    declare_use_joint_state_gui = DeclareLaunchArgument(
        'use_joint_state_gui',
        default_value='true',
        description='Launch joint_state_publisher_gui. Set to false when another node '
                     '(e.g. an IK solver) is going to publish /joint_states instead.'
    )

    return LaunchDescription([
        declare_model,
        declare_use_joint_state_gui,
        OpaqueFunction(function=launch_setup),
    ])