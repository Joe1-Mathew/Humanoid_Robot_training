import json
import os
import re
import sys
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory('rpo_description')

    model = LaunchConfiguration('model').perform(context)

    original_urdf_file = os.path.join(pkg_share, 'models', 'rpo_model', 'urdf', 'rpo_model.urdf')
    model_urdf_file = os.path.join(pkg_share, 'models', model, 'urdf', f'{model}.urdf')
    rviz_config = os.path.join(pkg_share, 'rviz', 'compare.rviz')

    with open(original_urdf_file, 'r') as f:
        original_description = f.read()
    with open(model_urdf_file, 'r') as f:
        model_description = f.read()

    original_ns = 'original'
    model_ns = 'compare'

    all_joint_names = set(re.findall(
        r'<joint\s+name="([^"]+)"\s+type="(?:revolute|continuous|prismatic)"', original_description))
    model_active_joint_names = set(re.findall(
        r'<joint\s+name="([^"]+)"\s+type="(?:revolute|continuous|prismatic)"', model_description))
    gui_description = original_description

    for name in all_joint_names - model_active_joint_names:
        gui_description = re.sub(
            r'(<joint\s+name="%s"\s+type=")(?:revolute|continuous|prismatic)(")' % re.escape(name),
            r'\g<1>fixed\g<2>',
            gui_description,
        )

    original_rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=original_ns,
        output='screen',
        parameters=[{'robot_description': original_description, 'frame_prefix': f'{original_ns}/'}],
    )

    model_rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=model_ns,
        output='screen',
        parameters=[{'robot_description': model_description, 'frame_prefix': f'{model_ns}/'}],
    )

    gui_ns = 'gui'

    gui_rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=gui_ns,
        output='screen',
        parameters=[{'robot_description': gui_description, 'frame_prefix': f'{gui_ns}/'}],
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        namespace=gui_ns,
        output='screen',
        remappings=[('joint_states', f'/{model_ns}/joint_states')],
    )

    origin_to_original_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='origin_to_original',
        output='screen',
        arguments=['--x', '0', '--y', '-0.25', '--z', '0', '--roll', '0', '--pitch', '0', '--yaw', '0',
                   '--frame-id', 'origin', '--child-frame-id', f'{original_ns}/base_link'],
    )

    origin_to_model_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='origin_to_model',
        output='screen',
        arguments=['--x', '0', '--y', '2.0', '--z', '0.3', '--roll', '-0.10', '--pitch', '-1.57', '--yaw', '2.90',
                   '--frame-id', 'origin', '--child-frame-id', f'{model_ns}/root'],
    )

    joint_names_file = tempfile.NamedTemporaryFile(
        mode='w', suffix='.json', prefix='joint_names_', delete=False)
    json.dump(sorted(all_joint_names), joint_names_file)
    joint_names_file.close()

    fill_defaults_script = os.path.join(os.path.dirname(__file__), 'joint_state_fill_defaults.py')
    joint_state_fill_defaults_node = ExecuteProcess(
        cmd=[
            sys.executable,
            fill_defaults_script,
            '--ros-args',
            '-p', f'source_topic:=/{model_ns}/joint_states',
            '-p', f'target_topic:=/{original_ns}/joint_states',
            '-p', f'joint_names_file:={joint_names_file.name}',
        ],
        output='screen',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        additional_env={
            'QT_ENABLE_HIGHDPI_SCALING': '0',
        },
    )

    return [
        original_rsp_node,
        model_rsp_node,
        gui_rsp_node,
        joint_state_publisher_gui_node,
        origin_to_original_tf_node,
        origin_to_model_tf_node,
        joint_state_fill_defaults_node,
        rviz_node,
    ]


def generate_launch_description():
    declare_model = DeclareLaunchArgument(
        'model',
        default_value='full_assembly',
        description='Exported URDF to compare against rpo.urdf (gripper, left_leg, hip, lower_body, full_assembly)'
    )

    return LaunchDescription([
        declare_model,
        OpaqueFunction(function=launch_setup),
    ])

