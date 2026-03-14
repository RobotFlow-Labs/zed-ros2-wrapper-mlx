# Copyright 2025 Stereolabs
# Copyright 2025 AIFLOW LABS LIMITED (macOS/MLX port)
#
# Licensed under the Apache License, Version 2.0 (the 'License');
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an 'AS IS' BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# =============================================================================
# macOS / Apple Silicon / MLX Launch File
# =============================================================================
#
# Simplified launch file for the ZED ROS 2 wrapper on macOS with MLX depth.
#
# Key differences from the upstream zed_camera.launch.py:
#   - Loads macOS-specific config files (common_stereo_macos.yaml, zed2i_macos.yaml)
#   - Only starts the stereo camera component (no mono camera support)
#   - Skips object detection and body tracking node configuration
#   - Defaults camera_model to 'zed2i'
#   - Removes simulation, streaming, and GNSS launch arguments
#   - Removes NITROS-related configuration
#
# Usage:
#   ros2 launch zed_wrapper zed_camera_macos.launch.py
#   ros2 launch zed_wrapper zed_camera_macos.launch.py camera_model:=zed2
#   ros2 launch zed_wrapper zed_camera_macos.launch.py ros_params_override_path:=/path/to/override.yaml
#
# =============================================================================

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    LogInfo
)
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    Command,
    TextSubstitution
)
from launch_ros.actions import (
    ComposableNodeContainer,
    LoadComposableNodes
)
from launch_ros.descriptions import ComposableNode

try:
    from launch_ros.actions import Node
except ImportError:
    # Fallback for older ROS 2 distributions
    from launch_ros.actions import LifecycleNode as Node

# Enable colored output
os.environ["RCUTILS_COLORIZED_OUTPUT"] = "1"

# macOS-specific configuration files
default_config_common_macos = os.path.join(
    get_package_share_directory('zed_wrapper'),
    'config',
    'common_stereo_macos.yaml'
)

# URDF/xacro file to be loaded by the Robot State Publisher node
default_xacro_path = os.path.join(
    get_package_share_directory('zed_description'),
    'urdf',
    'zed_descr.urdf.xacro'
)


def launch_setup(context, *args, **kwargs):
    return_array = []

    # Launch configuration variables
    node_log_type = LaunchConfiguration('node_log_type')
    svo_path = LaunchConfiguration('svo_path')
    publish_svo_clock = LaunchConfiguration('publish_svo_clock')
    enable_ipc = LaunchConfiguration('enable_ipc')
    use_sim_time = LaunchConfiguration('use_sim_time')

    container_name = LaunchConfiguration('container_name')
    namespace = LaunchConfiguration('namespace')
    camera_name = LaunchConfiguration('camera_name')
    camera_model = LaunchConfiguration('camera_model')
    node_name = LaunchConfiguration('node_name')
    ros_params_override_path = LaunchConfiguration('ros_params_override_path')

    serial_number = LaunchConfiguration('serial_number')
    camera_id = LaunchConfiguration('camera_id')

    publish_urdf = LaunchConfiguration('publish_urdf')
    publish_tf = LaunchConfiguration('publish_tf')
    publish_map_tf = LaunchConfiguration('publish_map_tf')
    publish_imu_tf = LaunchConfiguration('publish_imu_tf')
    xacro_path = LaunchConfiguration('xacro_path')

    # Resolve values
    node_log_type_val = node_log_type.perform(context)
    container_name_val = container_name.perform(context)
    namespace_val = namespace.perform(context)
    camera_name_val = camera_name.perform(context)
    camera_model_val = camera_model.perform(context)
    node_name_val = node_name.perform(context)

    if node_log_type_val == 'both':
        node_log_effective = 'both'
    else:
        node_log_effective = {
            'stdout': node_log_type_val,
            'stderr': node_log_type_val
        }

    if camera_name_val == '':
        camera_name_val = 'zed'

    if namespace_val == '':
        namespace_val = camera_name_val
    else:
        node_name_val = camera_name_val

    # ----- macOS: always use stereo common config -----
    config_common_path_val = default_config_common_macos
    info = '[macOS/MLX] Using common configuration file: ' + config_common_path_val
    return_array.append(LogInfo(msg=TextSubstitution(text=info)))

    # Camera-specific configuration file (macOS variant if available)
    config_camera_macos_path = os.path.join(
        get_package_share_directory('zed_wrapper'),
        'config',
        camera_model_val + '_macos.yaml'
    )
    config_camera_upstream_path = os.path.join(
        get_package_share_directory('zed_wrapper'),
        'config',
        camera_model_val + '.yaml'
    )

    # Prefer macOS-specific camera config, fall back to upstream
    if os.path.exists(config_camera_macos_path):
        config_camera_path = config_camera_macos_path
        info = '[macOS/MLX] Using macOS camera configuration: ' + config_camera_path
    else:
        config_camera_path = config_camera_upstream_path
        info = '[macOS/MLX] Using upstream camera configuration (no macOS variant): ' + config_camera_path
    return_array.append(LogInfo(msg=TextSubstitution(text=info)))

    # Log macOS-specific info
    return_array.append(LogInfo(msg=TextSubstitution(
        text='[macOS/MLX] Object detection: DISABLED (requires CUDA)')))
    return_array.append(LogInfo(msg=TextSubstitution(
        text='[macOS/MLX] Body tracking: DISABLED (requires CUDA)')))
    return_array.append(LogInfo(msg=TextSubstitution(
        text='[macOS/MLX] Depth mode: MLX (Apple Silicon GPU stereo matching)')))

    # Xacro command with options
    xacro_command = [
        'xacro', ' ',
        xacro_path.perform(context), ' ',
        'camera_name:=', camera_name_val, ' ',
        'camera_model:=', camera_model_val, ' ',
    ]

    # Robot State Publisher node
    rsp_name = camera_name_val + '_state_publisher'
    rsp_node = Node(
        condition=IfCondition(publish_urdf),
        package='robot_state_publisher',
        namespace=namespace_val,
        executable='robot_state_publisher',
        name=rsp_name,
        output=node_log_effective,
        parameters=[{
            'use_sim_time': publish_svo_clock,
            'robot_description': Command(xacro_command)
        }],
        remappings=[('robot_description', camera_name_val + '_description')]
    )
    return_array.append(rsp_node)

    # ROS 2 Component Container
    if container_name_val == '':
        container_name_val = 'zed_container'
        container_exec = 'component_container_isolated'
        arguments_val = [
            '--use_multi_threaded_executor',
            '--ros-args', '--log-level', 'info'
        ]

        zed_container = ComposableNodeContainer(
            name=container_name_val,
            namespace=namespace_val,
            package='rclcpp_components',
            executable=container_exec,
            arguments=arguments_val,
            output=node_log_effective,
            composable_node_descriptions=[]
        )
        return_array.append(zed_container)

    # ZED Node parameters -- macOS simplified (no OD/BT configs)
    ros_params_override_path_val = ros_params_override_path.perform(context)

    node_parameters = [
        config_common_path_val,   # macOS common parameters
        config_camera_path,       # Camera-specific parameters
    ]

    if ros_params_override_path_val != '':
        node_parameters.append(ros_params_override_path)
        info = '[macOS/MLX] Using ROS parameters override file: ' + ros_params_override_path_val
        return_array.append(LogInfo(msg=TextSubstitution(text=info)))

    node_parameters.append({
        'use_sim_time': use_sim_time,
        'general.camera_name': camera_name_val,
        'general.camera_model': camera_model_val,
        'svo.svo_path': svo_path,
        'svo.publish_svo_clock': publish_svo_clock,
        'general.serial_number': serial_number,
        'general.camera_id': camera_id,
        'pos_tracking.publish_tf': publish_tf,
        'pos_tracking.publish_map_tf': publish_map_tf,
        'sensors.publish_imu_tf': publish_imu_tf,
    })

    # ZED Wrapper component -- stereo cameras only on macOS
    zed_wrapper_component = ComposableNode(
        package='zed_components',
        namespace=namespace_val,
        plugin='stereolabs::ZedCamera',
        name=node_name_val,
        parameters=node_parameters,
        extra_arguments=[{'use_intra_process_comms': enable_ipc}]
    )

    full_container_name = '/' + namespace_val + '/' + container_name_val
    info = '[macOS/MLX] Loading ZED node `' + node_name_val + '` in container `' + full_container_name + '`'
    return_array.append(LogInfo(msg=TextSubstitution(text=info)))

    load_composable_node = LoadComposableNodes(
        target_container=full_container_name,
        composable_node_descriptions=[zed_wrapper_component]
    )
    return_array.append(load_composable_node)

    return return_array


def generate_launch_description():
    return LaunchDescription(
        [
            # Declare launch arguments -- macOS simplified set
            DeclareLaunchArgument(
                'node_log_type',
                default_value=TextSubstitution(text='both'),
                description='The log type of the node: screen, log, or both.',
                choices=['screen', 'log', 'both']),
            DeclareLaunchArgument(
                'camera_name',
                default_value=TextSubstitution(text='zed'),
                description='The name of the camera. Used as node namespace.'),
            DeclareLaunchArgument(
                'camera_model',
                default_value=TextSubstitution(text='zed2i'),
                description='The model of the camera. Default: zed2i (primary macOS-tested model).',
                choices=['zed', 'zedm', 'zed2', 'zed2i']),
            DeclareLaunchArgument(
                'container_name',
                default_value='',
                description='The name of an existing container to load the ZED component into. If empty, a new container is created.'),
            DeclareLaunchArgument(
                'namespace',
                default_value='',
                description='The namespace of the node. If empty, the camera name is used.'),
            DeclareLaunchArgument(
                'node_name',
                default_value='zed_node',
                description='The name of the zed_wrapper node.'),
            DeclareLaunchArgument(
                'ros_params_override_path',
                default_value='',
                description='Path to a YAML file to override default parameters.'),
            DeclareLaunchArgument(
                'serial_number',
                default_value='0',
                description='The serial number of the camera to open.'),
            DeclareLaunchArgument(
                'camera_id',
                default_value='-1',
                description='The ID of the camera to open.'),
            DeclareLaunchArgument(
                'publish_urdf',
                default_value='true',
                description='Enable URDF processing and Robot State Publisher.',
                choices=['true', 'false']),
            DeclareLaunchArgument(
                'publish_tf',
                default_value='true',
                description='Enable publication of the odom -> camera_link TF.',
                choices=['true', 'false']),
            DeclareLaunchArgument(
                'publish_map_tf',
                default_value='false',
                description='Enable publication of the map -> odom TF. Disabled by default on macOS (no SLAM).',
                choices=['true', 'false']),
            DeclareLaunchArgument(
                'publish_imu_tf',
                default_value='false',
                description='Enable publication of the IMU TF.',
                choices=['true', 'false']),
            DeclareLaunchArgument(
                'xacro_path',
                default_value=TextSubstitution(text=default_xacro_path),
                description='Path to the camera URDF xacro file.'),
            DeclareLaunchArgument(
                'svo_path',
                default_value=TextSubstitution(text='live'),
                description='Path to an input SVO file. Use "live" for live camera.'),
            DeclareLaunchArgument(
                'publish_svo_clock',
                default_value='false',
                description='Publish SVO clock to /clock topic.',
                choices=['true', 'false']),
            DeclareLaunchArgument(
                'enable_ipc',
                default_value='true',
                description='Enable intra-process communication (IPC) with ROS 2 Composition.',
                choices=['true', 'false']),
            DeclareLaunchArgument(
                'use_sim_time',
                default_value='false',
                description='Use simulated clock from /clock topic.',
                choices=['true', 'false']),
            OpaqueFunction(function=launch_setup)
        ]
    )
