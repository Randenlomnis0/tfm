# Launch principal del Jackal J100 real: Livox Mid-360 + pointcloud_to_laserscan,
# LiODOM (/odom y TF odom->base_link), SLAM Toolbox (map->odom) y Nav2
# (/navigate_to_pose).
#
# Arbol TF plano en /tf y /tf_static: map -> odom -> base_link -> lidar3d_0.
# Todos los topics sin namespace (/scan, /odom, /map, /tf); el unico con
# namespace es /j100_0562/cmd_vel (donde escucha el MCU del Jackal).
# Publican: static_transform_publisher (base_link->lidar3d_0), liodom
# (odom->base_link), slam_toolbox (map->odom).

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node


# Driver Livox + pointcloud_to_laserscan -> /livox/points y /scan (sin namespace).
def _build_livox_and_p2l(context):
    livox_config = LaunchConfiguration('livox_config').perform(context)

    livox_driver_node = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        parameters=[
            {'xfer_format': 0},                            # PointCloud2
            {'multi_topic': 0},
            {'data_src': 0},
            {'publish_freq': 10.0},
            {'output_data_type': 0},
            {'frame_id': 'lidar3d_0'},                     # frame TF declarado en URDF
            {'user_config_path': livox_config},
            {'cmdline_input_bd_code': 'livox0000000001'},
        ],
        remappings=[
            ('livox/points', '/livox/points'),
            ('livox/imu', '/livox/imu'),
        ],
    )

    # pointcloud_to_laserscan: proyecta la nube 3D a LaserScan 2D para SLAM/Nav2.
    pc2_scan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'target_frame': 'lidar3d_0',       # proyecta al frame del chasis
            'transform_tolerance': 0.3,
            'min_height': 0.25,
            'max_height': 0.4,
            'angle_min': -3.14159265,
            'angle_max': 3.14159265,
            'angle_increment': 0.00872665,     # 0.5 grados -> 720 rayos
            'scan_time': 0.1,
            'range_min': 0.5,
            'range_max': 50.0,
            'use_inf': True,
            'inf_epsilon': 1.0,
        }],
        remappings=[
            ('cloud_in', '/livox/points'),
            ('scan', '/scan'),
        ],
    )

    return [livox_driver_node, pc2_scan_node]


# TF estatico base_link -> lidar3d_0 (x=0.15, z=0.304 m, medidos al Livox).
# z es critico: un error desplaza que puntos entran en la proyeccion /scan.
def _build_static_tf_lidar():
    return Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_lidar3d_0',
        output='screen',
        arguments=[
            '--x', '-0.15',
            '--y', '0.0',
            '--z', '0.304',
            '--roll', '0.0',
            '--pitch', '0.0',
            '--yaw', '3.1415',
            '--frame-id', 'base_link',
            '--child-frame-id', 'lidar3d_0',
        ],
    )


# RViz2
def _build_rviz_node(context):
    rviz_enabled = LaunchConfiguration('rviz').perform(context).lower() == 'true'
    if not rviz_enabled:
        return []

    pkg_tfm = get_package_share_directory('tfm')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_tfm',
        output='screen',
        arguments=['-d', os.path.join(pkg_tfm, 'config', 'tfm_config.rviz')],
        parameters=[{'use_sim_time': False}],
    )

    return [rviz_node]


def _delayed_actions(context, delay_name, actions):
    delay = float(LaunchConfiguration(delay_name).perform(context))
    if delay <= 0.0:
        return actions

    return [TimerAction(period=delay, actions=actions)]


def _config_display_path(pkg_tfm, params_file):
    if os.path.isabs(params_file):
        return params_file
    return os.path.join(pkg_tfm, 'config', params_file)


def _selected_slam_params(context):
    """YAML de SLAM: slam_params (override manual) si se pasa, si no slam_profile."""

    manual_slam_params = LaunchConfiguration('slam_params').perform(context).strip()
    if manual_slam_params:
        return manual_slam_params

    return 'tfm_slam_real.yaml'


def _build_slam_launch(context):
    """Construye el include de SLAM Toolbox cuando ya se conocen los argumentos."""

    pkg_tfm = get_package_share_directory('tfm')
    slam_params = _selected_slam_params(context)
    slam_profile = LaunchConfiguration('slam_profile').perform(context).strip()
    manual_slam_params = LaunchConfiguration('slam_params').perform(context).strip()
    slam_source = (
        'slam_params override'
        if manual_slam_params
        else f'slam_profile:={slam_profile}'
    )
    slam_yaml_path = _config_display_path(pkg_tfm, slam_params)

    return [
        LogInfo(msg=(
            f'[tfm_real] SLAM YAML: {slam_yaml_path} '
            f'({slam_source}, slam_sync:={LaunchConfiguration("slam_sync").perform(context)})'
        )),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_tfm, 'launch', 'tfm_slam.launch.py')
            ),
            launch_arguments={
                'use_sim_time': 'false',
                'sync': LaunchConfiguration('slam_sync'),
                'slam_params': slam_params,
            }.items()
        )
    ]


def _selected_nav2_params(context):
    """YAML de Nav2: nav2_params (override manual) si se pasa, si no nav2_profile."""

    manual_nav2_params = LaunchConfiguration('nav2_params').perform(context).strip()
    if manual_nav2_params:
        return manual_nav2_params

    return 'tfm_nav2_real.yaml'


def _build_nav2_launch(context):
    """Construye el include de Nav2 cuando ya se conocen los argumentos."""

    pkg_tfm = get_package_share_directory('tfm')
    nav2_launch_file = LaunchConfiguration('nav2_launch').perform(context)
    nav2_params = _selected_nav2_params(context)
    nav2_profile = LaunchConfiguration('nav2_profile').perform(context).strip()
    manual_nav2_params = LaunchConfiguration('nav2_params').perform(context).strip()
    use_profile_params = nav2_launch_file == 'tfm_nav2.launch.py' or bool(manual_nav2_params)
    if use_profile_params:
        nav2_source = (
            'nav2_params override'
            if manual_nav2_params
            else f'nav2_profile:={nav2_profile}'
        )
        nav2_yaml_path = _config_display_path(pkg_tfm, nav2_params)
        nav2_launch_arguments = {
            'use_sim_time': 'false',
            'nav2_params': nav2_params,
        }
    else:
        nav2_source = (
            f'{nav2_launch_file} default; nav2_profile:={nav2_profile} '
            'no pisa el YAML del launch especial'
        )
        nav2_yaml_path = f'default interno de {nav2_launch_file}'
        nav2_launch_arguments = {
            'use_sim_time': 'false',
        }

    return [
        LogInfo(msg=(
            f'[tfm_real] Nav2 YAML: {nav2_yaml_path} '
            f'({nav2_source}, nav2_launch:={nav2_launch_file})'
        )),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_tfm, 'launch', nav2_launch_file)
            ),
            launch_arguments=nav2_launch_arguments.items()
        )
    ]


def generate_launch_description():
    arg_rviz = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        choices=['true', 'false'],
        description='Lanzar RViz2 junto con el resto de nodos'
    )

    arg_livox_config = DeclareLaunchArgument(
        'livox_config',
        default_value=os.path.join(
            get_package_share_directory('livox_ros_driver2'),
            'config', 'MID360_config.json'),
        description='Ruta al JSON de configuracion del driver Livox Mid-360'
    )

    arg_odom_delay = DeclareLaunchArgument(
        'odom_delay',
        default_value='4.0',
        description='Segundos de espera antes de arrancar laser_scan_matcher'
    )

    arg_slam_delay = DeclareLaunchArgument(
        'slam_delay',
        default_value='10.0',
        description='Segundos de espera antes de arrancar SLAM Toolbox'
    )

    arg_slam_profile = DeclareLaunchArgument(
        'slam_profile',
        default_value='slam_1',
        choices=['slam_1'],
        description=(
            'Perfil de parametros SLAM. slam_1 -> tfm_slam_real.yaml. '
            'Si slam_params no esta vacio, slam_params tiene prioridad.'
        )
    )

    arg_slam_params = DeclareLaunchArgument(
        'slam_params',
        default_value='',
        description=(
            'Override manual opcional del YAML SLAM dentro de config/. '
            'Si se deja vacio, se usa slam_profile.'
        )
    )

    arg_slam_sync = DeclareLaunchArgument(
        'slam_sync',
        default_value='false',
        choices=['true', 'false'],
        description=(
            'Selecciona el launch de slam_toolbox: false usa online_async_launch.py '
            'como hasta ahora; true usa online_sync_launch.py para pruebas.'
        )
    )

    arg_nav2_delay = DeclareLaunchArgument(
        'nav2_delay',
        default_value='20.0',
        description='Segundos de espera antes de arrancar Nav2 y RViz'
    )

    arg_nav2_launch = DeclareLaunchArgument(
        'nav2_launch',
        default_value='tfm_nav2.launch.py',
        description=(
            'Launch Nav2 dentro de launch/. El default usa nav2_1 como perfil de referencia.'
        )
    )

    arg_nav2_profile = DeclareLaunchArgument(
        'nav2_profile',
        default_value='nav2_1',
        choices=['nav2_1'],
        description=(
            'Perfil de parametros Nav2. nav2_1 -> tfm_nav2_real.yaml. '
            'Si nav2_params no esta vacio, nav2_params tiene prioridad. '
            'Las misiones declarativas dejan behavior_tree vacio para usar el BT por defecto.'
        )
    )

    arg_nav2_params = DeclareLaunchArgument(
        'nav2_params',
        default_value='',
        description=(
            'Override manual opcional del YAML dentro de config/. '
            'Si se deja vacio, se usa nav2_profile.'
        )
    )

    laser_scan_matcher_node = Node(
        package='ros2_laser_scan_matcher',
        executable='laser_scan_matcher',
        name='laser_scan_matcher',
        output='screen',
        parameters=[{
            'base_frame': 'base_link',
            'laser_frame': 'lidar3d_0',
            'odom_frame': 'odom',
            'map_frame': 'map',
            'publish_tf': True,
            'publish_odom': '/odom',
        }],
    )

    # SLAM Toolbox: publica map->odom.
    slam_launch = OpaqueFunction(
        function=_build_slam_launch
    )

    # Stack Nav2 (collision_monitor -> /j100_0562/cmd_vel). Debe estar vivo antes
    # de lanzar una mision: mission_manager espera su action server /navigate_to_pose.
    nav2_launch = OpaqueFunction(function=_build_nav2_launch)

    livox_opaque = OpaqueFunction(function=_build_livox_and_p2l)
    rviz_opaque = OpaqueFunction(function=_build_rviz_node)
    static_tf_lidar = _build_static_tf_lidar()
    
    delayed_odometry = OpaqueFunction(
        function=lambda context: _delayed_actions(context, 'odom_delay', [laser_scan_matcher_node])
    )

    delayed_slam = OpaqueFunction(
        function=lambda context: _delayed_actions(
            context,
            'slam_delay',
            [slam_launch],
        )
    )

    delayed_nav2_and_rviz = OpaqueFunction(
        function=lambda context: _delayed_actions(context, 'nav2_delay', [nav2_launch, rviz_opaque])
    )

    # camera_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         PathJoinSubstitution(["realsense2_camera", "launch", "rs_launch"])
    #     ),
    #     launch_arguments={
    #         'camera_namespace': "/j100_0562",
    #         'camera_name': "D435_1",
    #     }.items(),
    # )

    ld = LaunchDescription()

    ld.add_action(arg_rviz)
    ld.add_action(arg_livox_config)
    ld.add_action(arg_odom_delay)
    ld.add_action(arg_slam_delay)
    ld.add_action(arg_slam_profile)
    ld.add_action(arg_slam_params)
    ld.add_action(arg_slam_sync)
    ld.add_action(arg_nav2_delay)
    ld.add_action(arg_nav2_launch)
    ld.add_action(arg_nav2_profile)
    ld.add_action(arg_nav2_params)

    ld.add_action(static_tf_lidar)
    ld.add_action(livox_opaque)
    ld.add_action(delayed_odometry)
    ld.add_action(delayed_slam)
    ld.add_action(delayed_nav2_and_rviz)
    # ld.add_action(camera_launch)

    return ld
