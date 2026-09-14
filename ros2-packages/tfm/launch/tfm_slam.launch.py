# Wrapper que arranca SLAM Toolbox en topics planos (/scan, /map, /tf), sin
# namespace ni remapeos. Publica el TF map->odom. arg sync elige online
# sincrono/asincrono.

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    pkg_tfm = get_package_share_directory('tfm')
    pkg_slam_toolbox = get_package_share_directory('slam_toolbox')

    arg_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='false', choices=['true', 'false'])
    arg_autostart = DeclareLaunchArgument('autostart', default_value='true', choices=['true', 'false'])
    arg_use_lifecycle_manager = DeclareLaunchArgument('use_lifecycle_manager', default_value='false', choices=['true', 'false'])
    arg_sync = DeclareLaunchArgument('sync', default_value='true', choices=['true', 'false'])
    arg_slam_params = DeclareLaunchArgument('slam_params', default_value='tfm_slam_real.yaml',
                                             description='Fichero de parametros SLAM dentro de tfm/config/')

    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    use_lifecycle_manager = LaunchConfiguration('use_lifecycle_manager')
    sync = LaunchConfiguration('sync')
    slam_params = LaunchConfiguration('slam_params')

    # Sin reescritura de claves: los topics ya son planos en el YAML.
    rewritten_parameters = RewrittenYaml(
        source_file=PathJoinSubstitution([pkg_tfm, 'config', slam_params]),
        param_rewrites={},
        convert_types=True
    )

    launch_slam_sync = PathJoinSubstitution([pkg_slam_toolbox, 'launch', 'online_sync_launch.py'])
    launch_slam_async = PathJoinSubstitution([pkg_slam_toolbox, 'launch', 'online_async_launch.py'])

    slam_sync = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_slam_sync),
        launch_arguments=[
            ('use_sim_time', use_sim_time),
            ('autostart', autostart),
            ('use_lifecycle_manager', use_lifecycle_manager),
            ('slam_params_file', rewritten_parameters),
        ],
        condition=IfCondition(sync)
    )

    slam_async = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_slam_async),
        launch_arguments=[
            ('use_sim_time', use_sim_time),
            ('autostart', autostart),
            ('use_lifecycle_manager', use_lifecycle_manager),
            ('slam_params_file', rewritten_parameters),
        ],
        condition=UnlessCondition(sync)
    )

    return LaunchDescription([
        arg_use_sim_time,
        arg_autostart,
        arg_use_lifecycle_manager,
        arg_sync,
        arg_slam_params,
        slam_sync,
        slam_async,
    ])
