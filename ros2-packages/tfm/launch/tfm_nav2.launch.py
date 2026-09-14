# Wrapper del stack Nav2 en namespace raiz. El unico topic con namespace es la
# salida de collision_monitor (cmd_vel_out_topic -> MCU del Jackal).

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from nav2_common.launch import RewrittenYaml


def _build_nav2_include(context):
    pkg_tfm = get_package_share_directory('tfm')
    use_sim_time = LaunchConfiguration('use_sim_time')
    cmd_vel_out_topic = LaunchConfiguration('cmd_vel_out_topic')

    nav2_params_value = LaunchConfiguration('nav2_params').perform(context).strip()
    nav2_stack_value = LaunchConfiguration('nav2_stack').perform(context).strip()

    if nav2_stack_value == 'auto':
        nav2_stack_value = 'full'

    if os.path.isabs(nav2_params_value):
        nav2_params_file = nav2_params_value
    else:
        nav2_params_file = os.path.join(pkg_tfm, 'config', nav2_params_value)

    # param_rewrites = {
    #     'cmd_vel_out_topic': cmd_vel_out_topic,
    #     # BT default cuando el goal llega con behavior_tree vacio; un goal con BT
    #     # propio (p. ej. P1 de corridor_block) lo sobreescribe.
    #     'default_nav_to_pose_bt_xml': os.path.join(
    #         pkg_tfm, 'config', 'behavior_trees', 'navigate_to_pose_default.xml'
    #     ),
    # }

    param_rewrites = {
        'cmd_vel_out_topic': cmd_vel_out_topic,
    }

    rewritten_parameters = RewrittenYaml(
        source_file=nav2_params_file,
        param_rewrites=param_rewrites,
        convert_types=True
    )

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([pkg_tfm, 'launch', 'navigation_launch.py'])
            ),
            launch_arguments=[
                # namespace='' obligatorio: navigation_launch.py envuelve el YAML
                # con root_key=namespace pero lanza los nodos en root; con namespace
                # los nodos no encontrarian sus parametros.
                ('namespace', ''),
                ('use_sim_time', use_sim_time),
                ('params_file', rewritten_parameters),
                ('nav2_stack', nav2_stack_value),
                ('use_composition', 'False'),
            ]
        )
    ]


def generate_launch_description():
    arg_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        choices=['true', 'false'],
        description='Usar el reloj ROS configurado externamente'
    )
    arg_nav2_params = DeclareLaunchArgument(
        'nav2_params',
        default_value='tfm_nav2_real.yaml',
        description='Nombre del archivo de parametros Nav2 dentro de config/'
    )
    arg_nav2_stack = DeclareLaunchArgument(
        'nav2_stack',
        default_value='auto',
        choices=['auto', 'full'],
        description='auto usa full; full lanza la cadena Nav2 normal.'
    )
    arg_cmd_vel_out_topic = DeclareLaunchArgument(
        'cmd_vel_out_topic',
        default_value='/j100_0562/cmd_vel',
        description=(
            'Topic final donde collision_monitor publica la velocidad. '
            'En robot real debe ser /j100_0562/cmd_vel; en simulacion, '
            'normalmente /cpr_j100_0000/cmd_vel.'
        )
    )

    nav2 = OpaqueFunction(function=_build_nav2_include)

    return LaunchDescription([
        arg_use_sim_time,
        arg_nav2_params,
        arg_nav2_stack,
        arg_cmd_vel_out_topic,
        nav2,
    ])
