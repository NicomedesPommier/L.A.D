from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    cors_origin     = LaunchConfiguration('cors_allow_origin')
    static_port     = LaunchConfiguration('static_port')
    enable_rosapi   = LaunchConfiguration('enable_rosapi')
    enable_jsp      = LaunchConfiguration('enable_jsp')
    enable_wvs      = LaunchConfiguration('enable_wvs')
    wvs_port        = LaunchConfiguration('wvs_port')
    enable_turtle   = LaunchConfiguration('enable_turtlesim')

    # Rutas desde qcar_description instalado
    qcar_share  = get_package_share_directory('qcar_description')
    static_root = os.path.dirname(qcar_share)  # .../install/.../share
    urdf_file   = os.path.join(qcar_share, 'urdf', 'robot_runtime.urdf')

    env = [
        SetEnvironmentVariable(name='CORS_ALLOW_ORIGIN', value=cors_origin),
        SetEnvironmentVariable(name='QT_QPA_PLATFORM', value='offscreen')
    ]

    # HTTP estático con CORS (sirve /share, por lo tanto /qcar_description/...)
    http = ExecuteProcess(
        cmd=['python3', '-u', '/http_cors.py', '--dir', static_root, '--port', static_port],
        output='screen'
    )

    rosbridge = ExecuteProcess(
        cmd=['ros2', 'launch', 'rosbridge_server', 'rosbridge_websocket_launch.xml'],
        output='screen'
    )

    tf2web = Node(
        package='tf2_web_republisher_py', executable='tf2_web_republisher',
        name='tf2_web_republisher', output='screen'
    )

    # Publica TF desde el URDF plano generado por el entrypoint
    rsp = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        name='robot_state_publisher', output='screen',
        parameters=[{'robot_description': open(urdf_file).read(), 'use_tf_static': True}]
    )

    jsp = Node(
        package='joint_state_publisher', executable='joint_state_publisher',
        name='joint_state_publisher', output='screen',
        condition=IfCondition(enable_jsp)
    )

    rosapi = Node(
        package='rosapi', executable='rosapi_node', name='rosapi',
        output='screen', condition=IfCondition(enable_rosapi)
    )

    wvs = Node(
        package='web_video_server', executable='web_video_server', name='web_video_server',
        output='screen', arguments=['--port', wvs_port],
        condition=IfCondition(enable_wvs)
    )

    turtlesim = ExecuteProcess(
        cmd=['bash', '-lc', 'xvfb-run -s "-screen 0 800x600x24" ros2 run turtlesim turtlesim_node'],
        output='screen', condition=IfCondition(enable_turtle)
    )

    return LaunchDescription([
        DeclareLaunchArgument('cors_allow_origin', default_value='http://localhost:8000'),
        DeclareLaunchArgument('static_port',      default_value='7000'),
        DeclareLaunchArgument('enable_rosapi',    default_value='true'),
        DeclareLaunchArgument('enable_jsp',       default_value='true'),
        DeclareLaunchArgument('enable_wvs',       default_value='false'),
        DeclareLaunchArgument('wvs_port',         default_value='8080'),
        DeclareLaunchArgument('enable_turtlesim', default_value='true'),
        *env, http, rosbridge, tf2web, rsp, jsp, rosapi, wvs, turtlesim
    ])
