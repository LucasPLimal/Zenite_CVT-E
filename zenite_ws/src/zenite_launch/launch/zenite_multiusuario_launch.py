# Launch do sistema multiusuário do Zênite (criado por Danilo).
#
# Sobe tudo o que é preciso para vários usuários controlarem vários robôs pela
# interface web, sem rfcomm manual e sem xterm:
#
#   - acquisition_node   câmera (parâmetro video_index)
#   - localization_node  localiza os robôs pelo ArUco (/robot_<id>/current_pose)
#   - control_node       UMA instância por robô (robot_id)
#   - bluetooth_node     procura, pareia e conecta os ESP32 e envia as velocidades
#   - web_interface      interface web (http://<ip do servidor>:8000)
#
# O zenite_launch.py original continua disponível para o uso antigo, com a
# interface gráfica (interface_node) e um robô só em /dev/rfcomm0.
#
# Exemplos:
#   ros2 launch zenite_launch zenite_multiusuario_launch.py
#   ros2 launch zenite_launch zenite_multiusuario_launch.py video_index:=0
#   ros2 launch zenite_launch zenite_multiusuario_launch.py robot_ids:=0,1,2,3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _control_nodes(context):
    """Cria um control_node para cada ID da lista robot_ids (ex.: "0,1,2")."""
    ids = [i.strip() for i in
           LaunchConfiguration('robot_ids').perform(context).split(',')
           if i.strip()]
    return [
        Node(
            package='control_node',
            executable='control_node',
            name=f'control_node_{robot_id}',
            output='screen',
            parameters=[{'robot_id': int(robot_id)}],
        )
        for robot_id in ids
    ]


def generate_launch_description():
    return LaunchDescription([

        DeclareLaunchArgument(
            'video_index', default_value='2',
            description='Índice da câmera (/dev/video<N>). No Pi com a C920 use 0.'),
        DeclareLaunchArgument(
            'robot_ids', default_value='0,1,2',
            description='IDs dos robôs (IDs dos marcadores ArUco), separados por vírgula.'),
        DeclareLaunchArgument(
            'port', default_value='8000',
            description='Porta da interface web.'),

        Node(
            package='acquisition_node',
            executable='acquisition_node',
            name='acquisition_node',
            output='screen',
            parameters=[{'video_index': ParameterValue(
                LaunchConfiguration('video_index'), value_type=int)}],
        ),

        Node(
            package='localization_node',
            executable='localization_node',
            name='localization_node',
            output='screen',
        ),

        OpaqueFunction(function=_control_nodes),

        Node(
            package='bluetooth_node',
            executable='bluetooth_node',
            name='bluetooth_node',
            output='screen',
        ),

        Node(
            package='web_interface',
            executable='web_interface',
            name='web_interface_node',
            output='screen',
            parameters=[{'port': ParameterValue(
                LaunchConfiguration('port'), value_type=int)}],
        ),
    ])
