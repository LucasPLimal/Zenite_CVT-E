# Importa a classe responsável por criar uma descrição de lançamento (launch)
# contendo todos os nós que serão executados.
from launch import LaunchDescription

# Importa a ação Node, utilizada para iniciar nós do ROS 2.
from launch_ros.actions import Node


# Função obrigatória em um arquivo launch do ROS 2.
# Ela retorna a descrição completa do lançamento.
def generate_launch_description():

    # Cria e retorna uma LaunchDescription contendo todos os nós desejados.
    return LaunchDescription([

        # ------------------------- Acquisition Node -------------------------

        Node(

            # Nome do pacote onde está o executável.
            package='acquisition_node',

            # Nome do executável compilado.
            executable='acquisition_node',

            # Nome que o nó terá dentro da rede ROS 2.
            name='acquisition_node',

            # Faz com que todas as mensagens (logs) apareçam no terminal.
            output='screen',

            # Abre o nó em uma janela separada do xterm.
            # O parâmetro "-hold" mantém a janela aberta mesmo após o programa terminar,
            # permitindo visualizar mensagens de erro.
            prefix='xterm -hold -e'
        ),

        # ------------------------- Calibration Node -------------------------

        Node(

            # Pacote onde está localizado o CalibrationNode.
            package='calibration_node',

            # Executável que será iniciado.
            executable='calibration_node',

            # Nome do nó dentro do ROS 2.
            name='calibration_node',

            # Exibe as mensagens do nó no terminal.
            output='screen',

            # Executa o nó em uma janela própria do xterm,
            # facilitando a depuração e visualização dos logs.
            prefix='xterm -hold -e'
        ),
    ])