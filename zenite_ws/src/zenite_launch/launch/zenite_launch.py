# Importa a classe LaunchDescription, responsável por definir quais nós
# serão iniciados quando o arquivo launch for executado.
from launch import LaunchDescription

# Importa a ação Node, utilizada para criar e configurar nós ROS 2.
from launch_ros.actions import Node

# Função obrigatória em arquivos launch do ROS 2.
# O ROS procura exatamente esta função para obter a lista de nós que serão executados.
def generate_launch_description():

    # Retorna uma descrição contendo todos os nós que serão inicializados.
    return LaunchDescription([

        # ------------------------- Acquisition Node -------------------------

        # Cria o nó responsável pela aquisição das imagens da câmera.
        Node(

            # Pacote onde está localizado o executável.
            package='acquisition_node',

            # Nome do executável que será iniciado.
            executable='acquisition_node',

            # Nome que aparecerá no grafo do ROS.
            name='acquisition_node',

            # Exibe toda a saída (logs) diretamente no terminal.
            output='screen',

            # Abre uma janela xterm exclusiva para esse nó.
            # O parâmetro -hold mantém o terminal aberto mesmo após o programa finalizar,
            # permitindo visualizar possíveis erros.
            prefix='xterm -hold -e'
        ),

        # -------------------------- Interface Node --------------------------

        # Nó responsável pela interface gráfica do sistema.
        # Exibe a câmera, recebe cliques do usuário e envia a posição desejada.
        Node(
            package='interface_node',
            executable='interface_node',
            name='interface_node',
            output='screen',
            prefix='xterm -hold -e'
        ),

        # --------------------------- Control Node ---------------------------

        # Nó responsável pelo algoritmo de controle do robô.
        # Recebe posição atual e posição desejada e calcula
        # as velocidades das rodas.
        Node(
            package='control_node',
            executable='control_node',
            name='control_node',
            output='screen',
            prefix='xterm -hold -e'
        ),

        # ------------------------ Transmission Node -------------------------

        # Nó responsável por converter as velocidades calculadas
        # em pacotes Bluetooth enviados ao Arduino.
        Node(
            package='transmission_node',
            executable='transmission_node',
            name='transmission_node',
            output='screen',
            prefix='xterm -hold -e'
        ),

        # ----------------------- Localization Node --------------------------

        # Nó responsável pelo rastreamento do robô através da câmera.
        # Utiliza o Tracker CSRT para acompanhar o carrinho,
        # converte a posição de pixels para metros e publica
        # a posição atual para o ControlNode.
        Node(
            package='localization_node',
            executable='localization_node',
            name='localization_node',
            output='screen',
            prefix='xterm -hold -e'
        ),
    ])