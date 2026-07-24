// Inclui o cabeçalho da classe CalibrationNode.
// Toda a implementação do nó está nesse arquivo.
#include <calibration_node/calibration_node.hpp>

int main(int argc, char** argv)
{
    // Inicializa o ROS 2.
    // Prepara toda a infraestrutura de comunicação
    // (publishers, subscribers, timers, parâmetros, etc.).
    rclcpp::init(argc, argv);

    // Cria uma instância do CalibrationNode.
    // std::make_shared cria o objeto em um shared_ptr,
    // que é o formato esperado pelo ROS 2.
    rclcpp::spin(std::make_shared<CalibrationNode>());

    // Mantém o nó executando indefinidamente.
    //
    // Enquanto o programa estiver aberto, o ROS ficará:
    // - recebendo imagens da câmera;
    // - chamando imageCallback() sempre que chegar um frame;
    // - processando os cliques do usuário;
    // - calculando a homografia quando os quatro pontos forem definidos.
    //
    // O spin() só termina quando:
    // - o usuário fecha o programa;
    // - ou ocorre um shutdown do ROS.

    // Finaliza corretamente o ROS 2,
    // liberando todos os recursos utilizados.
    rclcpp::shutdown();

    // Indica que o programa terminou sem erros.
    return 0;
}