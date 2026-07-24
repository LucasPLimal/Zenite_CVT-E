// Inclui o arquivo de cabeçalho da classe TransmissionNode
#include <transmission_node/transmission_node.hpp>

// Função principal do programa
int main(int argc, char * argv[])
{
  // Inicializa o sistema de comunicação do ROS 2.
  // Deve ser chamado antes de criar qualquer nó.
  rclcpp::init(argc, argv);

  // Cria uma instância do TransmissionNode e mantém o nó em execução.
  // O spin() fica processando callbacks (mensagens recebidas) continuamente.
  rclcpp::spin(std::make_shared<TransmissionNode>());

  // Finaliza o ROS 2, liberando todos os recursos utilizados.
  rclcpp::shutdown();

  // Encerra o programa informando que tudo ocorreu corretamente.
  return 0;
}