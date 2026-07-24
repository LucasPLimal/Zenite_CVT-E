// Inclui o arquivo de cabeçalho do ControlNode.
//
// Neste arquivo estão definidos:
// - a classe ControlNode;
// - seus atributos;
// - seus métodos;
// - publishers;
// - subscribers;
// - timer;
// - toda a lógica responsável pelo controle do robô.
//
// O main.cpp apenas cria essa classe e inicia sua execução.
#include <control_node/control_node.hpp>

// Função principal do programa.
//
// Todo programa em C++ começa pela função main().
// Quando o executável do ControlNode é iniciado pelo ROS2,
// esta função é a primeira a ser executada.
int main(int argc, char * argv[])
{
  // Inicializa toda a infraestrutura do ROS2.
  //
  // Esse comando deve obrigatoriamente ser chamado antes
  // da criação de qualquer Node.
  //
  // argc e argv recebem os argumentos passados pela linha
  // de comando, permitindo que o ROS2 processe parâmetros,
  // namespaces, remapeamentos de tópicos, entre outros.
  rclcpp::init(argc, argv);

  // Cria uma instância da classe ControlNode.
  //
  // std::make_shared cria o objeto utilizando um Smart Pointer
  // (std::shared_ptr), que é o padrão utilizado pelo ROS2 para
  // gerenciamento automático de memória.
  //
  // Em seguida, rclcpp::spin() coloca o nó em execução.
  //
  // Enquanto o programa estiver rodando, o ROS2 ficará
  // aguardando eventos, como:
  //
  // • chegada de mensagens dos Subscribers;
  // • execução periódica do Timer;
  // • chamadas de serviços (caso existam);
  // • demais eventos internos do ROS2.
  //
  // A função somente retorna quando o nó é encerrado.
  rclcpp::spin(std::make_shared<ControlNode>());

  // Finaliza a infraestrutura do ROS2.
  //
  // Libera todos os recursos utilizados pela biblioteca,
  // encerra comunicações e realiza o desligamento correto
  // do sistema.
  rclcpp::shutdown();

  // Retorna 0 para o sistema operacional.
  //
  // O valor 0 indica que o programa foi executado e encerrado
  // com sucesso, sem erros.
  return 0;
}