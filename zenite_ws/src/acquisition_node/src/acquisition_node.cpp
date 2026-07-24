// Inclui o arquivo de cabeçalho (header) do AcquisitionNode.
//
// Esse arquivo contém a declaração da classe AcquisitionNode,
// ou seja, toda a estrutura do nó responsável pela aquisição
// das imagens da câmera.
#include "acquisition_node/acquisition_node.hpp"


// ===============================================================
// Função principal do programa.
//
// Todo programa em C++ começa sua execução por essa função.
// ===============================================================
int main(int argc, char** argv)
{

    // Inicializa o ROS 2.
    //
    // Essa chamada prepara toda a infraestrutura necessária
    // para o funcionamento do ROS:
    //
    // • criação do contexto do ROS;
    // • comunicação DDS;
    // • gerenciamento de nós;
    // • parâmetros;
    // • tópicos;
    // • serviços;
    // • actions.
    //
    // argc e argv permitem que o ROS interprete argumentos
    // passados pela linha de comando.
    rclcpp::init(argc, argv);


    // Cria uma instância do AcquisitionNode.
    //
    // std::make_shared cria um objeto utilizando um
    // ponteiro inteligente (shared_ptr), responsável por
    // gerenciar automaticamente a memória do objeto.
    //
    // Quando não houver mais referências para esse ponteiro,
    // a memória será liberada automaticamente.
    auto node = std::make_shared<AcquisitionNode>();


    // Coloca o nó em execução.
    //
    // A função spin() mantém o programa em funcionamento
    // enquanto o ROS processa continuamente:
    //
    // • timers;
    // • subscribers;
    // • callbacks;
    // • serviços;
    // • actions.
    //
    // No caso do AcquisitionNode, ela executará
    // continuamente:
    //
    // • o timer responsável por capturar os frames;
    // • o callback que recebe novos parâmetros da câmera.
    //
    // Essa função somente retorna quando o ROS for encerrado.
    rclcpp::spin(node);


    // Finaliza o ROS 2.
    //
    // Libera todos os recursos utilizados pelo middleware,
    // encerra comunicações e realiza a limpeza necessária.
    rclcpp::shutdown();


    // Retorna 0 indicando que o programa terminou
    // corretamente.
    return 0;
}