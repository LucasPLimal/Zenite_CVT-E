// Inclui o arquivo de cabeçalho da classe InterfaceNode.
//
// Nesse arquivo estão definidos todos os métodos,
// atributos, publishers, subscribers e a lógica
// responsável pela interface gráfica do Projeto Zênite.
#include <interface_node/interface_node.hpp>

// Função principal do programa.
//
// Todo programa em C++ inicia sua execução por esta função.
int main(int argc, char** argv)
{
    // Inicializa toda a infraestrutura do ROS2.
    //
    // Esse comando prepara o sistema para criar nós,
    // publishers, subscribers, timers, parâmetros
    // e demais recursos do framework.
    rclcpp::init(argc, argv);

    // Cria uma instância do InterfaceNode.
    //
    // std::make_shared() cria o objeto utilizando
    // um Smart Pointer, que é o padrão adotado
    // pelo ROS2 para gerenciamento automático
    // de memória.
    auto node = std::make_shared<InterfaceNode>();

    // Cria um objeto responsável por controlar
    // a frequência de execução do loop principal.
    //
    // Neste caso, o laço executará aproximadamente
    // 30 vezes por segundo (30 Hz).
    rclcpp::Rate rate(30); // 30 Hz

    // Loop principal do programa.
    //
    // Permanecerá executando enquanto o ROS2
    // estiver ativo.
    while (rclcpp::ok()) {

        // Processa todos os callbacks pendentes
        // deste nó.
        //
        // Isso inclui:
        //
        // • recebimento de novas imagens;
        // • atualização de parâmetros;
        // • qualquer outra mensagem recebida.
        //
        // Diferente do rclcpp::spin(), esta função
        // retorna imediatamente após executar os
        // callbacks disponíveis.
        rclcpp::spin_some(node);

        // Obtém a imagem mais recente recebida
        // pela InterfaceNode.
        cv::Mat img = node->getLatestImage();

        // Verifica se existe uma imagem válida.
        if (!img.empty())

            // Exibe a imagem na janela chamada
            // "Camera".
            cv::imshow("Camera", img);

        // Publica continuamente os valores atuais
        // dos sliders (Hue, Saturation e Brightness).
        //
        // Esses valores são enviados para a
        // AcquisitionNode através do tópico
        // "image_params".
        node->publishParams();

        // Aguarda aproximadamente 1 ms para que
        // a janela do OpenCV possa atualizar e,
        // ao mesmo tempo, captura alguma tecla
        // pressionada pelo usuário.
        int key = cv::waitKey(1);

        // Se a tecla pressionada for ESC (código 27),
        // encerra o programa.
        if (key == 27)
            break;

        // Aguarda o tempo restante necessário
        // para manter a frequência de 30 Hz.
        //
        // Caso o processamento tenha sido rápido,
        // esta função faz o programa "dormir"
        // até completar o período correspondente.
        rate.sleep();
    }

    // Fecha todas as janelas abertas pelo OpenCV.
    cv::destroyAllWindows();

    // Encerra corretamente toda a infraestrutura
    // do ROS2, liberando recursos e finalizando
    // as comunicações entre os nós.
    rclcpp::shutdown();

    // Retorna 0 ao sistema operacional,
    // indicando que o programa foi encerrado
    // com sucesso.
    return 0;
}