// Inclui o arquivo de cabeçalho da classe LocalizationNode
#include <localization_node/localization_node.hpp>

// Função principal do programa
int main(int argc, char** argv)
{
    // Inicializa o ROS 2
    rclcpp::init(argc, argv);

    // Cria uma instância do nó LocalizationNode
    auto node = std::make_shared<LocalizationNode>();

    // Define a taxa de execução do loop principal em 30 Hz
    rclcpp::Rate rate(30);

    // Cria um objeto do rastreador CSRT do OpenCV
    // Esse algoritmo será responsável por acompanhar o robô entre os quadros da câmera
    cv::Ptr<cv::Tracker> tracker = cv::TrackerCSRT::create();

    // Variável que armazenará a região de interesse (ROI) onde está o robô
    cv::Rect2d roi;

    // Indica se o rastreador já foi inicializado
    bool inicializado = false;

    // Vetor utilizado para armazenar o histórico das posições do robô
    // Serve apenas para desenhar a trajetória na tela
    std::vector<cv::Point> trajetoria;

    // Loop principal do programa
    while (rclcpp::ok()) {

        // Processa todos os callbacks pendentes do ROS 2
        rclcpp::spin_some(node);

        // Se ainda não existe imagem disponível, espera o próximo ciclo
        if (!node->hasImage()) {
            rate.sleep();
            continue;
        }

        // Obtém o frame mais recente enviado pelo AcquisitionNode
        cv::Mat frame = node->getLatestImage();

        // Caso a imagem esteja vazia, pula esta iteração
        if (frame.empty()) continue;

        // Verifica se o rastreador ainda não foi inicializado
        if (!inicializado) {

            // Permite ao usuário selecionar manualmente o robô na primeira imagem
            roi = cv::selectROI("Selecione o carrinho", frame);

            // Verifica se foi realmente selecionada uma região válida
            if (roi.width > 0 && roi.height > 0) {

                // Inicializa o rastreador utilizando a ROI escolhida
                tracker->init(frame, roi);

                // Marca que o rastreador já está pronto para funcionar
                inicializado = true;

                // Fecha a janela de seleção
                cv::destroyWindow("Selecione o carrinho");

                // Exibe mensagem informando que o rastreamento começou
                RCLCPP_INFO(node->get_logger(), "Rastreamento inicializado.");
            }
        }
        else {

            // Variável que receberá a nova posição calculada pelo CSRT
            cv::Rect roi_atual;

            // Atualiza o rastreador utilizando o frame atual
            // Retorna true caso consiga localizar o robô
            bool sucesso = tracker->update(frame, roi_atual);

            // Se o rastreamento foi bem sucedido
            if (sucesso) {

                // Atualiza a ROI armazenada
                roi = roi_atual;

                // Desenha um retângulo verde ao redor do robô
                cv::rectangle(frame, roi, cv::Scalar(0, 255, 0), 2);

                // Calcula o centro da ROI
                // Esse será considerado a posição do robô na imagem
                cv::Point2f centro(
                    roi.x + roi.width / 2.0,
                    roi.y + roi.height / 2.0);

                // Adiciona esse ponto ao histórico da trajetória
                trajetoria.push_back(centro);

                // Desenha toda a trajetória percorrida pelo robô
                for (size_t i = 1; i < trajetoria.size(); ++i)
                    cv::line(frame,
                             trajetoria[i - 1],
                             trajetoria[i],
                             cv::Scalar(255, 0, 0),
                             2);

                // Converte a posição em pixels para coordenadas reais em metros
                // utilizando a homografia calculada no CalibrationNode
                cv::Point2f world_point = node->converter_.pixelToMeter(centro);

                // Publica a posição do robô no tópico ROS
                // A própria função já possui um cooldown para evitar excesso de mensagens
                node->publishPosition(world_point);

            }
            else {

                // Caso o rastreador perca o robô,
                // exibe uma mensagem em vermelho na imagem
                cv::putText(frame,
                            "Falha no rastreamento",
                            cv::Point(50, 50),
                            cv::FONT_HERSHEY_SIMPLEX,
                            0.75,
                            cv::Scalar(0, 0, 255),
                            2);
            }
        }

        // Exibe a imagem com todos os desenhos feitos
        cv::imshow("Rastreamento do Carrinho", frame);

        // Aguarda aproximadamente 30 ms
        // Se o usuário pressionar ESC (27), encerra o programa
        if (cv::waitKey(30) == 27)
            break;

        // Aguarda até completar os 30 Hz definidos anteriormente
        rate.sleep();
    }

    // Finaliza o ROS 2
    rclcpp::shutdown();

    // Fecha todas as janelas do OpenCV
    cv::destroyAllWindows();

    // Finaliza o programa informando que tudo ocorreu corretamente
    return 0;
}