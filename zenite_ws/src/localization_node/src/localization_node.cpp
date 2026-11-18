// Inclui o arquivo de cabeçalho da classe LocalizationNode
#include <localization_node/localization_node.hpp>

// Inclui o detector de marcadores ArUco
#include <localization_node/aruco_tracker.hpp>

#include <cmath>
#include <cstdio>
#include <deque>
#include <map>

// Função principal do programa
int main(int argc, char** argv)
{
    // Inicializa o ROS 2
    rclcpp::init(argc, argv);

    // Cria uma instância do nó LocalizationNode
    auto node = std::make_shared<LocalizationNode>();

    // Alteração feita por Danilo: a janela do OpenCV agora é opcional.
    // Com show_window:=false (padrão) o nó roda sem display, como no Raspberry Pi
    // via SSH, em que cv::imshow derrubava o processo (Qt "xcb" não encontrado).
    // Para ver a janela num computador com tela: show_window:=true.
    node->declare_parameter<bool>("show_window", false);
    const bool mostrar_janela = node->get_parameter("show_window").as_bool();

    // Alteração feita por Danilo: o nó só lia a homografia ao iniciar, então uma
    // calibração feita depois (pela interface web) nunca era vista. Agora, enquanto
    // não houver homografia, ele tenta recarregar o YAML a cada 2 s.
    const std::string caminho_yaml =
        node->get_parameter("scale_yaml_path").as_string();
    auto ultima_tentativa = std::chrono::steady_clock::now();

    // Define a taxa de execução do loop principal em 30 Hz
    rclcpp::Rate rate(30);

    // Detector dos marcadores ArUco
    ArucoTracker tracker;

    // Histórico das posições de cada robô (por ID),
    // usado apenas para desenhar a trajetória na tela.
    // Guarda no máximo kMaxTrajetoria pontos por robô.
    constexpr size_t kMaxTrajetoria = 500;
    std::map<int, std::deque<cv::Point>> trajetorias;

    // Cores usadas para diferenciar os robôs (BGR), escolhidas pelo ID.
    const cv::Scalar cores[] = {
        cv::Scalar(0, 255, 0),     // verde
        cv::Scalar(0, 0, 255),     // vermelho
        cv::Scalar(255, 0, 0),     // azul
        cv::Scalar(0, 255, 255),   // amarelo
        cv::Scalar(255, 0, 255),   // magenta
        cv::Scalar(255, 255, 0)    // ciano
    };

    // Converte Point2f em Point (inteiro) para desenhar.
    auto P = [](const cv::Point2f& p) {
        return cv::Point(cvRound(p.x), cvRound(p.y));
    };

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
        if (frame.empty()) {
            rate.sleep();
            continue;
        }

        // Alteração feita por Danilo: recarrega a homografia se ainda não houver.
        if (!node->converter_.ready()) {
            const auto agora = std::chrono::steady_clock::now();
            if (agora - ultima_tentativa > std::chrono::seconds(2)) {
                ultima_tentativa = agora;
                node->converter_.loadFromYaml(caminho_yaml);
            }
        }

        // Procura os marcadores no frame original (sem desenhos)
        // e mede quanto tempo a detecção levou
        cv::TickMeter cronometro;
        cronometro.start();
        const auto deteccoes = tracker.detect(frame);
        cronometro.stop();

        // Os desenhos vão numa cópia, para não atrapalhar a detecção
        cv::Mat tela = frame.clone();

        if (!deteccoes.empty()) {

            // Linha de texto atual (para listar um robô por linha)
            int linha = 0;

            for (const auto& d : deteccoes) {

                const cv::Scalar cor = cores[d.id % 6];

                // Desenha o contorno do marcador
                for (int i = 0; i < 4; ++i)
                    cv::line(tela,
                             P(d.corners[i]),
                             P(d.corners[(i + 1) % 4]),
                             cor, 2);

                // Desenha a direção da frente (centro -> frente)
                cv::arrowedLine(tela, P(d.center), P(d.front),
                                cv::Scalar(0, 0, 255), 2);

                // Escreve o ID ao lado do marcador
                char id_txt[16];
                std::snprintf(id_txt, sizeof(id_txt), "ID %d", d.id);
                cv::putText(tela, id_txt,
                            P(d.center) + cv::Point(10, -10),
                            cv::FONT_HERSHEY_SIMPLEX, 0.6, cor, 2);

                // Adiciona o centro ao histórico da trajetória deste robô
                auto& traj = trajetorias[d.id];
                traj.push_back(P(d.center));
                if (traj.size() > kMaxTrajetoria) traj.pop_front();

                // Desenha a trajetória percorrida
                for (size_t i = 1; i < traj.size(); ++i)
                    cv::line(tela, traj[i - 1], traj[i], cor, 2);

                // Alteração feita por Danilo: sem homografia não dá para converter
                // para metros (pixelToMeter lançaria exceção e derrubaria o nó).
                // O marcador continua desenhado, mas a pose não é publicada.
                if (!node->converter_.ready()) {
                    RCLCPP_WARN_THROTTLE(
                        node->get_logger(), *node->get_clock(), 2000,
                        "Marcador %d visto, mas sem calibração: a pose não será "
                        "publicada. Calibre pela interface web.", d.id);
                    continue;
                }

                // Converte centro e frente de pixels para metros
                // utilizando a homografia calculada no CalibrationNode
                cv::Point2f centro_m = node->converter_.pixelToMeter(d.center);
                cv::Point2f frente_m = node->converter_.pixelToMeter(d.front);

                // Orientação no referencial do mundo:
                // ângulo do vetor centro -> frente, em radianos
                const double theta = std::atan2(frente_m.y - centro_m.y,
                                                frente_m.x - centro_m.x);

                // Publica a pose do robô (com cooldown próprio por ID)
                node->publishPose(d.id, centro_m, theta);

                // Mostra o ângulo estimado na imagem, uma linha por robô
                char texto[64];
                std::snprintf(texto, sizeof(texto),
                              "ID %d: theta = %.1f graus",
                              d.id, theta * 180.0 / M_PI);
                cv::putText(tela, texto,
                            cv::Point(50, 50 + 30 * linha),
                            cv::FONT_HERSHEY_SIMPLEX, 0.75, cor, 2);
                ++linha;
            }

            // Imprime no terminal, no máximo 1 vez por segundo:
            // quantos marcadores foram vistos e o tempo de detecção
            RCLCPP_INFO_THROTTLE(
                node->get_logger(), *node->get_clock(), 1000,
                "Marcadores detectados: %zu, deteccao=%.2f ms",
                deteccoes.size(), cronometro.getTimeMilli());
        }
        else {

            // Nenhum marcador encontrado:
            // avisa no terminal (no máximo 1 vez por segundo)
            RCLCPP_WARN_THROTTLE(
                node->get_logger(), *node->get_clock(), 1000,
                "Nenhum marcador encontrado");

            // e exibe uma mensagem em vermelho na imagem
            cv::putText(tela,
                        "Nenhum marcador encontrado",
                        cv::Point(50, 50),
                        cv::FONT_HERSHEY_SIMPLEX,
                        0.75,
                        cv::Scalar(0, 0, 255),
                        2);
        }

        // Alteração feita por Danilo: exibe a janela só se show_window for true
        if (mostrar_janela) {

            // Exibe a imagem com todos os desenhos feitos
            cv::imshow("Rastreamento dos Carrinhos", tela);

            // Aguarda aproximadamente 30 ms
            // Se o usuário pressionar ESC (27), encerra o programa
            if (cv::waitKey(30) == 27)
                break;
        }

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
