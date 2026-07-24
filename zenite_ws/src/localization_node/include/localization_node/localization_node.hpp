// Biblioteca principal do ROS2.
//
// Disponibiliza toda a infraestrutura necessária para criação
// de nós, publishers, subscribers, timers, parâmetros e logs.
#include <rclcpp/rclcpp.hpp>

// Mensagem utilizada para transportar imagens entre os nós do ROS2.
#include <sensor_msgs/msg/image.hpp>

// Mensagem utilizada para enviar um vetor de números reais.
//
// Neste projeto ela transporta a posição atual do robô
// no formato:
// [x, y]
#include <std_msgs/msg/float64_multi_array.hpp>

// Biblioteca responsável por converter imagens do ROS2
// (sensor_msgs::msg::Image) para imagens OpenCV (cv::Mat)
// e vice-versa.
#include <cv_bridge/cv_bridge.hpp>

// Biblioteca principal do OpenCV.
//
// Disponibiliza todas as funções de processamento de imagens,
// desenho, manipulação de pixels e interface gráfica.
#include <opencv2/opencv.hpp>

// Biblioteca responsável pelos algoritmos de rastreamento
// (tracking) do OpenCV.
//
// Atualmente o Projeto Zênite utiliza o TrackerCSRT,
// embora anteriormente tenha utilizado o KCF.
#include <opencv2/tracking.hpp>

// Biblioteca própria do Projeto Zênite.
//
// Contém a classe PixelConverter,
// responsável por converter coordenadas em pixels
// para coordenadas reais utilizando a homografia
// calculada pelo CalibrationNode.
#include "zenite_utils/pixel_converter.hpp"

// Permite utilizar literais de tempo,
// como 50ms, 100ms, 1s etc.
using namespace std::chrono_literals;

// Declaração da classe LocalizationNode.
//
// Este nó é responsável por:
//
// • receber imagens da câmera;
// • rastrear continuamente o robô;
// • converter sua posição de pixels para metros;
// • publicar essa posição para o ControlNode.
class LocalizationNode : public rclcpp::Node {

public:

    // Construtor da classe.
    //
    // Executado automaticamente quando o nó é criado.
    LocalizationNode()

        // Inicializa a classe base do ROS2,
        // definindo o nome do nó.
        : Node("tracking_node"),

          // Armazena o instante atual.
          //
          // Será utilizado para controlar
          // a frequência máxima de publicação.
          last_publish_time_(this->now()),

          // Define um intervalo mínimo de
          // 50 milissegundos entre publicações.
          cooldown_duration_(50ms)
    {

        // Cria um Subscriber responsável por receber
        // continuamente as imagens publicadas
        // pela AcquisitionNode.
        image_sub_ =
            this->create_subscription<
                sensor_msgs::msg::Image>(

                // Nome do tópico.
                "camera_frame",

                // Tamanho da fila.
                10,

                // Callback executado sempre que
                // uma nova imagem chegar.
                std::bind(
                    &LocalizationNode::imageCallback,
                    this,
                    std::placeholders::_1));

        // Cria o Publisher responsável por enviar
        // a posição atual do robô.
        initial_pos_pub_ =
            this->create_publisher<
                std_msgs::msg::Float64MultiArray>(

                // Nome do tópico.
                "/current_position",

                // Tamanho da fila.
                10);

        // Declara um parâmetro ROS2 contendo
        // o caminho do arquivo YAML da homografia.
        std::string yaml_path =
            this->declare_parameter<std::string>(

                "scale_yaml_path",

                "/tmp/scale.yaml");

        // Carrega a homografia salva
        // pelo CalibrationNode.
        converter_.loadFromYaml(yaml_path);

        // Informa no terminal que o nó
        // foi iniciado corretamente.
        RCLCPP_INFO(
            this->get_logger(),
            "Localization_Node iniciado!");
    }

    // Callback executado sempre que uma nova
    // imagem chega da câmera.
    void imageCallback(
        const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try {

            // Converte a imagem ROS2
            // para OpenCV.
            cv_bridge::CvImagePtr cv_ptr =
                cv_bridge::toCvCopy(msg, "bgr8");

            // Verifica se a imagem é válida.
            if (!cv_ptr->image.empty())

                // Atualiza a última imagem disponível.
                latest_image_ = cv_ptr->image;

        }

        // Captura possíveis erros
        // durante a conversão.
        catch (const cv_bridge::Exception& e) {

            RCLCPP_ERROR(
                this->get_logger(),
                "Erro no cv_bridge: %s",
                e.what());
        }
    }

    // Getter responsável por devolver
    // a última imagem recebida.
    //
    // O main.cpp utiliza essa função
    // para alimentar o algoritmo CSRT.
    cv::Mat getLatestImage() const
    {
        return latest_image_;
    }

    // Verifica se já existe
    // alguma imagem válida armazenada.
    bool hasImage() const
    {
        return !latest_image_.empty();
    }

    // Publica a posição atual do robô.
    //
    // Recebe como entrada um ponto
    // já convertido para metros.
    void publishPosition(
        const cv::Point2f& world_point)
    {

        // Obtém o instante atual.
        auto now = this->now();

        // Verifica se o tempo decorrido
        // desde a última publicação é menor
        // que o cooldown.
        //
        // Caso seja, simplesmente retorna,
        // evitando publicar mensagens
        // em excesso.
        if (now - last_publish_time_
            < cooldown_duration_) {

            return;
        }

        // Atualiza o instante
        // da última publicação.
        last_publish_time_ = now;

        // Cria uma mensagem
        // Float64MultiArray.
        std_msgs::msg::Float64MultiArray msg;

        // Armazena as coordenadas
        // X e Y do robô.
        msg.data = {

            world_point.x,

            world_point.y
        };

        // Publica a posição atual.
        initial_pos_pub_->publish(msg);

        // Exibe no terminal
        // a posição enviada.
        RCLCPP_INFO(
            this->get_logger(),

            "Posicao enviada: (%.3f, %.3f) m",

            world_point.x,

            world_point.y);
    }

    // Objeto responsável por realizar
    // a conversão Pixel → Metro.
    //
    // Utiliza a homografia carregada
    // do arquivo YAML.
    zenite_utils::PixelConverter converter_;

private:

    // Subscriber responsável por receber
    // continuamente as imagens da câmera.
    rclcpp::Subscription<
        sensor_msgs::msg::Image>::SharedPtr image_sub_;

    // Publisher responsável por enviar
    // a posição atual do robô.
    rclcpp::Publisher<
        std_msgs::msg::Float64MultiArray>::SharedPtr initial_pos_pub_;

    // Armazena sempre a última imagem
    // recebida da câmera.
    cv::Mat latest_image_;

    // Instante da última publicação
    // realizada.
    rclcpp::Time last_publish_time_;

    // Intervalo mínimo permitido
    // entre duas publicações consecutivas.
    rclcpp::Duration cooldown_duration_;
};