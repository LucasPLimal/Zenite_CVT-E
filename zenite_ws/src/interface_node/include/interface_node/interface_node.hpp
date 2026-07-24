// Biblioteca principal do ROS2.
// Contém toda a infraestrutura necessária para criação de nós,
// publishers, subscribers, timers, parâmetros e logs.
#include <rclcpp/rclcpp.hpp>

// Mensagem utilizada para transportar imagens entre os nós do ROS2.
#include <sensor_msgs/msg/image.hpp>

// Mensagem utilizada para enviar um ponto no espaço.
// Neste projeto representa a posição desejada do robô.
#include <geometry_msgs/msg/point.hpp>

// Biblioteca responsável por converter imagens do ROS2
// (sensor_msgs::msg::Image) para o formato OpenCV (cv::Mat)
// e vice-versa.
#include <cv_bridge/cv_bridge.hpp>

// Biblioteca principal do OpenCV.
// Disponibiliza funções de visão computacional,
// interface gráfica, desenho e manipulação de imagens.
#include <opencv2/opencv.hpp>

// Biblioteca própria do Projeto Zênite.
//
// Contém a classe PixelConverter, responsável por converter
// coordenadas em pixels para coordenadas reais (metros)
// utilizando a matriz de homografia calculada pelo
// CalibrationNode.
#include "zenite_utils/pixel_converter.hpp"

// Inclui a mensagem personalizada criada para o projeto.
//
// Essa mensagem contém os parâmetros de ajuste da câmera,
// como brilho, saturação e "hue", permitindo que a InterfaceNode
// envie esses valores para a AcquisitionNode.
#include "image_adjust_msgs/msg/image_params.hpp"

// Declaração da classe InterfaceNode.
//
// Todo nó do ROS2 normalmente herda de rclcpp::Node.
// Essa classe é responsável pela interação entre o usuário
// e o sistema, permitindo:
//
// • visualizar a imagem da câmera;
// • selecionar um ponto de destino;
// • alterar parâmetros da câmera através de sliders.
class InterfaceNode : public rclcpp::Node {

public:

    // Construtor da classe.
    //
    // É executado automaticamente quando o nó é criado.
    InterfaceNode()

        // Chama o construtor da classe base (Node),
        // definindo o nome do nó como "interface_node".
        : Node("interface_node")
    {

        // Cria um Subscriber responsável por receber
        // continuamente as imagens publicadas pela
        // AcquisitionNode.
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(

            // Nome do tópico.
            "camera_frame",

            // Tamanho da fila.
            10,

            // Callback executado sempre que uma nova
            // imagem chegar.
            std::bind(&InterfaceNode::imageCallback,
                      this,
                      std::placeholders::_1));

        // Cria um Publisher responsável por enviar
        // o ponto escolhido pelo usuário.
        desired_pos_pub_ =
            this->create_publisher<geometry_msgs::msg::Point>(

                // Tópico utilizado pelo ControlNode.
                "/desired_position",

                // Tamanho da fila.
                10);

        // Cria um Publisher responsável por enviar
        // os parâmetros de ajuste da câmera.
        params_pub_ =
            this->create_publisher<
                image_adjust_msgs::msg::ImageParams>(

                // Nome do tópico.
                "image_params",

                // Tamanho da fila.
                10);

        // Declara um parâmetro ROS2 contendo
        // o caminho do arquivo YAML onde a homografia
        // foi salva pelo CalibrationNode.
        std::string yaml_path =
            this->declare_parameter<std::string>(

                // Nome do parâmetro.
                "scale_yaml_path",

                // Valor padrão.
                "/tmp/scale.yaml");

        // Carrega a matriz de homografia armazenada
        // nesse arquivo.
        converter_.loadFromYaml(yaml_path);

        // Cria a janela onde será exibida
        // a imagem da câmera.
        cv::namedWindow("Camera");

        // Cria uma segunda janela contendo
        // os sliders de ajuste.
        cv::namedWindow("Controles");

        // Define que todos os eventos de mouse
        // da janela "Camera" serão tratados pela função
        // estática onMouse().
        cv::setMouseCallback(

            "Camera",

            onMouse,

            // Ponteiro para esta instância.
            this);

        // Cria o slider responsável pelo Hue.
        //
        // O slider manipula diretamente a variável hue_.
        cv::createTrackbar(

            "Hue",

            "Controles",

            &hue_,

            255);

        // Slider da Saturação.
        cv::createTrackbar(

            "Saturation",

            "Controles",

            &saturation_,

            255);

        // Slider do Brilho.
        cv::createTrackbar(

            "Brightness",

            "Controles",

            &brightness_,

            255);

        // Exibe mensagem informando que
        // o InterfaceNode foi inicializado.
        RCLCPP_INFO(this->get_logger(),
                    "InterfaceNode iniciado!");
    }

    // Getter responsável por devolver
    // a última imagem recebida.
    //
    // O main.cpp utiliza essa função
    // para exibir continuamente a câmera.
    cv::Mat getLatestImage() const
    {
        return latest_image_;
    }

    // Função responsável por publicar
    // os valores atuais dos sliders.
    void publishParams()
    {
        // Cria uma mensagem do tipo ImageParams.
        auto msg = image_adjust_msgs::msg::ImageParams();

        // Converte o valor inteiro do slider
        // (0–255) para ponto flutuante
        // (0.0–1.0).
        msg.hue =
            static_cast<float>(hue_) / 255.0f;

        // Mesmo procedimento para Saturação.
        msg.saturation =
            static_cast<float>(saturation_) / 255.0f;

        // Mesmo procedimento para Brilho.
        msg.brightness =
            static_cast<float>(brightness_) / 255.0f;

        // Publica a mensagem.
        params_pub_->publish(msg);
    }

private:

    // Callback executado sempre que
    // uma nova imagem chega da câmera.
    void imageCallback(
        const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try
        {
            // Converte a imagem ROS2
            // para OpenCV.
            cv_bridge::CvImagePtr cv_ptr =
                cv_bridge::toCvCopy(msg, "bgr8");

            // Verifica se a imagem é válida.
            if (!cv_ptr->image.empty())

                // Atualiza a imagem armazenada.
                latest_image_ = cv_ptr->image;
        }

        // Captura erros de conversão.
        catch (const cv_bridge::Exception& e)
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "Erro no cv_bridge: %s",
                e.what());
        }
    }

    // Callback responsável pelos cliques
    // do usuário na janela da câmera.
    static void onMouse(
        int event,
        int x,
        int y,
        int,
        void* userdata)
    {
        // Ignora qualquer evento que
        // não seja clique do botão esquerdo.
        if (event != cv::EVENT_LBUTTONDOWN)
            return;

        // Recupera o objeto InterfaceNode.
        auto* self =
            static_cast<InterfaceNode*>(userdata);

        // Verifica se existe imagem carregada.
        if (self->latest_image_.empty())
            return;

        // Cria um ponto em pixels
        // correspondente ao clique.
        cv::Point2f pixel_point(x, y);

        // Converte pixels para metros
        // utilizando a homografia.
        cv::Point2f world_point =
            self->converter_.pixelToMeter(pixel_point);

        // Cria a mensagem que será enviada.
        geometry_msgs::msg::Point pt_msg;

        // Coordenada X em metros.
        pt_msg.x = world_point.x;

        // Coordenada Y em metros.
        pt_msg.y = world_point.y;

        // Coordenada Z.
        //
        // Como o robô opera em um plano,
        // permanece sempre igual a zero.
        pt_msg.z = 0.0;

        // Publica o destino.
        self->desired_pos_pub_->publish(pt_msg);

        // Exibe no terminal
        // a conversão realizada.
        RCLCPP_INFO(
            self->get_logger(),
            "Clique na imagem: (%.1f, %.1f) -> Mundo: (%.2f, %.2f)",
            pixel_point.x,
            pixel_point.y,
            world_point.x,
            world_point.y);
    }

    // Armazena a imagem mais recente recebida.
    cv::Mat latest_image_;

    // Responsável pela conversão
    // Pixel → Metro.
    zenite_utils::PixelConverter converter_;

    // Subscriber responsável por receber
    // imagens da câmera.
    rclcpp::Subscription<
        sensor_msgs::msg::Image>::SharedPtr image_sub_;

    // Publisher responsável por enviar
    // a posição desejada do robô.
    rclcpp::Publisher<
        geometry_msgs::msg::Point>::SharedPtr desired_pos_pub_;

    // Publisher responsável por enviar
    // os parâmetros da câmera.
    rclcpp::Publisher<
        image_adjust_msgs::msg::ImageParams>::SharedPtr params_pub_;

    // Valor atual do slider Hue.
    int hue_ = 255;

    // Valor atual do slider Saturação.
    int saturation_ = 172;

    // Valor atual do slider Brilho.
    int brightness_ = 150;
};