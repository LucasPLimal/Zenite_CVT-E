// Biblioteca principal do ROS 2.
// Fornece a classe Node, publishers, subscribers, timers, logs, etc.
#include <rclcpp/rclcpp.hpp>

// Mensagem ROS utilizada para transmitir imagens.
#include <sensor_msgs/msg/image.hpp>

// Biblioteca responsável por converter mensagens ROS em imagens OpenCV.
#include <cv_bridge/cv_bridge.hpp>

// Biblioteca principal do OpenCV.
#include <opencv2/opencv.hpp>

// Classe criada no projeto responsável pela conversão
// entre coordenadas de pixel e coordenadas reais (metros).
#include "zenite_utils/pixel_converter.hpp"

// Facilita o uso do placeholder _1 no std::bind.
using std::placeholders::_1;

// Classe responsável pela calibração da câmera.
// Herda de rclcpp::Node para se tornar um nó ROS2.
class CalibrationNode : public rclcpp::Node
{
public:

    // ==========================
    // CONSTRUTOR
    // ==========================
    CalibrationNode()
    : Node("calibration_node")   // Nome do nó no ROS2.
    {

        // Cria um subscriber para receber imagens publicadas
        // pelo AcquisitionNode no tópico "camera_frame".
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "camera_frame",               // tópico
            10,                           // tamanho da fila
            std::bind(&CalibrationNode::imageCallback, this, _1));



        // Declara um parâmetro ROS.
        // Esse parâmetro define onde será salvo o arquivo YAML
        // contendo a matriz de transformação.
        scale_yaml_path_ =
            this->declare_parameter<std::string>(
                "scale_yaml_path",
                "/tmp/scale.yaml");



        // Cria uma janela do OpenCV.
        // Nela será exibida a imagem para o usuário clicar.
        cv::namedWindow("Calibration - Defina os Pontos");



        // Associa um callback de mouse à janela.
        // Sempre que houver clique, a função onMouse será chamada.
        cv::setMouseCallback(
            "Calibration - Defina os Pontos",
            onMouse,
            this);



        // Apenas informa no terminal que o nó iniciou.
        RCLCPP_INFO(this->get_logger(), "CalibrationNode iniciado!");
    }

private:

    // ======================================================
    // CALLBACK DA IMAGEM
    // Executado sempre que uma nova imagem chega.
    // ======================================================
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try
        {

            // Converte a mensagem ROS para uma imagem OpenCV.
            cv::Mat frame =
                cv_bridge::toCvCopy(msg, "bgr8")->image;



            // Desenha todos os pontos clicados.
            for (const auto& p : image_points_)
                cv::circle(
                    frame,
                    p,
                    5,
                    cv::Scalar(0,0,255),
                    -1);



            // Se existir mais de um ponto,
            // desenha linhas ligando todos eles.
            if (image_points_.size() > 1)
            {
                for (size_t i = 0; i < image_points_.size(); ++i)
                    cv::line(
                        frame,
                        image_points_[i],
                        image_points_[(i+1)%image_points_.size()],
                        cv::Scalar(255,0,0),
                        2);
            }



            // Exibe a imagem atualizada.
            cv::imshow(
                "Calibration - Defina os Pontos",
                frame);



            // Atualiza a janela.
            cv::waitKey(1);



            // Quando já existem:
            // - 4 pontos da imagem
            // - 4 pontos do mundo
            // - ainda não salvou
            if (image_points_.size() == 4 &&
                world_points_.size() == 4 &&
                !saved_)
            {

                RCLCPP_INFO(
                    this->get_logger(),
                    "Calculando transformação e salvando escala...");



                // Calcula a homografia entre os pontos
                // da imagem e os pontos reais.
                converter_.setReferencePoints(
                    image_points_,
                    world_points_);




                // Salva a matriz calculada no YAML.
                converter_.saveToYaml(scale_yaml_path_);




                // Impede que esse cálculo seja feito novamente.
                saved_ = true;



                RCLCPP_INFO(
                    this->get_logger(),
                    "Transformação salva em %s",
                    scale_yaml_path_.c_str());
            }
        }

        // Caso ocorra erro na conversão da imagem.
        catch (const cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "Erro no cv_bridge: %s",
                e.what());
        }
    }



    // ======================================================
    // CALLBACK DO MOUSE
    // Executado quando o usuário interage com a janela.
    // ======================================================
    static void onMouse(
        int event,
        int x,
        int y,
        int,
        void* userdata)
    {

        // Recupera o ponteiro da própria classe.
        auto* self =
            static_cast<CalibrationNode*>(userdata);



        // Se clicou com o botão esquerdo
        // e ainda existem menos de quatro pontos.
        if (event == cv::EVENT_LBUTTONDOWN &&
            self->image_points_.size() < 4)
        {

            // Salva o ponto clicado.
            self->image_points_.push_back(
                cv::Point2f(x,y));



            RCLCPP_INFO(
                self->get_logger(),
                "Ponto de imagem (%d, %d) registrado",
                x,
                y);
        }



        // Assim que os quatro pontos forem definidos,
        // cria automaticamente os quatro pontos reais.
        if (self->image_points_.size() == 4 &&
            self->world_points_.empty())
        {

            // Esses valores representam
            // os cantos físicos do campo.
            self->world_points_ =
            {
                cv::Point2f(0.0f,0.0f),

                cv::Point2f(2.75f,0.0f),

                cv::Point2f(2.75f,2.25f),

                cv::Point2f(0.0f,2.25f)
            };



            RCLCPP_INFO(
                self->get_logger(),
                "Pontos do mundo definidos automaticamente (2.75 x 2.25 m).");
        }
    }



    // ======================================================
    // VARIÁVEIS DA CLASSE
    // ======================================================

    // Subscriber da câmera.
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;



    // Caminho do arquivo YAML.
    std::string scale_yaml_path_;



    // Pontos clicados na imagem.
    std::vector<cv::Point2f> image_points_;



    // Coordenadas reais correspondentes aos pontos.
    std::vector<cv::Point2f> world_points_;



    // Classe responsável por calcular e salvar
    // a transformação entre pixel e metro.
    zenite_utils::PixelConverter converter_;



    // Evita salvar diversas vezes.
    bool saved_ = false;
};