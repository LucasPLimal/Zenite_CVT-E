#pragma once

// Funções matemáticas (M_PI etc.).
#include <cmath>

// Utilitários da biblioteca padrão.
#include <chrono>
#include <map>
#include <string>

// Biblioteca principal do ROS2.
#include <rclcpp/rclcpp.hpp>

// Mensagem utilizada para transportar imagens entre os nós do ROS2.
#include <sensor_msgs/msg/image.hpp>

// Mensagem utilizada para enviar um vetor de números reais.
//
// Para cada robô <id>, este nó publica um único tópico:
// • /robot_<id>/current_pose -> [x, y, theta]
//
// x e y em metros e theta em radianos, no referencial do mundo
// definido pela calibração (homografia).
#include <std_msgs/msg/float64_multi_array.hpp>

// Conversão de imagens ROS2 <-> OpenCV.
#include <cv_bridge/cv_bridge.hpp>

// Biblioteca principal do OpenCV.
#include <opencv2/opencv.hpp>

// Conversão pixel -> metro (homografia do CalibrationNode).
#include "zenite_utils/pixel_converter.hpp"

// Permite utilizar literais de tempo, como 50ms, 100ms, 1s etc.
using namespace std::chrono_literals;

// Este nó é responsável por:
//
// • receber imagens da câmera;
// • (o main) detectar os marcadores ArUco sobre os robôs;
// • converter posição e orientação de pixels para metros;
// • publicar a pose (x, y, theta) de cada robô.
class LocalizationNode : public rclcpp::Node {

public:

    using ArrayMsg = std_msgs::msg::Float64MultiArray;
    using ArrayPub = rclcpp::Publisher<ArrayMsg>::SharedPtr;

    LocalizationNode()
        : Node("tracking_node"),

          // Intervalo mínimo de 50 ms entre publicações (por robô).
          cooldown_duration_(50ms)
    {

        // Subscriber das imagens publicadas pela AcquisitionNode.
        image_sub_ =
            this->create_subscription<
                sensor_msgs::msg::Image>(

                "camera_frame",

                10,

                std::bind(
                    &LocalizationNode::imageCallback,
                    this,
                    std::placeholders::_1));

        // Caminho do arquivo YAML da homografia.
        std::string yaml_path =
            this->declare_parameter<std::string>(

                "scale_yaml_path",

                "/tmp/scale.yaml");

        // Carrega a homografia salva pelo CalibrationNode.
        converter_.loadFromYaml(yaml_path);

        RCLCPP_INFO(
            this->get_logger(),
            "Localization_Node iniciado!");
    }

    // Callback executado sempre que uma nova imagem chega da câmera.
    void imageCallback(
        const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try {

            cv_bridge::CvImagePtr cv_ptr =
                cv_bridge::toCvCopy(msg, "bgr8");

            if (!cv_ptr->image.empty())
                latest_image_ = cv_ptr->image;

        }
        catch (const cv_bridge::Exception& e) {

            RCLCPP_ERROR(
                this->get_logger(),
                "Erro no cv_bridge: %s",
                e.what());
        }
    }

    // Devolve a última imagem recebida.
    cv::Mat getLatestImage() const
    {
        return latest_image_;
    }

    // Verifica se já existe alguma imagem válida armazenada.
    bool hasImage() const
    {
        return !latest_image_.empty();
    }

    // Publica a pose de um robô em /robot_<id>/current_pose.
    //
    // id          : ID do marcador ArUco do robô.
    // world_point : posição em metros (centro do marcador).
    // theta       : orientação em radianos, no referencial do mundo.
    //
    // Cada robô tem o seu próprio cooldown, então um robô
    // não bloqueia a publicação do outro.
    void publishPose(
        int id,
        const cv::Point2f& world_point,
        double theta)
    {

        // Primeira vez que este ID aparece: cria o publisher dele.
        auto it = robots_.find(id);
        if (it == robots_.end()) {

            RobotPub r;
            const std::string topic =
                "/robot_" + std::to_string(id) + "/current_pose";

            r.pose = this->create_publisher<ArrayMsg>(topic, 10);
            r.last = this->now() - cooldown_duration_;

            it = robots_.emplace(id, std::move(r)).first;

            RCLCPP_INFO(
                this->get_logger(),
                "Novo robo detectado: ID %d (topico %s)",
                id,
                topic.c_str());
        }

        RobotPub& r = it->second;

        // Respeita o intervalo mínimo entre publicações deste robô.
        const auto now = this->now();
        if (now - r.last < cooldown_duration_)
            return;
        r.last = now;

        // Mensagem com a pose completa: [x, y, theta].
        ArrayMsg pose_msg;
        pose_msg.data = {world_point.x, world_point.y, theta};

        r.pose->publish(pose_msg);

        // No máximo 1 mensagem por segundo no terminal.
        RCLCPP_INFO_THROTTLE(
            this->get_logger(), *this->get_clock(), 1000,
            "Pose enviada: ID %d (%.3f, %.3f) m, theta = %.1f graus",
            id,
            world_point.x,
            world_point.y,
            theta * 180.0 / M_PI);
    }

    // Objeto responsável pela conversão Pixel -> Metro.
    zenite_utils::PixelConverter converter_;

private:

    // Publisher e último instante de publicação de um robô.
    struct RobotPub {
        ArrayPub pose;
        rclcpp::Time last;
    };

    // Subscriber das imagens da câmera.
    rclcpp::Subscription<
        sensor_msgs::msg::Image>::SharedPtr image_sub_;

    // Um publisher por ID de robô.
    std::map<int, RobotPub> robots_;

    // Última imagem recebida da câmera.
    cv::Mat latest_image_;

    // Intervalo mínimo entre duas publicações do mesmo robô.
    rclcpp::Duration cooldown_duration_;
};
