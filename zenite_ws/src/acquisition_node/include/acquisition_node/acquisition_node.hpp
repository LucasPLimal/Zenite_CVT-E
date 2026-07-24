// ========================= BIBLIOTECAS =========================

// Biblioteca principal do ROS 2 em C++.
// Permite criar nós (Nodes), publishers, subscribers, timers,
// utilizar logs e todas as funcionalidades básicas do ROS.
#include <rclcpp/rclcpp.hpp>

// Define a mensagem Image do ROS 2.
// Essa mensagem será utilizada para publicar os frames da câmera.
#include <sensor_msgs/msg/image.hpp>

// Biblioteca responsável por converter imagens entre OpenCV (cv::Mat)
// e o formato de imagem utilizado pelo ROS (sensor_msgs::msg::Image).
#include <cv_bridge/cv_bridge.hpp>

// Biblioteca principal do OpenCV.
// Fornece captura de vídeo, processamento de imagens e visão computacional.
#include <opencv2/opencv.hpp>

// Mensagem personalizada criada para o projeto.
// Ela contém os parâmetros de ajuste da câmera
// (hue, saturation e brightness).
#include "image_adjust_msgs/msg/image_params.hpp"

// Biblioteca do Video4Linux2 (V4L2).
// Permite acessar diretamente os controles internos da câmera,
// como brilho, contraste, saturação e exposição.
#include <linux/videodev2.h>

// Biblioteca utilizada para abrir arquivos e dispositivos
// do Linux (como /dev/video0).
#include <fcntl.h>

// Biblioteca que permite enviar comandos diretamente ao driver
// da câmera utilizando ioctl().
#include <sys/ioctl.h>

// Biblioteca utilizada para fechar arquivos/dispositivos abertos.
#include <unistd.h>

// Biblioteca padrão para entrada e saída de dados.
#include <iostream>


// ===============================================================
// Classe responsável pela aquisição das imagens da câmera.
// ===============================================================
class AcquisitionNode : public rclcpp::Node {

public:

    // ===========================================================
    // Construtor da classe.
    // Executado automaticamente quando o nó é criado.
    // ===========================================================
    AcquisitionNode() : Node("acquisition_node") {

        // Cria um publisher responsável por publicar imagens
        // no tópico "camera_frame".
        //
        // Tipo da mensagem:
        // sensor_msgs::msg::Image
        //
        // Fila de até 10 mensagens.
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
            "camera_frame", 10);


        // Cria um subscriber responsável por receber
        // alterações dos parâmetros da câmera.
        //
        // Sempre que chegar uma mensagem em /image_params,
        // a função params_callback() será executada.
        params_sub_ =
            this->create_subscription<image_adjust_msgs::msg::ImageParams>(

                "/image_params",

                10,

                std::bind(
                    &AcquisitionNode::params_callback,
                    this,
                    std::placeholders::_1));



        // Cria um timer.
        //
        // A cada 33 ms (aproximadamente 30 FPS)
        // a função publish_frame() será chamada.
        timer_ = this->create_wall_timer(

            std::chrono::milliseconds(33),

            std::bind(
                &AcquisitionNode::publish_frame,
                this)
        );



        // =======================================================
        // Abre a câmera utilizando OpenCV.
        //
        // O índice 0 corresponde normalmente
        // à primeira câmera conectada ao computador.
        // =======================================================
        cap_.open(2);


        // Verifica se a câmera foi aberta corretamente.
        if (!cap_.isOpened()) {

            RCLCPP_ERROR(
                this->get_logger(),
                "Não foi possível abrir a câmera!");
        }



        // =======================================================
        // Abre o dispositivo da câmera utilizando V4L2.
        //
        // Isso permite controlar parâmetros internos da webcam.
        // =======================================================
        fd_ = open("/dev/video2", O_RDWR);


        // Caso ocorra erro.
        if (fd_ == -1) {

            RCLCPP_ERROR(
                this->get_logger(),
                "Erro ao acessar /dev/video0");
        }

        else {

            // ===================================================
            // Desativa a exposição automática da câmera.
            //
            // Isso evita que a luminosidade da imagem fique
            // variando constantemente durante a operação.
            // ===================================================

            struct v4l2_control ctrl{};

            ctrl.id = V4L2_CID_EXPOSURE_AUTO;

            ctrl.value = V4L2_EXPOSURE_MANUAL;

            if (ioctl(fd_, VIDIOC_S_CTRL, &ctrl) == -1) {

                RCLCPP_ERROR(
                    this->get_logger(),
                    "Erro ao desativar Auto Exposure");
            }
        }
    }



    // ===========================================================
    // Destrutor.
    //
    // Executado automaticamente quando o nó é encerrado.
    // ===========================================================
    ~AcquisitionNode() {

        // Fecha o dispositivo da câmera caso esteja aberto.
        if (fd_ != -1) {

            close(fd_);
        }
    }



    // ===========================================================
    // Callback executado sempre que chegam novos parâmetros
    // vindos do InterfaceNode.
    // ===========================================================
    void params_callback(

        const image_adjust_msgs::msg::ImageParams::SharedPtr msg)
    {

        // Atualiza os parâmetros armazenados.

        hue_ = msg->hue;

        saturation_ = msg->saturation;

        brightness_ = msg->brightness;


        // Envia imediatamente os novos valores para a câmera.
        send_params();
    }



    // ===========================================================
    // Envia os parâmetros atuais para o driver da câmera.
    // ===========================================================
    void send_params() {

        // Se a câmera não foi aberta,
        // não faz nada.
        if (fd_ == -1)
            return;


        // Ajusta brilho.
        if (brightness_ >= 0)

            set_control(
                V4L2_CID_BRIGHTNESS,
                brightness_ * 255);


        // Ajusta saturação.
        if (saturation_ >= 0)

            set_control(
                V4L2_CID_SATURATION,
                saturation_ * 255);


        // Ajusta contraste.
        //
        // OBS:
        // No código o parâmetro hue está sendo utilizado
        // para controlar o contraste.
        if (hue_ >= 0)

            set_control(
                V4L2_CID_CONTRAST,
                hue_ * 255);
    }



    // ===========================================================
    // Lê o valor atual de algum controle da câmera.
    //
    // Exemplo:
    // brilho
    // contraste
    // saturação
    // ===========================================================
    int get_control(__u32 id)
    {

        if (fd_ == -1)
            return -1;

        struct v4l2_control control{};

        control.id = id;

        if (ioctl(fd_, VIDIOC_G_CTRL, &control) == -1)

            return -1;

        return control.value;
    }



    // ===========================================================
    // Valores atuais dos controles.
    //
    // Variam normalmente entre 0 e 1.
    // ===========================================================
    float hue_ = 0.5f;

    float saturation_ = 0.5f;

    float brightness_ = 0.5f;



private:

    // ===========================================================
    // Captura um frame da câmera
    // e publica no ROS.
    //
    // É chamada automaticamente pelo timer.
    // ===========================================================
    void publish_frame()
    {

        // Imagem temporária do OpenCV.
        cv::Mat frame;


        // Captura um frame da câmera.
        cap_ >> frame;


        // Caso a imagem seja inválida,
        // encerra a função.
        if (frame.empty())
            return;


        // Converte cv::Mat
        // para sensor_msgs::msg::Image.
        auto msg =

            cv_bridge::CvImage(

                std_msgs::msg::Header(),

                "bgr8",

                frame)

            .toImageMsg();


        // Publica a imagem.
        publisher_->publish(*msg);
    }



    // ===========================================================
    // Altera algum parâmetro da câmera
    // utilizando Video4Linux2.
    // ===========================================================
    void set_control(__u32 id, int value)
    {

        struct v4l2_control control{};

        control.id = id;

        control.value = value;

        ioctl(fd_, VIDIOC_S_CTRL, &control);
    }



    // ===========================================================
    // Variáveis privadas.
    // ===========================================================

    // Descritor do dispositivo V4L2.
    int fd_ = -1;

    // Objeto do OpenCV responsável pela captura da câmera.
    cv::VideoCapture cap_;

    // Publisher dos frames.
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;

    // Subscriber dos parâmetros da câmera.
    rclcpp::Subscription<image_adjust_msgs::msg::ImageParams>::SharedPtr params_sub_;

    // Timer responsável pela captura periódica.
    rclcpp::TimerBase::SharedPtr timer_;
};