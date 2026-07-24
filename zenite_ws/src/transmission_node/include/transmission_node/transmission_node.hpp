// Biblioteca principal do ROS 2 (criação de nós, publishers, subscribers, logs, etc.)
#include <rclcpp/rclcpp.hpp>

// Mensagem utilizada para receber as velocidades das rodas
#include <geometry_msgs/msg/vector3.hpp>

// Biblioteca Boost.Asio para comunicação serial
#include <boost/asio.hpp>

// Classe responsável por controlar uma porta serial
#include <boost/asio/serial_port.hpp>

// Função utilizada para escrever dados na porta serial
#include <boost/asio/write.hpp>

// Biblioteca para utilizar std::clamp()
#include <algorithm>

// Facilita a escrita do namespace das configurações da porta serial
using boost::asio::serial_port_base;

// Classe principal do nó de transmissão
class TransmissionNode : public rclcpp::Node {
public:

  // ============================
  // Construtor do nó
  // ============================
  TransmissionNode()
  : Node("transmission_node"),   // Nome do nó no ROS2
    io_(),                       // Inicializa o serviço do Boost.Asio
    serial_(io_),                // Associa a porta serial ao io_service
    vmax_(40.0f)                 // Velocidade máxima considerada pelo sistema
  {
    try {

      // Abre a porta serial Bluetooth criada pelo rfcomm
      serial_.open("/dev/rfcomm0");

      // Código utilizado para criar essa porta:
      // sudo rfcomm bind /dev/rfcomm0 00:23:09:01:36:17

      // Configura a taxa de transmissão (baud rate)
      serial_.set_option(serial_port_base::baud_rate(9600));

      // Configura comunicação de 8 bits
      serial_.set_option(serial_port_base::character_size(8));

      // Sem bit de paridade
      serial_.set_option(serial_port_base::parity(serial_port_base::parity::none));

      // Um bit de parada
      serial_.set_option(serial_port_base::stop_bits(serial_port_base::stop_bits::one));

      // Informa que tudo ocorreu corretamente
      RCLCPP_INFO(this->get_logger(), "Porta serial /dev/rfcomm0 aberta com sucesso!");
    }

    // Caso ocorra algum erro ao abrir/configurar a porta
    catch (std::exception &e) {

      // Exibe erro fatal
      RCLCPP_FATAL(this->get_logger(),
                   "Falha ao abrir porta serial: %s",
                   e.what());

      // Encerra o ROS
      rclcpp::shutdown();
      return;
    }

    // Cria subscriber para receber velocidades das rodas
    subscriber_ =
      this->create_subscription<geometry_msgs::msg::Vector3>(
        "wheel_velocities",       // tópico publicado pelo Control Node
        10,                       // tamanho da fila
        std::bind(&TransmissionNode::vel_callback,
                  this,
                  std::placeholders::_1));

    // Cria uma thread separada para o Boost.Asio
    io_thread_ = std::thread([this]() {
      io_.run();
    });
  }

  // ============================
  // Destrutor
  // ============================
  ~TransmissionNode() {

    // Fecha a porta serial caso esteja aberta
    if (serial_.is_open())
      serial_.close();

    // Para o serviço do Boost
    io_.stop();

    // Aguarda a thread terminar
    if (io_thread_.joinable())
      io_thread_.join();
  }

private:

  // =====================================================
  // Callback executado sempre que chegam velocidades novas
  // =====================================================
  void vel_callback(const geometry_msgs::msg::Vector3::SharedPtr msg)
  {
    // Recebe velocidade da roda direita
    float v_dir = msg->x;

    // Recebe velocidade da roda esquerda
    float v_esq = msg->y;

    // Byte que armazenará a direção de cada motor
    uint8_t direcao = 0;

    // Bit 0 representa direção da roda direita
    if (v_dir >= 0)
      direcao |= 0b00000001;

    // Bit 1 representa direção da roda esquerda
    if (v_esq >= 0)
      direcao |= 0b00000010;

    // Limita velocidades dentro do intervalo permitido
    v_esq = std::clamp(v_esq, -vmax_, vmax_);
    v_dir = std::clamp(v_dir, -vmax_, vmax_);

    // Converte velocidade da roda esquerda em PWM (0-255)
    uint8_t pwm_esq =
      static_cast<uint8_t>(
        std::abs(v_esq) / vmax_ * 255.0f
      );

    // Converte velocidade da roda direita em PWM (0-255)
    uint8_t pwm_dir =
      static_cast<uint8_t>(
        std::abs(v_dir) / vmax_ * 255.0f
      );

    // Cria o pacote que será enviado ao Arduino
    //
    // Byte 0 -> direção
    // Byte 1 -> PWM esquerdo
    // Byte 2 -> PWM direito
    uint8_t pacote[3] = {
      direcao,
      pwm_esq,
      pwm_dir
    };

    try {

      // Envia os 3 bytes pela porta serial
      boost::asio::write(
        serial_,
        boost::asio::buffer(pacote, 3)
      );

      // Exibe o pacote enviado
      RCLCPP_INFO(
        this->get_logger(),
        "Enviado → DIR=%d | PWM_ESQ=%d | PWM_DIR=%d",
        direcao,
        pwm_esq,
        pwm_dir);
    }

    // Caso ocorra erro de transmissão
    catch (std::exception &e) {

      RCLCPP_WARN(
        this->get_logger(),
        "Erro ao enviar pacote serial: %s",
        e.what());
    }
  }

  // ============================
  // Objetos do Boost.Asio
  // ============================

  // Serviço principal do Boost.Asio
  boost::asio::io_service io_;

  // Porta serial
  boost::asio::serial_port serial_;

  // Thread dedicada ao Boost.Asio
  std::thread io_thread_;

  // ============================
  // Objetos do ROS2
  // ============================

  // Subscriber das velocidades das rodas
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr subscriber_;

  // Velocidade máxima utilizada para converter em PWM
  float vmax_;
};