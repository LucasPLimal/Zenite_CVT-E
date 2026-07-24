// Bibliotecas padrão do C++ usadas para tempo, funções matemáticas,
// ponteiros inteligentes, entrada/saída, containers e algoritmos.
#include <chrono>
#include <cmath>
#include <memory>
#include <iostream>
#include <vector>
#include <fstream>
#include <algorithm>

// Biblioteca principal do ROS 2.
// Fornece a classe Node, publishers, subscribers, timers, logs, etc.
#include <rclcpp/rclcpp.hpp>

// Mensagem ROS utilizada para publicar as velocidades das rodas (x, y, z).
#include <geometry_msgs/msg/vector3.hpp>

// Mensagem ROS utilizada para receber a posição desejada (ponto x, y, z).
#include <geometry_msgs/msg/point.hpp>

// Mensagem ROS utilizada para receber a posição atual como um vetor de
// números de ponto flutuante (ex: [x, y] ou [x, y, theta]).
#include <std_msgs/msg/float64_multi_array.hpp>

// Permite usar literais de tempo como "10ms", "1s", etc.
using namespace std::chrono_literals;

// Classe responsável pelo controle de posição do robô.
// Recebe a posição atual e a posição desejada, calcula o erro
// através de um controlador PI (linear e angular) e converte
// o resultado em velocidades para as rodas (cinemática diferencial).
// Herda de rclcpp::Node para se tornar um nó ROS2.
class ControlNode : public rclcpp::Node
{
public:

  // ==========================
  // CONSTRUTOR
  // ==========================
  ControlNode()
  : Node("control_node"),   // Nome do nó no ROS2.

    // ------------------------------------------------
    // Lista de inicialização: define os parâmetros
    // físicos do robô e os ganhos do controlador PI.
    // ------------------------------------------------
    L(0.16f),                // Distância entre rodas (ajustar conforme robô real)
    R(0.03f),                // Raio da roda (ajustar conforme robô real)
    dt(0.01f),               // Período de controle (10ms)
    kc_angular(5.0f),        // Ganho proporcional angular
    Ti_angular(1000.0f),     // Tempo integral angular
    kc_linear(1.0f),         // Ganho proporcional linear
    Ti_linear(10000.0f),     // Tempo integral linear
    integral_limit_linear(5.0f),   // Limite de saturação do integrador linear
    integral_limit_angular(5.0f),  // Limite de saturação do integrador angular

    // Estado inicial do robô: parado na origem, sem rotação.
    x(0.0f), y(0.0f), theta(0.0f),

    // Flags de controle: nenhuma posição foi recebida ainda
    // e o robô não está em movimento.
    has_current_pos(false), has_desired_pos(false),
    is_moving(false)
  {

    // Cria um publisher para enviar as velocidades calculadas
    // das rodas (direita/esquerda) para o restante do sistema.
    publisher_ = this->create_publisher<geometry_msgs::msg::Vector3>("wheel_velocities", 10);



    // Cria um subscriber para receber a posição atual do robô,
    // normalmente enviada por um sistema de visão ou sensores externos.
    current_pos_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      "/current_position", 10,
      std::bind(&ControlNode::currentPosCallback, this, std::placeholders::_1));



    // Cria um subscriber para receber a posição desejada (alvo)
    // que o robô deve alcançar.
    desired_pos_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
      "/desired_position", 10,
      std::bind(&ControlNode::desiredPosCallback, this, std::placeholders::_1));



    // Cria um timer que executa o loop de controle periodicamente,
    // de acordo com o período "dt" definido acima (10ms).
    timer_ = this->create_wall_timer(
      std::chrono::duration<float>(dt),
      std::bind(&ControlNode::controlLoop, this));



    // Apenas informa no terminal que o nó iniciou
    // e está aguardando os dados necessários.
    RCLCPP_INFO(this->get_logger(), "ControlNode iniciado");
    RCLCPP_INFO(this->get_logger(), "Aguardando posição atual e posição desejada...");
  }

private:

  // ======================================================
  // CALLBACK DA POSIÇÃO ATUAL
  // Executado sempre que uma nova leitura de posição
  // (vinda de sensores/visão) chega.
  // ======================================================
  void currentPosCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    // Só processa se a mensagem tiver pelo menos x e y.
    if (msg->data.size() >= 2) {

      // Espera-se: [x, y, theta] ou apenas [x, y].
      float new_x = static_cast<float>(msg->data[0]);
      float new_y = static_cast<float>(msg->data[1]);
      float new_theta = theta; // mantém o valor atual caso não seja fornecido



      if (msg->data.size() >= 3) {
        // Se o theta for fornecido diretamente na mensagem, usa esse valor.
        new_theta = static_cast<float>(msg->data[2]);
      } else if (has_current_pos) {
        // Caso contrário, estima o ângulo de orientação (theta)
        // a partir do deslocamento entre a posição anterior e a nova.
        float dx = new_x - x;
        float dy = new_y - y;
        float dist = std::hypot(dx, dy);

        // Só recalcula theta se o deslocamento for significativo,
        // evitando ruído quando o robô está praticamente parado.
        if (dist > 0.001f) { // Movimento significativo
          new_theta = std::atan2(dy, dx);
        }
      }



      // Atualiza o estado interno do robô com os novos valores.
      x = new_x;
      y = new_y;
      theta = new_theta;
      has_current_pos = true;



      // Log de depuração com a posição e orientação atuais (em graus).
      RCLCPP_DEBUG(this->get_logger(), "Pos atual: (%.3f, %.3f), θ=%.2f°", 
                   x, y, theta * 180.0f / M_PI);
    }
  }



  // ======================================================
  // CALLBACK DA POSIÇÃO DESEJADA
  // Executado sempre que uma nova posição-alvo é recebida.
  // ======================================================
  void desiredPosCallback(const geometry_msgs::msg::Point::SharedPtr msg)
  {
    // Armazena a nova referência de posição (alvo).
    x_ref = msg->x;
    y_ref = msg->y;
    has_desired_pos = true;

    // Reseta os integradores do PID sempre que a referência muda,
    // evitando que o erro acumulado de um alvo anterior
    // influencie o novo trajeto.
    i_linear = 0.0f;
    i_angular = 0.0f;

    // Marca o robô como "em movimento" em direção ao novo alvo.
    is_moving = true;

    RCLCPP_INFO(this->get_logger(), "Nova referência: (%.3f, %.3f)", x_ref, y_ref);
  }



  // ======================================================
  // LOOP PRINCIPAL DE CONTROLE
  // Executado periodicamente pelo timer (a cada "dt" segundos).
  // Calcula o erro em relação ao alvo e gera os comandos
  // de velocidade para as rodas.
  // ======================================================
  void controlLoop()
  {
    // Não faz nada se ainda não houver posição atual, posição
    // desejada, ou se o robô já tiver atingido o alvo (parado).
    if (!has_current_pos || !has_desired_pos || !is_moving) {
      return;
    }



    // Calcula a distância euclidiana até o alvo.
    float delta_x = x_ref - x;
    float delta_y = y_ref - y;
    float distance = std::hypot(delta_x, delta_y);



    // Critério de parada: se estiver perto o suficiente do alvo,
    // para o robô e encerra o movimento.
    if (distance < 0.10f) { // 5 cm de tolerância
      stopRobot();
      RCLCPP_INFO(this->get_logger(), "Alvo alcançado!");
      is_moving = false;
    }



    // Calcula o ângulo (em radianos) da direção que aponta
    // da posição atual até o alvo.
    float theta_ref = std::atan2(delta_y, delta_x);

    // Calcula o erro angular entre a orientação atual do robô
    // e a orientação desejada (em direção ao alvo).
    float error_angular = theta_ref - theta;

    // Normaliza o erro angular para o intervalo [-π, π],
    // evitando que o robô gire pelo caminho mais longo.
    while (error_angular > M_PI) error_angular -= 2.0f * M_PI;
    while (error_angular < -M_PI) error_angular += 2.0f * M_PI;



    // Limita o erro angular para ±90° (evita manobras bruscas
    // quando o robô está muito desalinhado com o alvo).
    const float MAX_ANGULAR_ERROR = M_PI_2;
    error_angular = std::clamp(error_angular, -MAX_ANGULAR_ERROR, MAX_ANGULAR_ERROR);



    // Erro linear: projeta a distância até o alvo na direção
    // atual do robô (cos do erro angular). Isso faz o robô
    // andar menos para frente quando ainda está girando bastante.
    float error_linear = distance * std::cos(error_angular);



    // **CONTROLE PID - Linear**

    // Termo proporcional: reage ao erro linear atual.
    float p_linear = kc_linear * error_linear;

    // Termo integral (Ki = Kc/Ti): acumula o erro ao longo do tempo
    // para eliminar erro residual em regime permanente.
    float Ki_linear = (Ti_linear > 0.0f) ? (kc_linear / Ti_linear) : 0.0f;
    i_linear += Ki_linear * error_linear * dt;

    // Satura o integrador para evitar o efeito "windup"
    // (acúmulo excessivo do termo integral).
    i_linear = std::clamp(i_linear, -integral_limit_linear, integral_limit_linear);

    // Sinal de controle linear final (soma dos termos P e I).
    float u_linear = p_linear + i_linear;

    // Guarda o erro atual para uso em um possível cálculo futuro
    // (ex: termo derivativo, caso seja adicionado).
    e_linear_ant = error_linear;



    // **CONTROLE PID - Angular**

    // Termo proporcional: reage ao erro angular atual.
    float p_angular = kc_angular * error_angular;

    // Termo integral do controlador angular.
    float Ki_angular = (Ti_angular > 0.0f) ? (kc_angular / Ti_angular) : 0.0f;
    i_angular += Ki_angular * error_angular * dt;

    // Satura o integrador angular, mesma lógica do linear.
    i_angular = std::clamp(i_angular, -integral_limit_angular, integral_limit_angular);

    // Sinal de controle angular final (soma dos termos P e I).
    float u_angular = p_angular + i_angular;

    // Guarda o erro atual para uso futuro.
    e_angular_ant = error_angular;



    // **CINEMÁTICA DIFERENCIAL**
    // v = velocidade linear, ω = velocidade angular
    float v = u_linear;
    float omega = u_angular;

    // Converte as velocidades linear e angular do robô em
    // velocidades individuais das rodas (direita e esquerda),
    // considerando a distância entre rodas (L) e o raio da roda (R).
    float v_direito = (2.0f * v + omega * L) / (2.0f * R);
    float v_esquerdo = (2.0f * v - omega * L) / (2.0f * R);



    // Publica as velocidades calculadas para as rodas.
    publishWheelVelocities(v_direito, v_esquerdo);



    // Log detalhado do estado atual do controlador,
    // útil para depuração e ajuste dos ganhos.
    RCLCPP_INFO(this->get_logger(), 
      "Pos: (%.3f, %.3f) | Ref: (%.3f, %.3f) | Dist: %.3f | "
      "θ: %.1f° | θ_ref: %.1f° | Vd: %.2f | Ve: %.2f",
      x, y, x_ref, y_ref, distance,
      theta * 180.0f / M_PI, theta_ref * 180.0f / M_PI,
      v_direito, v_esquerdo);
  }



  // ======================================================
  // PUBLICA AS VELOCIDADES DAS RODAS
  // Monta a mensagem Vector3 e publica no tópico correspondente.
  // ======================================================
  void publishWheelVelocities(float v_right, float v_left)
  {
    // Cria a mensagem que será publicada.
    auto msg = geometry_msgs::msg::Vector3();

    // Usa os campos x e y para representar as rodas direita e esquerda.
    msg.x = v_right;
    msg.y = v_left;

    // Campo z não utilizado no momento, mas reservado
    // para uma possível flag ou informação extra no futuro.
    msg.z = 0.0; // pode ser usado para flags se necessário

    // Publica a mensagem no tópico "wheel_velocities".
    publisher_->publish(msg);
  }



  // ======================================================
  // PARA O ROBÔ
  // Publica velocidade zero para as duas rodas.
  // ======================================================
  void stopRobot()
  {
    publishWheelVelocities(0.0f, 0.0f);
    RCLCPP_INFO(this->get_logger(), "Robô parado");
  }



  // ======================================================
  // VARIÁVEIS DA CLASSE
  // ======================================================

  // Parâmetros físicos do robô (constantes, definidas no construtor).
  const float L;      // Distância entre rodas (m)
  const float R;      // Raio da roda (m)
  const float dt;     // Período de controle (s)



  // Ganhos do controlador PI (constantes, ajustáveis por sintonia).
  const float kc_angular, Ti_angular;   // Ganho proporcional e tempo integral angular
  const float kc_linear, Ti_linear;     // Ganho proporcional e tempo integral linear
  const float integral_limit_linear, integral_limit_angular; // Limites anti-windup



  // Estado atual do robô (posição e orientação).
  float x, y, theta;          // Posição e orientação atuais



  // Posição desejada (alvo) recebida via subscriber.
  float x_ref, y_ref;         // Posição desejada



  // Estados internos do controlador PID.
  float e_linear_ant, e_angular_ant;  // Últimos erros calculados
  float i_linear, i_angular;          // Valores acumulados dos integradores



  // Flags de controle de fluxo do nó.
  bool has_current_pos;   // Indica se já foi recebida a posição atual
  bool has_desired_pos;   // Indica se já foi recebida a posição desejada
  bool is_moving;         // Indica se o robô ainda está se movendo em direção ao alvo



  // Interfaces ROS2: timer, publisher e subscribers.
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr current_pos_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr desired_pos_sub_;
};