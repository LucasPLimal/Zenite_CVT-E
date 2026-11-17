// Bibliotecas padrão do C++ usadas para tempo, funções matemáticas,
// ponteiros inteligentes, entrada/saída, containers e algoritmos.
#include <chrono>
#include <cmath>
#include <memory>
#include <iostream>
#include <string>
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

// Mensagem ROS utilizada para receber a pose atual do robô como um vetor de
// números de ponto flutuante: [x, y, theta] (x e y em metros, theta em radianos),
// publicada pelo LocalizationNode em /robot_<id>/current_pose.
#include <std_msgs/msg/float64_multi_array.hpp>

// Alteração feita por Danilo: mensagem Bool usada no tópico /robot_<id>/stop,
// que manda o robô parar (ex.: usuário desconectou da interface web).
#include <std_msgs/msg/bool.hpp>

// Permite usar literais de tempo como "10ms", "1s", etc.
using namespace std::chrono_literals;

// Classe responsável pelo controle de posição do robô.
// Recebe a pose atual e a posição desejada, calcula o erro
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
    L(0.15f),                // Distância entre rodas (ajustar conforme robô real)
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

    // Parâmetro com o ID do robô controlado por este nó (o mesmo ID do
    // marcador ArUco). Ele define de qual tópico a pose é lida.
    const int robot_id = this->declare_parameter<int>("robot_id", 0);
    const std::string robot_prefix = "/robot_" + std::to_string(robot_id);
    const std::string pose_topic = robot_prefix + "/current_pose";

    // Alteração feita por Danilo: os dois ajustes de teste que estavam fixos
    // no código (inverter o sentido da curva e a velocidade mínima das rodas)
    // viraram parâmetros, mantendo os mesmos valores padrão de antes. Assim
    // cada robô pode ser ajustado pelo launch sem recompilar.
    invert_turn_ = this->declare_parameter<bool>("invert_turn", true);
    min_wheel_speed_ = static_cast<float>(
      this->declare_parameter<double>("min_wheel_speed", 12.0));

    // Cria um publisher para enviar as velocidades calculadas
    // das rodas (direita/esquerda) para o restante do sistema.
    //
    // Alteração feita por Danilo: o tópico agora é por robô
    // (/robot_<id>/wheel_velocities), para vários robôs poderem rodar juntos.
    publisher_ = this->create_publisher<geometry_msgs::msg::Vector3>(
      robot_prefix + "/wheel_velocities", 10);

    // Cria um subscriber para receber a pose atual do robô
    // ([x, y, theta]), enviada pelo LocalizationNode.
    current_pos_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      pose_topic, 10,
      std::bind(&ControlNode::currentPosCallback, this, std::placeholders::_1));



    // Cria um subscriber para receber a posição desejada (alvo)
    // que o robô deve alcançar.
    //
    // Alteração feita por Danilo: o alvo agora é por robô
    // (/robot_<id>/desired_position) em vez do tópico global /desired_position,
    // para que o clique de um usuário não mova os robôs dos outros.
    desired_pos_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
      robot_prefix + "/desired_position", 10,
      std::bind(&ControlNode::desiredPosCallback, this, std::placeholders::_1));

    // Alteração feita por Danilo: tópico de parada do robô. Qualquer mensagem
    // recebida em /robot_<id>/stop cancela o alvo atual e para o robô.
    stop_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      robot_prefix + "/stop", 10,
      std::bind(&ControlNode::stopCallback, this, std::placeholders::_1));



    // Cria um timer que executa o loop de controle periodicamente,
    // de acordo com o período "dt" definido acima (10ms).
    timer_ = this->create_wall_timer(
      std::chrono::duration<float>(dt),
      std::bind(&ControlNode::controlLoop, this));



    // Apenas informa no terminal que o nó iniciou
    // e está aguardando os dados necessários.
    RCLCPP_INFO(this->get_logger(), "ControlNode iniciado");
    RCLCPP_INFO(this->get_logger(), "Lendo a pose do robo %d em %s",
                robot_id, pose_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "Aguardando posição atual e posição desejada...");
  }

private:

  // ======================================================
  // CALLBACK DA POSE ATUAL
  // Executado sempre que uma nova pose [x, y, theta]
  // (vinda do LocalizationNode) chega.
  // ======================================================
  void currentPosCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    // A pose publicada pelo LocalizationNode tem sempre 3 valores:
    // [x, y, theta]. Mensagens com outro tamanho são ignoradas.
    if (msg->data.size() < 3) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Pose invalida: esperado [x, y, theta], recebidos %zu valores",
        msg->data.size());
      return;
    }

    // Atualiza o estado interno do robô com a pose recebida
    // (x e y em metros, theta em radianos).
    x = static_cast<float>(msg->data[0]);
    y = static_cast<float>(msg->data[1]);
    theta = static_cast<float>(msg->data[2]);
    has_current_pos = true;



    // Log de depuração com a posição e orientação atuais (em graus).
    RCLCPP_DEBUG(this->get_logger(), "Pos atual: (%.3f, %.3f), θ=%.2f°",
                 x, y, theta * 180.0f / M_PI);
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
  // CALLBACK DE PARADA
  // Alteração feita por Danilo: executado quando chega uma mensagem em
  // /robot_<id>/stop. Cancela o alvo, zera os integradores do controle e
  // publica velocidade zero, para o robô não continuar indo até o alvo
  // antigo (ex.: usuário desconectou) nem herdar esse alvo no próximo uso.
  // ======================================================
  void stopCallback(const std_msgs::msg::Bool::SharedPtr /*msg*/)
  {
    is_moving = false;
    has_desired_pos = false;
    i_linear = 0.0f;
    i_angular = 0.0f;

    stopRobot();
    RCLCPP_INFO(this->get_logger(), "Parada solicitada: alvo cancelado");
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
    if (distance < 0.05f) { // 5 cm de tolerância
      stopRobot();
      RCLCPP_INFO(this->get_logger(), "Alvo alcançado!");
      is_moving = false;

      // Sai do loop: sem isso, o código abaixo calcularia e publicaria
      // novas velocidades logo depois do comando de parada.
      return;
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
    // Alteração feita por Danilo: a inversão (antes fixa, marcada como TESTE)
    // agora depende do parâmetro invert_turn (padrão true, igual ao antes).
    float omega = invert_turn_ ? -u_angular : u_angular;

    // Converte as velocidades linear e angular do robô em
    // velocidades individuais das rodas (direita e esquerda),
    // considerando a distância entre rodas (L) e o raio da roda (R).
    float v_direito = (2.0f * v + omega * L) / (2.0f * R);
    float v_esquerdo = (2.0f * v - omega * L) / (2.0f * R);



    // ------------------------------------------------------
    // TESTE (REMOVER QUANDO TROCAR AS PILHAS):
    // velocidade mínima (em módulo) das rodas.
    // Escala as duas rodas pelo mesmo fator, então a proporção
    // entre elas (e portanto o direcionamento) continua igual.
    //
    // Alteração feita por Danilo: o valor (antes fixo em 0xC = 12) agora vem
    // do parâmetro min_wheel_speed (padrão 12.0). Use 0 para desligar.
    // ------------------------------------------------------
    const float MIN_WHEEL_SPEED = min_wheel_speed_;
    float max_abs = std::max(std::fabs(v_direito), std::fabs(v_esquerdo));
    if (MIN_WHEEL_SPEED > 0.0f && max_abs > 0.0f && max_abs < MIN_WHEEL_SPEED) {
      float scale = MIN_WHEEL_SPEED / max_abs;
      v_direito  *= scale;
      v_esquerdo *= scale;
    }



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
  float x_ref = 0.0f, y_ref = 0.0f;         // Posição desejada



  // Estados internos do controlador PID.
  float e_linear_ant = 0.0f, e_angular_ant = 0.0f;  // Últimos erros calculados
  float i_linear = 0.0f, i_angular = 0.0f;          // Valores acumulados dos integradores



  // Alteração feita por Danilo: ajustes lidos de parâmetros (ver construtor).
  bool invert_turn_ = true;          // Inverte o sentido da curva
  float min_wheel_speed_ = 12.0f;    // Velocidade mínima das rodas (0 = desligado)



  // Flags de controle de fluxo do nó.
  bool has_current_pos;   // Indica se já foi recebida a posição atual
  bool has_desired_pos;   // Indica se já foi recebida a posição desejada
  bool is_moving;         // Indica se o robô ainda está se movendo em direção ao alvo



  // Interfaces ROS2: timer, publisher e subscribers.
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr current_pos_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr desired_pos_sub_;

  // Alteração feita por Danilo: subscriber do tópico de parada.
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr stop_sub_;
};