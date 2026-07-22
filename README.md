# Zênite — Sistema Educacional de Robótica Espacial

O **Zênite** é um sistema robótico baseado em visão zenital desenvolvido para introduzir conceitos de robótica espacial a estudantes do ensino fundamental e médio por meio de atividades práticas fundamentadas na metodologia **STEAM**.

O projeto integra visão computacional, controle de trajetória, modelagem cinemática e arquitetura modular com ROS 2, aplicados em um contexto educacional inspirado em missões de exploração espacial.

---

## Contexto

O projeto foi desenvolvido no âmbito do Centro Vocacional Tecnológico Espacial (CVT-E), no IFRN – Campus Parnamirim.

O sistema foi apresentado no artigo:

> *Development of a Robotic System for the Introduction of Space Robotics Concepts in Basic Education*

O Zênite é simultaneamente uma plataforma funcional e uma ferramenta didática para ensino de robótica e conceitos de engenharia espacial.

---

## Objetivos

- Introduzir conceitos de robótica e astronáutica no ensino básico  
- Ensinar sistemas de coordenadas e controle de movimento  
- Demonstrar planejamento e execução de trajetórias  
- Aplicar visão computacional para localização de robôs  
- Simular missões inspiradas em rovers marcianos  

---

## Arquitetura do Sistema

O Zênite utiliza uma arquitetura modular baseada em ROS 2, composta pelos seguintes nós:

### 🔹 acquisition_node
O acquisition_node é o responsável por fazer a comunicação direta com a câmera do projeto. Ele abre o dispositivo de vídeo, captura as imagens continuamente e envia esses frames para o ROS 2, permitindo que outros nós usem a imagem em tempo real. Além disso, ele recebe ajustes como brilho, saturação e hue vindos do interface_node onde aplica essas configurações diretamente na câmera, garantindo que a imagem exibida esteja sempre de acordo com o que o usuário escolheu. Para isso, ele desativa o modo automático de exposição e passa a controlar tudo manualmente. O nó funciona em um ritmo de aproximadamente 30 quadros por segundo, convertendo cada captura para o formato de mensagem do ROS e publicando no tópico correspondente. Assim, ele serve como a ponte entre a câmera física e o restante do sistema robótico, fornecendo imagens atualizadas e configuráveis.

### 🔹 calibration_node
O calibration_node é o responsável por fazer a calibração da área de atuação do robô, permitindo que a câmera saiba converter pontos da imagem para coordenadas reais no chão, através do cálculo de homografia. Ele recebe continuamente os frames da câmera e exibe essas imagens em uma janela, permitindo que o operador selecione quatro pontos da cena, normalmente os cantos da área de operação do robô. A partir dessas coordenadas,  o nó registra as posições na imagem e as utiliza para realizar cálculos matriciais que permitem associar esses pontos a medidas reais do ambiente, formando um retângulo de dimensões conhecidas. Com esses pares de pontos, o nó calcula a transformação necessária para converter pixels em metros e salva essa configuração em um arquivo .yaml para ser usada pelos outros nós. Assim, o calibration_node estabelece a base para que o robô entenda onde está e possa navegar de maneira precisa.

### 🔹 localization_node
O localization_node recebe continuamente as imagens da câmera e rastreia a posição do robô em tempo real, a partir de um algoritmo feito em C++, utilizando o método de rastreamento CSRT do OpenCV. No início, o usuário seleciona o carrinho na tela por meio de uma seleção baseada em pixels, permitindo que a função de monitoramento acompanhe automaticamente o robô nos frames seguintes. O nó então desenha um retângulo ao redor do carrinho, registra sua trajetória e calcula o centro dessa região. Esse ponto é convertido de pixels para metros utilizando a transformação (homografia) obtida pelo calibration_node, gerando a posição real do robô no ambiente, a qual é enviada ao control_node. Assim, o localization_node combina rastreamento visual com conversão de coordenadas físicas, permitindo acompanhar o movimento do robô de forma simples e contínua.

### 🔹 interface_node
O interface_node funciona como a parte visual e interativa do sistema. Ele recebe em tempo real as imagens enviadas pela câmera e as exibe na tela usando o OpenCV. A partir dessa visualização, o usuário pode clicar em qualquer ponto da imagem, e o nó converte automaticamente esse pixel para coordenadas reais do ambiente usando uma transformação previamente calibrada pelo calibration_node; esse ponto convertido é então enviado ao control_node como a nova posição desejada para o robô ir. Além disso, o nó oferece controles deslizantes para ajustar brilho, saturação e hue da imagem. Sempre que os sliders mudam, ele envia esses novos valores para o acquisition_node, que ajusta esses parâmetros da câmera. Assim, o interface_node atua como a ponte entre o usuário e o robô, permitindo escolher destinos apenas com cliques e ajustar a imagem de forma intuitiva.

### 🔹 control_node
O control_node é responsável por calcular os comandos de movimento do rover. Ele recebe continuamente a posição atual do robô fornecida pelo localization_node e a posição desejada definida pelo interface_node. Quando um ponto alvo é fornecido, o nó calcula o movimento necessário para que o robô alcance essa posição, determinando tanto o ajuste de orientação quanto o deslocamento para frente.

O robô é modelado como um robô móvel diferencial não-holonômico, cuja postura no plano cartesiano pode ser descrita pelo vetor de estado:

q = [x, y, θ]ᵀ

onde x e y representam a posição do robô no plano e θ representa sua orientação em relação ao referencial global.


O modelo cinemático do robô pode ser descrito pelas seguintes equações:

ẋ = v cos(θ)
ẏ = v sin(θ)
θ̇ = ω

onde v representa a velocidade linear do robô e ω representa sua velocidade angular.


Dada uma posição desejada (x_ref, y_ref), a orientação de referência utilizada para guiar o robô até o alvo é calculada como:

θ_ref = arctan2(y_ref − y, x_ref − x)

O erro angular é então definido como

e_θ = θ_ref − θ


A distância entre o robô e o ponto alvo é dada por:

Δl = √((x_ref − x)² + (y_ref − y)²)

Seguindo a estratégia de controle de posicionamento, o erro linear pode ser definido como:

e_s = Δl cos(e_θ)

Esse valor representa a projeção da distância até o alvo na direção atual de movimento do robô.


O control_node utiliza controladores proporcional–integral para reduzir tanto o erro linear quanto o erro angular, gerando os comandos de velocidade linear v e velocidade angular ω. Esses comandos são atualizados continuamente durante o ciclo de controle, permitindo que o robô corrija sua trajetória enquanto se aproxima do alvo.

Por fim, as velocidades linear e angular são convertidas nas velocidades individuais das rodas direita e esquerda do robô, de acordo com o modelo cinemático do sistema diferencial:

v_r = (2v + ωL) / (2R)

v_l = (2v − ωL) / (2R)

onde L representa a distância entre as rodas e R é o raio das rodas.

As velocidades calculadas são então enviadas ao transmission_node, que as converte em sinais de controle para os motores do robô. Esse processo ocorre continuamente, com o ciclo de controle sendo executado várias vezes por segundo até que o robô atinja o objetivo dentro de uma tolerância de erro aceitável.


### 🔹 transmission_node
O transmission_node é o responsável por enviar para o microcontrolador os comandos que o robô precisa para se mover. Ele abre uma comunicação Bluetooth, configura a porta serial e fica aguardando as velocidades calculadas pelo control_node. Quando recebe esses valores, ele converte a velocidade de cada roda para um valor de PWM entre 0 e 255 e também determina a direção de cada motor, usando bits para indicar se o movimento é para frente ou para trás. Em seguida, ele envia um pacote de três bytes contendo direção e potência dos dois motores para o microcontrolador. Assim, esse nó funciona como a ponte entre o ROS 2 e a parte física do robô, garantindo que as velocidades calculadas sejam transformadas em sinais reais de movimento.

---

## Robô

O robô possui uma configuração não holonômica, cinematicamente equivalente a uma plataforma de movimento diferencial, na qual as rodas de cada lado se movem juntas. Ele recebe continuamente, via módulo Bluetooth HC-06, pacotes contendo a direção e a intensidade de velocidade para cada motor. O microcontrolador interpreta esses três bytes, identifica o sentido correto de rotação e aplica sinais PWM nos dois canais da ponte H L298N, acionando cada motor de forma independente e viabilizando movimentos para frente, ré e curvas. Todo o processo ocorre em tempo real, com leitura constante dos pacotes e aplicação imediata das velocidades, o que torna o deslocamento estável e responsivo aos comandos enviados pelo sistema ROS 2.

### Componentes atuais:

- Arduino  
- Ponte H L298N  
- Módulo Bluetooth HC-06
- 4 motores CC (Rodas)

---

# Como Executar o Projeto

## Instalar o ROS 2 Jazzy

Siga as instruções oficiais de instalação para Ubuntu (Tenha a certeza de instalar os pacotes de desenvolvedor):

https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html

---

## Configurar o Ambiente ROS 2

Após instalar, execute:

```bash
source /opt/ros/jazzy/setup.bash
```

Para carregar automaticamente no terminal:

```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

Clone o Repositório:
```bash
git clone <URL_DO_REPOSITORIO>
cd zenite_ws
colcon build
```

Após compilar:

```bash
source install/setup.bash
```

## Configurar o Blueooth

Para associar a porta rfcomm ao endereço mac do robô (Apenas se o robô ainda for o mesmo, caso não, altere esse passo):
```bash
sudo rfcomm bind /dev/rfcomm0 00:23:09:01:36:17
```

## Executando os Nós

Abra terminais separados para cada nó:

acquisition_node:
```bash
ros2 run acquisition_node acquisition_node
```

calibration_node:
```bash
ros2 run calibration_node calibration_node
```

localization_node:
```bash
ros2 run localization_node localization_node
```

interface_node:
```bash
ros2 run interface_node interface_node
```

control_node:
```bash
ros2 run control_node control_node
```

transmission_node:
```bash
ros2 run transmission_node transmission_node
```


Ou utilize os Launchers:

Primeiro instale o xterm:
```bash
sudo apt install xterm
```

Depois rode os launchers: 

Para calibração:
```bash
ros2 launch zenite_launch zenite_calibration.py
```

Para execução completa:
```bash
ros2 launch zenite_launch zenite_launch.py
```
