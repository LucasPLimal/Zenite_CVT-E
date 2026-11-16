# web_interface

Interface web do Zênite: backend FastAPI (HTTP + WebSocket) + nó ponte ROS 2 (rclpy), como
alternativa ao `interface_node` (OpenCV local) e, opcionalmente, ao `localization_node`.

## Instalar dependências

```bash
sudo apt install python3-fastapi python3-uvicorn python3-jinja2 python3-numpy python3-yaml python3-opencv
```

O OpenCV é usado só para codificar os quadros em JPEG e para calcular a homografia da
calibração; não é necessário o módulo `opencv_contrib`.

## Rodar

```bash
ros2 run web_interface web_interface
```

Parâmetros opcionais:
```bash
ros2 run web_interface web_interface --ros-args -p port:=8080 -p scale_yaml_path:=/tmp/scale.yaml
```

Acesse `http://localhost:8000` (ou a porta configurada) no navegador.

## Funcionalidades

- **Vídeo ao vivo** — stream do `camera_frame` via WebSocket (JPEG).
- **Clique para mover** — clique na imagem envia o alvo (`/desired_position`) via homografia.
- **Ajustes de câmera** — sliders de brilho/saturação/hue (`image_params`).
- **Calibração** — botão "Calibrar ponto" replica o `calibration_node` (C++): 4 cliques nos
  cantos da arena (2,75 x 2,25 m) calculam e salvam a homografia em `scale.yaml`.
- **Posição do robô** — vem do `localization_node` (marcadores ArUco), que publica a pose em
  `/robot_<id>/current_pose`. A migração da interface para esse tópico ainda está pendente.

> O rastreamento manual por CSRT ("Localizar robô") foi removido da interface, pois
> publicaria a posição em conflito com o `localization_node`. Ele continua no histórico do
> Git (commits `6ba8132` e `be8f3fd`) caso precise ser recuperado.
