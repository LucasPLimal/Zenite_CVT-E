# web_interface

Interface web multiusuário do Zênite: backend FastAPI (HTTP + WebSocket) + nó ponte ROS 2 (rclpy),
como alternativa ao `interface_node` (OpenCV local).

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
- **Escolha do robô** — ao abrir a página o usuário escolhe um robô livre; a telemetria e o
  alvo passam a valer só para ele. Veja "Multiusuário" abaixo.
- **Clique para mover** — clique na imagem envia o alvo do robô escolhido
  (`/robot_<id>/desired_position`) via homografia.
- **Ajustes de câmera** — sliders de brilho/saturação/hue (`image_params`), globais.
- **Calibração** — botão "Calibrar pontos" replica o `calibration_node` (C++): 4 cliques nos
  cantos da arena (2,75 x 2,25 m) calculam e salvam a homografia em `scale.yaml`. Global e
  livre para todos os usuários.
- **Posição do robô** — vem do `localization_node` (marcadores ArUco), em
  `/robot_<id>/current_pose` (`[x, y, theta]`).
- **Conectar um robô** — painel na tela de escolha para procurar e conectar robôs novos por
  Bluetooth, sem terminal (usa o `bluetooth_node`).

## Multiusuário

- O navegador gera um token (guardado no `localStorage`) e o envia ao abrir o WebSocket.
- Cada robô tem no máximo um dono. Se a conexão do dono cair, o robô **para na hora** e fica
  reservado ao mesmo token por 30 s (parâmetro `reservation_grace_s`); depois é liberado.
- "Tornar todos os robôs livres" libera e para todos (fase de testes, qualquer usuário).
- A lista de robôs vem do `bluetooth_node` (`/bluetooth/state`). **Sem ele rodando**, a
  interface funciona no modo antigo: um robô só (ID 0), sem tela de escolha.

Mensagens do WebSocket (`/ws`):

| Sentido | Mensagem |
|---|---|
| cliente → servidor | `hello {token}`, `select_robot {id}`, `release_robot`, `release_all` |
| cliente → servidor | `bt_scan`, `bt_connect {mac}`, `goal {x, y}`, `params`, `calibrate`, `reload_homography` |
| servidor → cliente | `robots`, `bluetooth`, `bt_event`, `pose`, `status`, `goal_ack`, `calibrate_ack`, `error` |

Testes:

```bash
python3 -m pytest src/web_interface/test/          # reserva, sem ROS
ROS_DOMAIN_ID=79 python3 src/web_interface/test/integracao_multiusuario.py   # com ROS
```

> O rastreamento manual por CSRT ("Localizar robô") foi removido da interface, pois
> publicaria a posição em conflito com o `localization_node`. Ele continua no histórico do
> Git (commits `6ba8132` e `be8f3fd`) caso precise ser recuperado.
