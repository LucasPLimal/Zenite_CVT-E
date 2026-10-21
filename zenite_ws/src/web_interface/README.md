# web_interface

Interface web do Zênite: backend FastAPI (HTTP + WebSocket) + nó ponte ROS 2 (rclpy), como
alternativa ao `interface_node` (OpenCV local) e, opcionalmente, ao `localization_node`.

## Instalar dependências

```bash
sudo apt install python3-fastapi python3-uvicorn python3-jinja2 python3-numpy python3-yaml
pip install opencv-contrib-python-headless
```

**Importante — sobre o OpenCV:** este pacote precisa de `opencv-contrib-python` (não
`opencv-python` nem o pacote apt `python3-opencv`), porque a função "Localizar robô" usa
`cv2.TrackerCSRT`, que só existe no módulo `tracking` do `opencv_contrib`. Use a variante
**`-headless`**: o `web_interface_node` nunca abre janela local (diferente do
`calibration_node`/`localization_node` em C++, que usam `cv::imshow`), então não precisa da
GUI — a headless é mais leve e evita dependências de GTK/Qt, o que ajuda bastante num
Raspberry Pi sem ambiente gráfico.

Os pacotes `opencv-python`, `opencv-contrib-python`, `opencv-python-headless` e
`opencv-contrib-python-headless` **não podem coexistir** no mesmo ambiente: todos se
instalam sob o mesmo nome de import (`cv2`) e escrevem nos mesmos arquivos dentro de
`site-packages/cv2/` — instalar mais de um resulta num ambiente inconsistente (o último
instalado sobrescreve o anterior, e o resultado depende da ordem de instalação). Se o
ambiente já tiver `python3-opencv` (apt) ou alguma outra variante (pip) instalada, remova
antes:

```bash
sudo apt remove python3-opencv   # se instalado via apt
pip uninstall opencv-python opencv-python-headless opencv-contrib-python   # se instalado via pip
pip install opencv-contrib-python-headless
```

Em Raspberry Pi, verifique se existe wheel pré-compilada para `opencv-contrib-python` na
combinação de arquitetura/versão de Python usada (ex.: via piwheels.org) antes de instalar —
sem wheel pronta, o pip tenta compilar do zero, o que pode levar bastante tempo.

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
- **Localizar robô** — botão que replica o `localization_node` (C++): seleção do robô por
  marquee (arraste) inicializa um `cv2.TrackerCSRT`, que passa a publicar `/current_position`
  a cada frame (mesmo cooldown de 50ms do nó original). **Atenção**: se o `localization_node`
  também estiver rodando, os dois vão publicar no mesmo tópico ao mesmo tempo — não rode os
  dois juntos.
