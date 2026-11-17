# Changelog — web_interface

Formato baseado em [Keep a Changelog](https://keepachangelog.com/pt-BR/1.1.0/).
Commits seguem [Conventional Commits](https://www.conventionalcommits.org/pt-br/v1.0.0/)
(`feat`, `fix`, `style`, `chore`, `docs`) com escopo `web_interface`.

## [Não lançado] — branch `integration/web-interface`

Integração do front-end (Gustavo Andrey, `feature/componentes-css`) com o
back-end (`feature/calibration_node`), sobre `origin/main`.

### Adicionado
- Front ligado ao WebSocket: vídeo, badges, console, clique para alvo,
  sliders de brilho/saturação/matiz, calibração por 4 cliques, toggles de
  visualização e telemetria (atual com ângulo, inicial, alvo, último alvo e
  área de calibragem).
- Botão "Recarregar homografia" na seção de calibração.

- Tela de escolha de robô (multiusuário): cartões por robô com status,
  botão Conectar, barra "Você está controlando", Trocar de robô e
  "Tornar todos os robôs livres" (qualquer usuário, fase de testes).
  Só aparece quando o `bluetooth_node` está rodando; sem ele a interface
  segue no modo antigo, com um robô só (ID 0).
- **Multiusuário no servidor:** reserva de robôs por token do navegador
  (`reservations.py`). O robô escolhido fica do usuário enquanto houver
  conexão e por 30 s depois da queda (parado na hora, por segurança); se o
  mesmo navegador volta nesse prazo, retoma o robô. Troca de robô, liberação
  por perda de Bluetooth e "Tornar todos livres" também mandam parar o robô.
- Tópicos por robô: `/robot_<id>/desired_position`, `/robot_<id>/stop` e
  `/robot_<id>/current_pose` (a pose e o ângulo vêm do `localization_node`
  por ArUco e só chegam ao dono do robô).
- Painel "Conectar um robô" na tela de escolha: procura dispositivos
  Bluetooth, conecta e mostra os avisos do `bluetooth_node` (robô conectado,
  dispositivo que não é robô, robô fora do cadastro).
- Teste integrado com ROS (`test/integracao_multiusuario.py`) e testes da
  reserva (`test/test_reservations.py`).
- Integração da branch `dev` do Gustavo (animações das abas).
- `acquisition_node`: parâmetro `video_index` para escolher a câmera.

### Em outros pacotes (multiusuário)
Alterações em nós de outras equipes levam o comentário "Alteração feita por
Danilo" no código.
- `bluetooth_node` (novo): procura, pareia e conecta os ESP32 por RFCOMM sem
  `rfcomm bind`, confirma o robô pela apresentação `ZENIT,<nome>` e o
  cadastro `robots.yaml` (até 8 robôs), reconecta robôs conhecidos sozinho e
  envia as velocidades das rodas.
- `control_node`: uma instância por robô (`robot_id`), tópicos
  `/robot_<id>/desired_position` e `/robot_<id>/wheel_velocities`, novo
  `/robot_<id>/stop`, e os ajustes de teste viraram os parâmetros
  `invert_turn` e `min_wheel_speed` (mesmos valores padrão).
- `transmission_node`: parâmetros `robot_id` e `serial_port` (uso antigo com
  `rfcomm`).
- `FirmwareEsp32.ino`: `ROBOT_NAME` por robô e envio de `ZENIT,<nome>` ao
  conectar.
- `zenite_launch`: novo `zenite_multiusuario_launch.py`.
- `tools/bluetooth_poc.py`: prova de conceito do Bluetooth no Raspberry Pi.

### Alterado
- O `/current_position` (obsoleto) deixou de ser usado: a interface lê
  `/robot_<id>/current_pose`.
- O alvo deixou de ser global (`/desired_position`): o `interface_node`
  antigo não move mais os robôs, a menos que seu tópico seja remapeado.
- `.gitignore`: ignora `venv/`, `__pycache__/`, `*.pyc` e `node_modules/`.
  O `venv` e os caches deixam de ser versionados.

### Removido
- Rastreamento manual por CSRT e botão "Localizar robô" (conflitava com o
  rastreamento ArUco do `localization_node`). Recuperável pelo histórico
  (`6ba8132`, `be8f3fd`). `package.xml` volta a depender de `python3-opencv`.

### Corrigido
- `index.html` carregava `style.css` e `app.js` duas vezes.
- Calibração quebrada pelo merge do `dev` (função e elemento de status
  removidos); o progresso agora aparece no console.

### Decisões e limitações conhecidas
- Contraste, exposição, ganho e foco aparecem desativados: `ImageParams` só
  tem brilho, saturação e hue, e o ROS não foi alterado.
- "Posição alvo" é o último alvo clicado e "Último ponto alvo" é o anterior;
  não há detecção de chegada ao alvo.
- `style.css` é gerado a partir de `input.css` (`npm run build:css`), mas as
  regras novas (`.canvas-wrap`, `.no-video`, cursor de seleção, slider
  desativado) foram espelhadas à mão nos dois arquivos; rode o build quando
  houver `node_modules` e confira o resultado.
- Validado com ROS 2 Jazzy, câmera sintética e Bluetooth simulado; a parte
  com o Bluetooth real (pareamento, 3 conexões simultâneas) e os robôs
  físicos ainda precisam ser validados no Raspberry Pi 5.
- A conexão Bluetooth depende de o ESP32 ter o firmware novo (apresentação
  `ZENIT,<nome>`) e de usar o ESP32 original (só ele tem Bluetooth clássico).
- Os IDs 0, 1 e 2 em `robots.yaml` são provisórios: troque pelos IDs reais
  dos marcadores ArUco.

### Histórico
Os commits do Gustavo (`style:`/`feat:`) e do Danilo (calibração) foram
reaplicados com `git cherry-pick` preservando a autoria, sobre uma base sem
o `venv` que havia sido versionado em `2cb46ba`.
