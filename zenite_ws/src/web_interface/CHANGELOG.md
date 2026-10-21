# Changelog — web_interface

Formato baseado em [Keep a Changelog](https://keepachangelog.com/pt-BR/1.1.0/).
Commits seguem [Conventional Commits](https://www.conventionalcommits.org/pt-br/v1.0.0/)
(`feat`, `fix`, `style`, `chore`, `docs`) com escopo `web_interface`.

## [Não lançado] — branch `integration/web-interface`

Integração do front-end (Gustavo Andrey, `feature/componentes-css`) com o
back-end (`feature/calibration_node`), sobre `origin/main`.

### Adicionado
- Rastreamento do robô via `cv2.TrackerCSRT` no backend (`locate_robot`,
  `reset_tracking`), publicando `/current_position` com cooldown de 50 ms.
- Front ligado ao WebSocket: vídeo, badges, console, clique para alvo,
  sliders de brilho/saturação/matiz, calibração por 4 cliques, localizar
  robô por arraste, toggles de visualização e telemetria (atual, inicial,
  alvo, último alvo e área de calibragem).
- Botão "Recarregar homografia" na seção de calibração.

### Alterado
- `package.xml`: removida a dependência `python3-opencv`; o CSRT exige
  `opencv-contrib` (ver README).
- `.gitignore`: ignora `venv/`, `__pycache__/`, `*.pyc` e `node_modules/`.
  O `venv` e os caches deixam de ser versionados.

### Corrigido
- `index.html` carregava `style.css` e `app.js` duas vezes.

### Decisões e limitações conhecidas
- Contraste, exposição, ganho e foco aparecem desativados: `ImageParams` só
  tem brilho, saturação e hue, e o ROS não foi alterado.
- "Posição alvo" é o último alvo clicado e "Último ponto alvo" é o anterior;
  não há detecção de chegada ao alvo.
- "Parar rastreamento" substitui o botão Resetar removido do layout.
- `style.css` é gerado a partir de `input.css` (`npm run build:css`), mas as
  regras novas (`.canvas-wrap`, `.no-video`, cursor de seleção, slider
  desativado) foram espelhadas à mão nos dois arquivos; rode o build quando
  houver `node_modules` e confira o resultado.
- Não rode `localization_node` junto: ambos publicam em `/current_position`.
- Validado com ROS 2 Jazzy e câmera sintética; ainda não testado com câmera
  e robô físicos nem pelo celular.

### Histórico
Os commits do Gustavo (`style:`/`feat:`) e do Danilo (calibração) foram
reaplicados com `git cherry-pick` preservando a autoria, sobre uma base sem
o `venv` que havia sido versionado em `2cb46ba`.
