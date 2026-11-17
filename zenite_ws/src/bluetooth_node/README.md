# bluetooth_node

Gerenciador de Bluetooth dos robôs (ESP32). Procura, pareia e conecta os robôs por RFCOMM
(socket do Python, sem `rfcomm bind`, sem `/dev/rfcomm*` e sem root) e repassa as velocidades
das rodas. Substitui o `transmission_node` + `sudo rfcomm bind` manual no uso multiusuário.

## Como um robô é aceito

1. O usuário clica em **Procurar robôs** na interface web (ou o servidor reconecta sozinho um
   robô já conhecido).
2. O servidor pareia (sem PIN), abre a conexão RFCOMM e espera a apresentação `ZENIT,<nome>`
   que o firmware envia ao conectar.
3. O `<nome>` precisa estar em `config/robots.yaml`. Se a apresentação não chegar, ou o nome
   não estiver no cadastro, a conexão é fechada e o pareamento removido.
4. O MAC descoberto é guardado em `~/.zenite/bluetooth_known.json`; nas próximas vezes o
   robô reconecta sozinho (a cada 10 s, `reconnect_s`).

Máximo de 8 robôs. O ID do robô é o ID do marcador ArUco.

## Cadastro (`config/robots.yaml`)

```yaml
robots:
  - {id: 0, name: zenitinho, label: Zenitinho}
```

`name` é o `ROBOT_NAME` do `FirmwareEsp32.ino` (um nome diferente por robô).

## Tópicos

| Tópico | Tipo | Uso |
|---|---|---|
| `/robot_<id>/wheel_velocities` | `geometry_msgs/Vector3` | entrada: x = roda direita, y = esquerda |
| `/robot_<id>/stop` | `std_msgs/Bool` | entrada: envia parada ao robô |
| `/bluetooth/command` | `std_msgs/String` (JSON) | `{"action":"scan"}`, `{"action":"connect","mac":"…"}`, `{"action":"disconnect","id":0}` |
| `/bluetooth/state` | `std_msgs/String` (JSON, transient_local) | busca em andamento, dispositivos e robôs |
| `/bluetooth/events` | `std_msgs/String` (JSON) | avisos pontuais (conectado, recusado…) |

Parâmetros: `robots_file`, `known_file`, `max_send_hz` (30), `auto_reconnect` (true),
`reconnect_s` (10).

## Pacote de 3 bytes

`[direção, PWM_esquerdo, PWM_direito]`, igual ao `transmission_node`: bit 0 = roda direita
para frente, bit 1 = esquerda para frente; velocidade limitada a ±40 e convertida em PWM 0–255.

## Requisitos e limites

- BlueZ (`bluetoothctl`) no Raspberry Pi: `sudo apt install bluez`.
- Só o ESP32 original tem Bluetooth clássico (C3, S2, C6 e H2 não têm).
- A busca de dispositivos degrada as conexões ativas: só é feita sob demanda.
- Wi-Fi e Bluetooth dividem a faixa de 2,4 GHz; prefira cabo ou Wi-Fi 5 GHz no servidor.
- **A parte com o Bluetooth real ainda precisa ser validada no Raspberry Pi** (pareamento sem
  PIN, canal RFCOMM 1, 3 conexões simultâneas). Use `tools/bluetooth_poc.py` para isso.

## Testes

```bash
python3 -m pytest src/bluetooth_node/test/
```
