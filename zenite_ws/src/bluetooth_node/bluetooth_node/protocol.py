"""Protocolo com o ESP32 (sem ROS): apresentação e pacote de comando."""

HELLO_PREFIX = 'ZENIT,'
VMAX = 40.0  # mesma velocidade máxima do transmission_node (C++)


def parse_hello(linha: str):
    """'ZENIT,<nome>' -> nome; qualquer outra coisa -> None."""
    linha = linha.strip()
    if not linha.startswith(HELLO_PREFIX):
        return None
    nome = linha[len(HELLO_PREFIX):].strip()
    return nome or None


def build_packet(v_dir: float, v_esq: float, vmax: float = VMAX) -> bytes:
    """Monta o pacote de 3 bytes [direção, PWM esq., PWM dir.].

    Mesma conta do transmission_node: bit 0 = roda direita para frente,
    bit 1 = roda esquerda para frente; velocidade limitada a ±vmax e
    convertida para PWM de 0 a 255.
    """
    direcao = 0
    if v_dir >= 0:
        direcao |= 0b01
    if v_esq >= 0:
        direcao |= 0b10

    v_esq = max(-vmax, min(vmax, v_esq))
    v_dir = max(-vmax, min(vmax, v_dir))
    pwm_esq = int(abs(v_esq) / vmax * 255.0)
    pwm_dir = int(abs(v_dir) / vmax * 255.0)
    return bytes([direcao, pwm_esq, pwm_dir])


STOP_PACKET = bytes([0b11, 0, 0])
