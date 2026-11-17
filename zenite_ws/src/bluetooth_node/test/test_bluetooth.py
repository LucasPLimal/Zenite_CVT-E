"""Testes do protocolo, cadastro e gerenciador (sem hardware nem ROS)."""
import os
import socket
import sys
import time

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from bluetooth_node import bluez  # noqa: E402
from bluetooth_node.manager import BluetoothManager  # noqa: E402
from bluetooth_node.protocol import build_packet, parse_hello  # noqa: E402
from bluetooth_node.registry import KnownStore, Registry, RobotSpec  # noqa: E402

MAC_A = '00:4B:12:3C:74:CE'
MAC_B = '00:4B:12:3B:FF:52'


def espera(cond, limite=3.0):
    fim = time.time() + limite
    while time.time() < fim:
        if cond():
            return True
        time.sleep(0.02)
    return False


# ---------------------------------------------------------------- protocolo

def test_hello():
    assert parse_hello('ZENIT,zenitinho\r\n') == 'zenitinho'
    assert parse_hello('ZENIT,') is None
    assert parse_hello('OLA') is None


def test_pacote_igual_ao_transmission_node():
    # v_dir=20 (50% de 40) -> 127; v_esq=-40 -> 255 e roda esq. para trás
    assert build_packet(20.0, -40.0) == bytes([0b01, 255, 127])
    assert build_packet(-10.0, 10.0) == bytes([0b10, 63, 63])
    assert build_packet(0.0, 0.0) == bytes([0b11, 0, 0])
    assert build_packet(500.0, 500.0) == bytes([0b11, 255, 255])  # satura


def test_parse_bluetoothctl():
    texto = ("\x1b[0;92m[NEW]\x1b[0m Device 00:4B:12:3C:74:CE zenitinho\n"
             "[CHG] Device 00:4B:12:3C:74:CE RSSI: -60\n"
             "[NEW] Device AA-BB-CC-DD-EE-FF AA-BB-CC-DD-EE-FF\n"
             "[CHG] Device AA:BB:CC:DD:EE:FF Name: Celular\n")
    assert bluez.parse_dispositivos(bluez.limpar(texto)) == {
        '00:4B:12:3C:74:CE': 'zenitinho', 'AA:BB:CC:DD:EE:FF': 'Celular'}


# ----------------------------------------------------------------- cadastro

def test_cadastro_valida(tmp_path):
    with pytest.raises(ValueError):
        Registry([RobotSpec(0, 'a', 'A'), RobotSpec(0, 'b', 'B')])
    with pytest.raises(ValueError):
        Registry([RobotSpec(0, 'a', 'A'), RobotSpec(1, 'a', 'B')])
    with pytest.raises(ValueError):
        Registry([RobotSpec(i, f'r{i}', f'R{i}') for i in range(9)])
    reg = Registry.from_yaml(os.path.join(
        os.path.dirname(__file__), '..', 'config', 'robots.yaml'))
    assert [r.id for r in reg.robots] == [0, 1, 2]


def test_known_store_persiste(tmp_path):
    caminho = str(tmp_path / 'sub' / 'known.json')
    KnownStore(caminho).set('zenitinho', MAC_A)
    assert KnownStore(caminho).get('zenitinho') == MAC_A


# ------------------------------------------------------------- gerenciador

class Mundo:
    """Dispositivos falsos: cada MAC tem um socket do 'robô' (lado do ESP32)."""

    def __init__(self):
        self.robos = {}      # mac -> (lado_pi, lado_esp)
        self.pareados = []
        self.esquecidos = []
        self.eventos = []
        self.mudancas = 0

    def adiciona(self, mac, hello):
        pi, esp = socket.socketpair()
        if hello is not None:
            esp.sendall(hello.encode())
        self.robos[mac] = (pi, esp)

    def connector(self, mac):
        if mac not in self.robos:
            raise OSError('Host is down')
        return self.robos[mac][0]


@pytest.fixture
def mundo(tmp_path):
    m = Mundo()
    reg = Registry([RobotSpec(0, 'zenitinho', 'Zenitinho'),
                    RobotSpec(1, 'parnadiloJR', 'Parnadilo JR')])
    known = KnownStore(str(tmp_path / 'known.json'))
    mgr = BluetoothManager(
        reg, known, connector=m.connector,
        scanner=lambda: {MAC_A: 'zenitinho', 'AA:AA:AA:AA:AA:AA': 'Celular'},
        pairer=lambda mac: m.pareados.append(mac) or True,
        remover=lambda mac: m.esquecidos.append(mac),
        is_paired=lambda mac: False,
        hello_timeout=0.5, reconnect_s=0.2,
        on_change=lambda: setattr(m, 'mudancas', m.mudancas + 1),
        on_event=m.eventos.append)
    m.mgr, m.known = mgr, known
    yield m
    mgr.stop()


def test_busca_marca_robos_provaveis(mundo):
    mundo.mgr.scan()
    devs = mundo.mgr.state()['devices']
    assert [d['name'] for d in devs] == ['zenitinho', 'Celular']
    assert devs[0]['likely_robot'] and not devs[1]['likely_robot']


def test_conecta_robo_e_confirma_pela_apresentacao(mundo):
    mundo.adiciona(MAC_A, 'ZENIT,zenitinho\n')
    ok, _ = mundo.mgr.connect(MAC_A)
    assert ok
    assert MAC_A in mundo.pareados
    estado = {r['id']: r for r in mundo.mgr.state()['robots']}
    assert estado[0]['bluetooth'] and estado[0]['mac'] == MAC_A
    assert not estado[1]['bluetooth']
    assert mundo.known.get('zenitinho') == MAC_A
    assert any(e['event'] == 'robot_connected' for e in mundo.eventos)
    # o servidor manda um pacote de parada ao conectar
    assert mundo.robos[MAC_A][1].recv(3) == bytes([0b11, 0, 0])


def test_dispositivo_sem_apresentacao_nao_e_robo(mundo):
    mundo.adiciona('AA:AA:AA:AA:AA:AA', None)
    ok, msg = mundo.mgr.connect('AA:AA:AA:AA:AA:AA')
    assert not ok and 'robô' in msg
    assert 'AA:AA:AA:AA:AA:AA' in mundo.esquecidos
    assert any(e['event'] == 'not_a_robot' for e in mundo.eventos)
    assert not any(r['bluetooth'] for r in mundo.mgr.state()['robots'])


def test_robo_nao_cadastrado_e_recusado(mundo):
    mundo.adiciona(MAC_B, 'ZENIT,desconhecido\n')
    ok, msg = mundo.mgr.connect(MAC_B)
    assert not ok and 'cadastro' in msg
    assert any(e['event'] == 'not_registered' for e in mundo.eventos)


def test_envia_velocidade_e_parada(mundo):
    mundo.adiciona(MAC_A, 'ZENIT,zenitinho\n')
    mundo.mgr.connect(MAC_A)
    esp = mundo.robos[MAC_A][1]
    esp.recv(3)  # parada inicial
    assert mundo.mgr.send_velocity(0, 20.0, -40.0)
    assert esp.recv(3) == bytes([0b01, 255, 127])
    assert mundo.mgr.send_stop(0)
    assert esp.recv(3) == bytes([0b11, 0, 0])
    assert not mundo.mgr.send_velocity(1, 1.0, 1.0)  # robô 1 não conectado


def test_detecta_queda(mundo):
    mundo.adiciona(MAC_A, 'ZENIT,zenitinho\n')
    mundo.mgr.connect(MAC_A)
    mundo.robos[MAC_A][1].close()  # robô desligou
    assert espera(lambda: not mundo.mgr.is_connected(0))
    assert any(e['event'] == 'robot_disconnected' for e in mundo.eventos)


def test_reconexao_automatica_de_robo_conhecido(mundo):
    mundo.adiciona(MAC_A, 'ZENIT,zenitinho\n')
    mundo.mgr.connect(MAC_A)
    mundo.robos[MAC_A][1].close()
    assert espera(lambda: not mundo.mgr.is_connected(0))
    mundo.adiciona(MAC_A, 'ZENIT,zenitinho\n')  # robô voltou
    mundo.mgr.start()
    assert espera(lambda: mundo.mgr.is_connected(0))


def test_nao_conecta_duas_vezes_o_mesmo_dispositivo(mundo):
    mundo.adiciona(MAC_A, 'ZENIT,zenitinho\n')
    assert mundo.mgr.connect(MAC_A)[0]
    assert not mundo.mgr.connect(MAC_A)[0]
