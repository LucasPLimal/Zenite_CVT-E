"""Testes da reserva de robôs (sem ROS): python3 -m pytest test/"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from web_interface.reservations import ReservationManager  # noqa: E402

ROBOS = {
    0: {'name': 'Zenitinho', 'bluetooth': True},
    1: {'name': 'Parnadilo JR', 'bluetooth': True},
    2: {'name': 'Jerimum', 'bluetooth': False},
}


class Relogio:
    def __init__(self):
        self.t = 100.0

    def __call__(self):
        return self.t


def novo():
    relogio = Relogio()
    paradas = []
    m = ReservationManager(grace_s=30, on_stop=paradas.append, clock=relogio)
    m.update_robots(ROBOS)
    return m, relogio, paradas


def status(m, token):
    return {r['id']: r['status'] for r in m.view_for(token)}


def test_escolha_e_exclusividade():
    m, _, _ = novo()
    m.connect(1, 'A')
    m.connect(2, 'B')
    assert m.select('A', 0) == (True, '')
    assert status(m, 'A')[0] == 'mine'
    assert status(m, 'B')[0] == 'busy'
    ok, erro = m.select('B', 0)
    assert not ok and 'uso' in erro


def test_robo_sem_bluetooth_nao_pode_ser_escolhido():
    m, _, _ = novo()
    ok, erro = m.select('A', 2)
    assert not ok and 'Bluetooth' in erro
    assert not m.select('A', 9)[0]


def test_queda_para_na_hora_e_reserva_por_30s():
    m, relogio, paradas = novo()
    m.connect(1, 'A')
    m.select('A', 0)
    m.disconnect(1)
    assert paradas == [0]                       # parou imediatamente
    assert status(m, 'B')[0] == 'busy'          # ainda reservado
    relogio.t += 29
    assert not m.tick()
    assert status(m, 'B')[0] == 'busy'
    relogio.t += 2
    assert m.tick()                             # passou de 30 s: liberou
    assert status(m, 'B')[0] == 'free'


def test_voltar_dentro_da_tolerancia_retoma_o_robo():
    m, relogio, _ = novo()
    m.connect(1, 'A')
    m.select('A', 0)
    m.disconnect(1)
    relogio.t += 10
    assert m.connect(5, 'A') is True            # retomou
    relogio.t += 60
    assert not m.tick()                         # tolerância cancelada
    assert status(m, 'A')[0] == 'mine'


def test_duas_abas_do_mesmo_token():
    m, _, paradas = novo()
    m.connect(1, 'A')
    m.connect(2, 'A')
    m.select('A', 0)
    m.disconnect(1)
    assert paradas == []                        # ainda há uma aba aberta
    m.disconnect(2)
    assert paradas == [0]


def test_liberar_proprio_robo_e_trocar():
    m, _, paradas = novo()
    m.connect(1, 'A')
    m.select('A', 0)
    assert m.release('A')
    assert paradas == [0]
    assert status(m, 'A')[0] == 'free'
    m.select('A', 0)
    m.select('A', 1)                            # trocar libera o anterior
    assert status(m, 'A') == {0: 'free', 1: 'mine', 2: 'free'}


def test_liberar_todos():
    m, _, paradas = novo()
    m.connect(1, 'A')
    m.connect(2, 'B')
    m.select('A', 0)
    m.select('B', 1)
    assert m.release_all()
    assert sorted(paradas) == [0, 1]
    assert set(status(m, 'A').values()) == {'free'}
    assert not m.release_all()


def test_robo_que_perde_bluetooth_e_liberado():
    m, _, paradas = novo()
    m.connect(1, 'A')
    m.select('A', 0)
    robos = {i: dict(r) for i, r in ROBOS.items()}
    robos[0]['bluetooth'] = False
    assert m.update_robots(robos)
    assert paradas == [0]
    assert status(m, 'A')[0] == 'free'


def test_maximo_de_8_robos():
    m, _, _ = novo()
    m.update_robots({i: {'name': f'R{i}', 'bluetooth': True} for i in range(12)})
    assert len(m.view_for('A')) == 8


def test_marcador_visivel():
    m, _, _ = novo()
    vista = {r['id']: r['marker'] for r in m.view_for('A', markers={1})}
    assert vista == {0: False, 1: True, 2: False}
