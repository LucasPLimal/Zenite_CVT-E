"""Reserva de robôs por usuário (em memória, sem ROS).

Cada navegador tem um *token* (guardado no localStorage e enviado ao abrir o
WebSocket). Um robô escolhido fica reservado ao token:

  * enquanto houver uma conexão (aba) aberta com esse token;
  * por `grace_s` segundos (30 s por padrão) depois que a última conexão cai.
    Nesse intervalo o robô é parado na hora, por segurança, mas continua
    reservado: se o mesmo navegador voltar, retoma o robô; senão ele é
    liberado.

Também há `release_all()` (botão "Tornar todos os robôs livres", usado na fase
de testes) e a atualização da lista de robôs vinda do Bluetooth.

A classe não conhece ROS nem asyncio: recebe `on_stop(robot_id)` (para mandar
o robô parar) e um relógio injetável, o que permite testá-la sem hardware.
"""

import time
from typing import Callable, Dict, Optional, Set

MAX_ROBOTS = 8


class ReservationManager:
    def __init__(self, grace_s: float = 30.0,
                 on_stop: Optional[Callable[[int], None]] = None,
                 clock: Callable[[], float] = time.monotonic):
        self.grace_s = grace_s
        self._on_stop = on_stop or (lambda robot_id: None)
        self._clock = clock

        # id -> {'name': str, 'bluetooth': bool}
        self._robots: Dict[int, dict] = {}
        # id -> token dono do robô
        self._owner: Dict[int, str] = {}
        # id -> instante em que a reserva expira (só durante a tolerância)
        self._grace_until: Dict[int, float] = {}
        # token -> conexões abertas
        self._connections: Dict[str, Set[int]] = {}

    # ------------------------------------------------------------------ #
    # Lista de robôs (vem do gerenciador de Bluetooth)
    # ------------------------------------------------------------------ #

    def update_robots(self, robots: Dict[int, dict]) -> bool:
        """Atualiza os robôs conhecidos. Retorna True se algo mudou.

        `robots` é {id: {'name': str, 'bluetooth': bool}}. No máximo
        MAX_ROBOTS (os de menor ID). Robô que perdeu o Bluetooth é liberado.
        """
        novo = {int(i): {'name': str(r.get('name', f'Robô {i}')),
                         'bluetooth': bool(r.get('bluetooth'))}
                for i, r in sorted(robots.items())[:MAX_ROBOTS]}
        changed = novo != self._robots

        for robot_id in list(self._owner):
            if robot_id not in novo or not novo[robot_id]['bluetooth']:
                self._free(robot_id, stop=True)
                changed = True

        self._robots = novo
        return changed

    # ------------------------------------------------------------------ #
    # Conexões (abas) dos navegadores
    # ------------------------------------------------------------------ #

    def connect(self, conn_id: int, token: str) -> bool:
        """Registra uma conexão. Se o token tinha um robô em tolerância, retoma."""
        self._connections.setdefault(token, set()).add(conn_id)
        robot_id = self.robot_of(token)
        if robot_id is not None and robot_id in self._grace_until:
            del self._grace_until[robot_id]
            return True
        return False

    def disconnect(self, conn_id: int) -> bool:
        """Remove uma conexão. Se era a última do token, para o robô e inicia
        a tolerância. Retorna True se o estado das reservas mudou."""
        for token, conns in list(self._connections.items()):
            if conn_id not in conns:
                continue
            conns.discard(conn_id)
            if conns:
                return False
            del self._connections[token]
            robot_id = self.robot_of(token)
            if robot_id is None:
                return False
            self._on_stop(robot_id)
            self._grace_until[robot_id] = self._clock() + self.grace_s
            return True
        return False

    def tick(self) -> bool:
        """Libera as reservas cuja tolerância acabou. Retorna True se liberou."""
        agora = self._clock()
        vencidos = [r for r, t in self._grace_until.items() if agora >= t]
        for robot_id in vencidos:
            self._free(robot_id, stop=True)
        return bool(vencidos)

    # ------------------------------------------------------------------ #
    # Ações do usuário
    # ------------------------------------------------------------------ #

    def select(self, token: str, robot_id: int):
        """Reserva o robô para o token. Retorna (ok, mensagem_de_erro)."""
        robot = self._robots.get(robot_id)
        if robot is None:
            return False, 'Robô não cadastrado.'
        if not robot['bluetooth']:
            return False, 'Robô sem conexão Bluetooth.'
        dono = self._owner.get(robot_id)
        if dono is not None and dono != token:
            return False, 'Robô já está em uso por outra pessoa.'

        atual = self.robot_of(token)
        if atual is not None and atual != robot_id:
            self._free(atual, stop=True)

        self._owner[robot_id] = token
        self._grace_until.pop(robot_id, None)
        return True, ''

    def release(self, token: str) -> bool:
        """Libera o robô do token (para o robô). Retorna True se havia um."""
        robot_id = self.robot_of(token)
        if robot_id is None:
            return False
        self._free(robot_id, stop=True)
        return True

    def release_all(self) -> bool:
        """Libera todos os robôs, de qualquer dono, parando cada um."""
        ids = list(self._owner)
        for robot_id in ids:
            self._free(robot_id, stop=True)
        return bool(ids)

    # ------------------------------------------------------------------ #
    # Consultas
    # ------------------------------------------------------------------ #

    def robot_of(self, token: str) -> Optional[int]:
        for robot_id, dono in self._owner.items():
            if dono == token:
                return robot_id
        return None

    def owns(self, token: str, robot_id: int) -> bool:
        return self._owner.get(robot_id) == token

    def view_for(self, token: str, markers: Optional[Set[int]] = None) -> list:
        """Lista de robôs no formato enviado ao cliente (visão do `token`)."""
        markers = markers or set()
        saida = []
        for robot_id, robot in sorted(self._robots.items()):
            dono = self._owner.get(robot_id)
            if dono is None:
                status = 'free'
            elif dono == token:
                status = 'mine'
            else:
                status = 'busy'
            saida.append({
                'id': robot_id,
                'name': robot['name'],
                'status': status,
                'bluetooth': robot['bluetooth'],
                'marker': robot_id in markers,
            })
        return saida

    # ------------------------------------------------------------------ #

    def _free(self, robot_id: int, stop: bool):
        self._owner.pop(robot_id, None)
        self._grace_until.pop(robot_id, None)
        if stop:
            self._on_stop(robot_id)
