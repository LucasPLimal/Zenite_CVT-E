"""Gerenciador de conexões Bluetooth dos robôs (sem ROS).

Fluxo de um robô novo:
  procurar (scan) -> usuário escolhe um dispositivo -> parear -> abrir RFCOMM
  -> esperar a apresentação "ZENIT,<nome>" -> conferir o nome no cadastro.
Se a apresentação não chega ou o nome não está no cadastro, o dispositivo não
é um robô: a conexão é fechada e o pareamento removido.

Robôs já conhecidos (MAC guardado) são reconectados sozinhos em segundo plano.

As partes que falam com o Bluetooth real (`connector`, `scanner`, `pairer`,
`remover`) são injetáveis, o que permite testar o fluxo sem hardware.
"""

import threading
import time
from typing import Callable, Dict, Optional

from . import bluez
from .protocol import STOP_PACKET, build_packet, parse_hello
from .registry import MAX_ROBOTS, KnownStore, Registry


class _Link:
    def __init__(self, spec, mac, sock):
        self.spec = spec
        self.mac = mac
        self.sock = sock
        self.send_lock = threading.Lock()
        self.stop_reader = threading.Event()


class BluetoothManager:
    def __init__(self, registry: Registry, known: KnownStore,
                 connector: Callable = bluez.conectar_rfcomm,
                 scanner: Callable = bluez.buscar,
                 pairer: Callable = bluez.parear,
                 remover: Callable = bluez.esquecer,
                 is_paired: Callable = bluez.esta_pareado,
                 hello_timeout: float = 5.0,
                 reconnect_s: float = 10.0,
                 on_change: Optional[Callable[[], None]] = None,
                 on_event: Optional[Callable[[dict], None]] = None):
        self.registry = registry
        self.known = known
        self._connector = connector
        self._scanner = scanner
        self._pairer = pairer
        self._remover = remover
        self._is_paired = is_paired
        self.hello_timeout = hello_timeout
        self.reconnect_s = reconnect_s
        self._on_change = on_change or (lambda: None)
        self._on_event = on_event or (lambda ev: None)

        self._lock = threading.Lock()          # protege links/scan/devices
        self._connect_lock = threading.Lock()  # uma conexão por vez
        self._links: Dict[int, _Link] = {}
        self._scanning = False
        self._devices = []
        self._running = False

    # ------------------------------------------------------------------ #
    # Estado
    # ------------------------------------------------------------------ #

    def state(self) -> dict:
        with self._lock:
            robots = []
            for spec in self.registry.robots:
                link = self._links.get(spec.id)
                robots.append({
                    'id': spec.id,
                    'name': spec.name,
                    'label': spec.label,
                    'mac': link.mac if link else self.known.get(spec.name),
                    'bluetooth': link is not None,
                })
            return {'scanning': self._scanning,
                    'devices': list(self._devices),
                    'robots': robots}

    def is_connected(self, robot_id: int) -> bool:
        with self._lock:
            return robot_id in self._links

    # ------------------------------------------------------------------ #
    # Busca
    # ------------------------------------------------------------------ #

    def scan(self) -> None:
        """Procura dispositivos (bloqueia). Atualiza a lista de dispositivos."""
        with self._lock:
            if self._scanning:
                return
            self._scanning = True
            self._devices = []
        self._on_change()

        try:
            achados = self._scanner()
            erro = None
        except Exception as e:  # BlueZ ausente, adaptador desligado etc.
            achados, erro = {}, str(e)

        with self._lock:
            conectados = {l.mac for l in self._links.values()}
            devs = [{'mac': mac, 'name': nome,
                     'likely_robot': self.registry.by_name(nome) is not None,
                     'connected': mac in conectados}
                    for mac, nome in achados.items()]
            devs.sort(key=lambda d: (not d['likely_robot'], d['name'] == '',
                                     d['name'].lower()))
            self._devices = devs
            self._scanning = False
        if erro:
            self._on_event({'event': 'scan_error',
                            'message': f'Falha na busca Bluetooth: {erro}'})
        self._on_change()

    def scan_async(self) -> None:
        threading.Thread(target=self.scan, daemon=True).start()

    # ------------------------------------------------------------------ #
    # Conexão
    # ------------------------------------------------------------------ #

    def connect(self, mac: str, pair: bool = True, quiet: bool = False):
        """Conecta a um dispositivo e confirma que é um robô cadastrado.

        Retorna (ok, mensagem). Com quiet=True falhas não geram evento
        (usado na reconexão automática).
        """
        mac = mac.upper()
        with self._connect_lock:
            with self._lock:
                if any(l.mac == mac for l in self._links.values()):
                    return False, 'Esse dispositivo já está conectado.'
                if len(self._links) >= MAX_ROBOTS:
                    return False, f'Limite de {MAX_ROBOTS} robôs conectados.'

            def falha(msg, evento='connect_failed'):
                if not quiet:
                    self._on_event({'event': evento, 'mac': mac, 'message': msg})
                return False, msg

            if pair:
                try:
                    if not self._is_paired(mac):
                        self._pairer(mac)
                except Exception:
                    pass  # tenta conectar mesmo assim

            try:
                sock = self._connector(mac)
            except OSError as e:
                return falha(f'Não foi possível conectar: {e}')

            nome = self._read_hello(sock)
            if nome is None:
                sock.close()
                if pair:
                    self._remover(mac)
                return falha('O dispositivo não se apresentou como robô.',
                             'not_a_robot')

            spec = self.registry.by_name(nome)
            if spec is None:
                sock.close()
                if pair:
                    self._remover(mac)
                return falha(f'Robô "{nome}" não está no cadastro.',
                             'not_registered')

            with self._lock:
                if spec.id in self._links:
                    sock.close()
                    return falha(f'O robô {spec.label} já está conectado.')
                link = _Link(spec, mac, sock)
                self._links[spec.id] = link

            self.known.set(spec.name, mac)
            self._send(link, STOP_PACKET)
            threading.Thread(target=self._reader, args=(link,),
                             daemon=True).start()

        self._on_event({'event': 'robot_connected', 'id': spec.id,
                        'name': spec.name, 'label': spec.label, 'mac': mac,
                        'message': f'Robô {spec.label} conectado.'})
        self._on_change()
        return True, ''

    def connect_async(self, mac: str) -> None:
        threading.Thread(target=self.connect, args=(mac,), daemon=True).start()

    def _read_hello(self, sock) -> Optional[str]:
        """Lê a apresentação 'ZENIT,<nome>\\n'. None se não chegar a tempo."""
        buf = b''
        fim = time.monotonic() + self.hello_timeout
        while b'\n' not in buf and len(buf) < 128:
            restante = fim - time.monotonic()
            if restante <= 0:
                return None
            sock.settimeout(restante)
            try:
                dados = sock.recv(64)
            except (OSError, TimeoutError):
                return None
            if not dados:
                return None
            buf += dados
        return parse_hello(buf.split(b'\n')[0].decode(errors='replace'))

    def disconnect(self, robot_id: int) -> None:
        with self._lock:
            link = self._links.get(robot_id)
        if link:
            self._drop(link, 'Desconectado pelo servidor.')

    # ------------------------------------------------------------------ #
    # Envio de comandos
    # ------------------------------------------------------------------ #

    def send_velocity(self, robot_id: int, v_dir: float, v_esq: float) -> bool:
        with self._lock:
            link = self._links.get(robot_id)
        if link is None:
            return False
        return self._send(link, build_packet(v_dir, v_esq))

    def send_stop(self, robot_id: int) -> bool:
        with self._lock:
            link = self._links.get(robot_id)
        if link is None:
            return False
        return self._send(link, STOP_PACKET)

    def _send(self, link: _Link, packet: bytes) -> bool:
        try:
            with link.send_lock:
                link.sock.sendall(packet)
            return True
        except OSError:
            self._drop(link, 'Falha ao enviar: conexão perdida.')
            return False

    # ------------------------------------------------------------------ #
    # Manutenção das conexões
    # ------------------------------------------------------------------ #

    def _reader(self, link: _Link) -> None:
        """Descarta o que o robô envia e detecta a queda da conexão."""
        link.sock.settimeout(1.0)
        while not link.stop_reader.is_set():
            try:
                dados = link.sock.recv(64)
            except TimeoutError:
                continue
            except OSError:
                break
            if not dados:
                break
        self._drop(link, 'Conexão com o robô perdida.')

    def _drop(self, link: _Link, motivo: str) -> None:
        with self._lock:
            if self._links.get(link.spec.id) is not link:
                return
            del self._links[link.spec.id]
        link.stop_reader.set()
        try:
            link.sock.close()
        except OSError:
            pass
        self._on_event({'event': 'robot_disconnected', 'id': link.spec.id,
                        'name': link.spec.name, 'label': link.spec.label,
                        'message': f'Robô {link.spec.label} desconectado.'})
        self._on_change()

    # ------------------------------------------------------------------ #
    # Reconexão automática
    # ------------------------------------------------------------------ #

    def start(self) -> None:
        self._running = True
        threading.Thread(target=self._reconnect_loop, daemon=True).start()

    def stop(self) -> None:
        self._running = False
        with self._lock:
            links = list(self._links.values())
        for link in links:
            self._send(link, STOP_PACKET)
            self._drop(link, 'Encerrando.')

    def _reconnect_loop(self) -> None:
        while self._running:
            with self._lock:
                ocupado = self._scanning
            if not ocupado:
                self.reconnect_known()
            fim = time.monotonic() + self.reconnect_s
            while self._running and time.monotonic() < fim:
                time.sleep(0.2)

    def reconnect_known(self) -> None:
        """Tenta reconectar os robôs com MAC conhecido que estão desligados."""
        for spec in self.registry.robots:
            mac = self.known.get(spec.name)
            if not mac or self.is_connected(spec.id):
                continue
            if not self._running:
                return
            self.connect(mac, pair=False, quiet=True)
