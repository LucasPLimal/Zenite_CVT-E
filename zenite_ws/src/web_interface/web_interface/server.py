"""Backend FastAPI da interface web do Zênite (HTTP + WebSocket).

Um único WebSocket (/ws) por cliente transporta tudo:
  servidor -> cliente : frames JPEG (mensagens binárias) e JSON
      {"type": "pose" | "status" | "goal_ack" | "calibrate_ack" | "error"
               | "robots" | "bluetooth" | "bt_event"}
  cliente -> servidor : JSON
      {"type": "hello", "token": ...}            identifica o navegador
      {"type": "select_robot", "id": n}          escolhe um robô livre
      {"type": "release_robot"}                  devolve o robô
      {"type": "release_all"}                    libera todos (fase de testes)
      {"type": "bt_scan"} | {"type": "bt_connect", "mac": ...}
      {"type": "goal" | "params" | "calibrate" | "reload_homography"}

Multiusuário: cada navegador tem um token; o robô escolhido fica reservado a
ele (ver reservations.py). Alvo (clique) e telemetria valem só para o robô do
usuário. Calibração e parâmetros de imagem são globais e livres para todos.
Sem o bluetooth_node rodando, a interface funciona no modo antigo: um robô só
(ID 0), sem tela de escolha.

Todo o envio ao cliente é feito por uma única task (_stream_to_client), que
drena uma fila asyncio de mensagens JSON e envia o frame/pose mais recentes —
duas tasks nunca escrevem no mesmo WebSocket ao mesmo tempo.
"""

import asyncio
import contextlib
import uuid
from typing import Dict

from ament_index_python.packages import get_package_share_directory
from fastapi import FastAPI, Request, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse
from fastapi.staticfiles import StaticFiles
from fastapi.templating import Jinja2Templates

from .reservations import ReservationManager
from .ros_bridge import RosBridgeNode

STREAM_RATE_HZ = 30.0
HOUSEKEEPING_PERIOD_S = 0.25
LEGACY_ROBOT_ID = 0


class Client:
    def __init__(self, token: str):
        self.token = token
        self.queue: asyncio.Queue = asyncio.Queue()


def create_app(bridge: RosBridgeNode) -> FastAPI:
    share_dir = get_package_share_directory('web_interface')
    reservations = ReservationManager(
        grace_s=bridge.reservation_grace_s, on_stop=bridge.publish_stop)
    clients: Dict[int, Client] = {}
    last_markers = set()
    last_bt_version = -1

    # ------------------------------------------------------------------ #
    # Mensagens para os clientes
    # ------------------------------------------------------------------ #

    def multiuser() -> bool:
        return bridge.has_bluetooth()

    def robots_message(client: Client) -> dict:
        return {'type': 'robots',
                'robots': reservations.view_for(client.token, last_markers)}

    def bluetooth_message() -> dict:
        state, _ = bridge.get_bluetooth()
        state = state or {}
        return {'type': 'bluetooth',
                'scanning': bool(state.get('scanning')),
                'devices': state.get('devices', [])}

    def push_robots():
        if not multiuser():
            return
        for client in clients.values():
            client.queue.put_nowait(robots_message(client))

    def push_all(message: dict):
        for client in clients.values():
            client.queue.put_nowait(message)

    def refresh_from_bluetooth():
        """Reflete no sistema o estado novo do bluetooth_node."""
        nonlocal last_bt_version
        state, version = bridge.get_bluetooth()
        if state is None or version == last_bt_version:
            return
        last_bt_version = version
        reservations.update_robots({
            r['id']: {'name': r.get('label') or r['name'],
                      'bluetooth': r['bluetooth']}
            for r in state.get('robots', [])
        })
        push_all(bluetooth_message())
        push_robots()

    async def housekeeping():
        """Tarefa de fundo: liberações por tempo, marcadores e Bluetooth."""
        nonlocal last_markers
        while True:
            await asyncio.sleep(HOUSEKEEPING_PERIOD_S)
            changed = False

            refresh_from_bluetooth()

            if reservations.tick():
                changed = True

            markers = bridge.visible_markers()
            if markers != last_markers:
                last_markers = markers
                changed = True

            if changed:
                push_robots()

            for evento in bridge.pop_bluetooth_events():
                push_all({'type': 'bt_event', **evento})

    @contextlib.asynccontextmanager
    async def lifespan(app: FastAPI):
        task = asyncio.create_task(housekeeping())
        try:
            yield
        finally:
            task.cancel()

    app = FastAPI(title='Zênite — Interface Web', lifespan=lifespan)
    app.mount('/static', StaticFiles(directory=f'{share_dir}/static'), name='static')
    templates = Jinja2Templates(directory=f'{share_dir}/templates')

    @app.get('/', response_class=HTMLResponse)
    async def index(request: Request):
        return templates.TemplateResponse(request, 'index.html')

    # ------------------------------------------------------------------ #
    # WebSocket
    # ------------------------------------------------------------------ #

    @app.websocket('/ws')
    async def websocket_endpoint(ws: WebSocket):
        await ws.accept()

        conn_id = id(ws)
        # Sem "hello" o navegador recebe um token próprio da conexão.
        client = Client(token=f'conn-{uuid.uuid4().hex}')
        clients[conn_id] = client
        reservations.connect(conn_id, client.token)

        client.queue.put_nowait(
            {'type': 'status', 'homography': bridge.converter.ready})
        refresh_from_bluetooth()  # lista de robôs já atualizada para o novo cliente
        if multiuser():
            client.queue.put_nowait(robots_message(client))
            client.queue.put_nowait(bluetooth_message())

        sender = asyncio.create_task(_stream_to_client(ws, client))
        try:
            while True:
                data = await ws.receive_json()
                await _handle_client_message(conn_id, client, data)
        except WebSocketDisconnect:
            pass
        finally:
            sender.cancel()
            clients.pop(conn_id, None)
            if reservations.disconnect(conn_id):
                push_robots()

    def robot_of(client: Client):
        """ID do robô do cliente (None se ainda não escolheu)."""
        if not multiuser():
            return LEGACY_ROBOT_ID
        return reservations.robot_of(client.token)

    async def _stream_to_client(ws: WebSocket, client: Client):
        """Única task que escreve no WebSocket: fila JSON + frame/pose mais recentes."""
        frame_seq = 0
        pose_seq = 0
        pose_robot = None
        period = 1.0 / STREAM_RATE_HZ
        while True:
            while not client.queue.empty():
                await ws.send_json(client.queue.get_nowait())

            jpeg, frame_seq = bridge.get_frame(frame_seq)
            if jpeg is not None:
                await ws.send_bytes(jpeg)

            robot_id = robot_of(client)
            if robot_id != pose_robot:  # trocou de robô: recomeça a telemetria
                pose_robot, pose_seq = robot_id, 0
            if robot_id is not None:
                pose, pose_seq = bridge.get_pose(robot_id, pose_seq)
                if pose is not None:
                    await ws.send_json({'type': 'pose', **pose})

            await asyncio.sleep(period)

    async def _handle_client_message(conn_id: int, client: Client, data: dict):
        msg_type = data.get('type')
        out = client.queue

        if msg_type == 'hello':
            token = str(data.get('token', ''))[:128]
            if token and token != client.token:
                # Troca o token provisório pelo do navegador (retoma o robô
                # que estiver em tolerância, se houver).
                reservations.disconnect(conn_id)
                client.token = token
                reservations.connect(conn_id, token)
                push_robots()
                if multiuser():
                    out.put_nowait(bluetooth_message())

        elif msg_type == 'select_robot':
            ok, erro = reservations.select(client.token, int(data.get('id', -1)))
            if not ok:
                out.put_nowait({'type': 'error', 'message': erro})
            push_robots()

        elif msg_type == 'release_robot':
            reservations.release(client.token)
            push_robots()

        elif msg_type == 'release_all':
            reservations.release_all()
            push_robots()

        elif msg_type == 'bt_scan':
            bridge.bluetooth_command({'action': 'scan'})

        elif msg_type == 'bt_connect':
            bridge.bluetooth_command(
                {'action': 'connect', 'mac': str(data.get('mac', ''))})

        elif msg_type == 'goal':
            robot_id = robot_of(client)
            if robot_id is None:
                out.put_nowait({'type': 'error',
                                'message': 'Escolha um robô antes de enviar um alvo.'})
                return
            try:
                ack = bridge.publish_goal(
                    robot_id, float(data['x']), float(data['y']))
                out.put_nowait({'type': 'goal_ack', **ack})
            except Exception as e:
                out.put_nowait({'type': 'error', 'message': str(e)})

        elif msg_type == 'params':
            bridge.publish_params(
                brightness=float(data.get('brightness', 0.5)),
                saturation=float(data.get('saturation', 0.5)),
                hue=float(data.get('hue', 0.5)),
            )

        elif msg_type == 'calibrate':
            try:
                bridge.calibrate(data['points'])
                out.put_nowait({'type': 'calibrate_ack', 'homography': True})
            except Exception as e:
                out.put_nowait({'type': 'error', 'message': str(e)})

        elif msg_type == 'reload_homography':
            ok = bridge.try_load_homography()
            out.put_nowait({'type': 'status', 'homography': ok})

    return app
