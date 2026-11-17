"""Teste integrado do servidor multiusuário (precisa do ROS 2, sem hardware).

Sobe a ponte ROS e o app FastAPI no mesmo processo, simula o bluetooth_node
(estado, eventos) e dois navegadores, e confere: lista de robôs, reserva
exclusiva, alvo/pose só do dono, parada na queda, tolerância de 30 s (2 s aqui),
retomada pelo mesmo token, liberar todos e perda de Bluetooth.

    source /opt/ros/jazzy/setup.bash && source install/setup.bash
    ROS_DOMAIN_ID=79 python3 src/web_interface/test/integracao_multiusuario.py
"""
import json, os, sys, threading, time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import String, Bool, Float64MultiArray
from geometry_msgs.msg import Point
import asyncio

rclpy.init(args=['--ros-args', '-p', 'reservation_grace_s:=2.0',
                 '-p', 'scale_yaml_path:=/tmp/scale_integ.yaml'])
from web_interface.ros_bridge import RosBridgeNode
from web_interface.server import create_app

bridge = RosBridgeNode()
bridge.calibrate([[0, 0], [275, 0], [275, 225], [0, 225]])

class Fake(Node):
    def __init__(self):
        super().__init__('fake_bt')
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.state_pub = self.create_publisher(String, '/bluetooth/state', latched)
        self.ev_pub = self.create_publisher(String, '/bluetooth/events', 10)
        self.pose_pubs = {i: self.create_publisher(Float64MultiArray, f'/robot_{i}/current_pose', 10) for i in (0, 1)}
        self.got = {'goal0': [], 'stop0': [], 'stop1': [], 'cmd': []}
        self.create_subscription(Point, '/robot_0/desired_position', lambda m: self.got['goal0'].append((m.x, m.y)), 10)
        self.create_subscription(Bool, '/robot_0/stop', lambda m: self.got['stop0'].append(1), 10)
        self.create_subscription(Bool, '/robot_1/stop', lambda m: self.got['stop1'].append(1), 10)
        self.create_subscription(String, '/bluetooth/command', lambda m: self.got['cmd'].append(m.data), 10)
        self.create_timer(0.1, self.poses)
    def publish_state(self, bt0=True, bt1=True, bt2=False):
        st = {'scanning': False, 'devices': [{'mac': 'AA:AA', 'name': 'zenitinho', 'likely_robot': True}],
              'robots': [{'id': 0, 'name': 'zenitinho', 'label': 'Zenitinho', 'mac': 'M0', 'bluetooth': bt0},
                         {'id': 1, 'name': 'parnadiloJR', 'label': 'Parnadilo JR', 'mac': 'M1', 'bluetooth': bt1},
                         {'id': 2, 'name': 'Jerimum', 'label': 'Jerimum', 'mac': None, 'bluetooth': bt2}]}
        self.state_pub.publish(String(data=json.dumps(st)))
    def poses(self):
        for i, p in self.pose_pubs.items():
            p.publish(Float64MultiArray(data=[0.5 + i, 0.5, 0.25]))

fake = Fake()
from rclpy.executors import MultiThreadedExecutor
ex = MultiThreadedExecutor(); ex.add_node(bridge); ex.add_node(fake)
threading.Thread(target=ex.spin, daemon=True).start()
time.sleep(1.0)
fake.publish_state()
time.sleep(1.0)


class Sess:
    """Cliente WebSocket falso falando ASGI direto com o app (sem httpx/websockets)."""
    def __init__(self, app):
        self.app = app
        self.inbox = asyncio.Queue()    # cliente -> servidor
        self.outbox = asyncio.Queue()   # servidor -> cliente
        self.task = None
    async def __aenter__(self):
        scope = {'type': 'websocket', 'asgi': {'version': '3.0'}, 'path': '/ws', 'raw_path': b'/ws',
                 'query_string': b'', 'headers': [], 'subprotocols': [], 'scheme': 'ws',
                 'server': ('x', 80), 'client': ('y', 1), 'root_path': ''}
        await self.inbox.put({'type': 'websocket.connect'})
        async def receive(): return await self.inbox.get()
        async def send(m): await self.outbox.put(m)
        self.task = asyncio.create_task(self.app(scope, receive, send))
        return self
    async def __aexit__(self, *a):
        await self.inbox.put({'type': 'websocket.disconnect', 'code': 1000})
        await asyncio.wait_for(self.task, 5)
    async def send_json(self, d):
        await self.inbox.put({'type': 'websocket.receive', 'text': json.dumps(d)})

async def recv_until(ws, pred, timeout=5.0):
    fim = time.time() + timeout
    seen = []
    while time.time() < fim:
        try:
            m = await asyncio.wait_for(ws.outbox.get(), max(0.05, fim - time.time()))
        except asyncio.TimeoutError:
            break
        if m.get('text'):
            d = json.loads(m['text']); seen.append(d)
            if pred(d): return d, seen
    raise AssertionError(f'não chegou; vi {[s["type"] for s in seen][-6:]}')

class R(dict):
    def __missing__(self, k): return None
def robots(d): return R({r['id']: r['status'] for r in d['robots']})
def ok(msg): print('OK  ', msg)

async def main():
    app = create_app(bridge)
    async with app.router.lifespan_context(app):
        async with Sess(app) as A, Sess(app) as B:
            await A.send_json({'type': 'hello', 'token': 'tokA'})
            await B.send_json({'type': 'hello', 'token': 'tokB'})
            d, _ = await recv_until(A, lambda d: d['type'] == 'robots' and len(d['robots']) == 3)
            assert robots(d) == {0: 'free', 1: 'free', 2: 'free'}, robots(d)
            assert d['robots'][2]['bluetooth'] is False
            ok('lista de 3 robôs (o 3º sem Bluetooth)')

            await A.send_json({'type': 'select_robot', 'id': 0})
            await recv_until(A, lambda d: d['type'] == 'robots' and robots(d)[0] == 'mine')
            await recv_until(B, lambda d: d['type'] == 'robots' and robots(d)[0] == 'busy')
            ok('A reservou o robô 0; B o vê ocupado')

            await B.send_json({'type': 'select_robot', 'id': 0})
            d, _ = await recv_until(B, lambda d: d['type'] == 'error')
            assert 'uso' in d['message']; ok('B não consegue pegar o robô 0')
            await B.send_json({'type': 'goal', 'x': 10, 'y': 10})
            d, _ = await recv_until(B, lambda d: d['type'] == 'error')
            assert 'Escolha' in d['message']; ok('B sem robô não envia alvo')

            await A.send_json({'type': 'goal', 'x': 100, 'y': 50})
            await recv_until(A, lambda d: d['type'] == 'goal_ack')
            await asyncio.sleep(0.5)
            assert len(fake.got['goal0']) == 1, fake.got
            ok(f'alvo de A chegou em /robot_0/desired_position {fake.got["goal0"][0]}')

            d, _ = await recv_until(A, lambda d: d['type'] == 'pose')
            assert d['robot_id'] == 0 and abs(d['theta'] - 0.25) < 1e-6
            ok(f'A recebe a pose do robô 0 com theta ({d["x"]}, {d["y"]})')
            try:
                await recv_until(B, lambda d: d['type'] == 'pose', timeout=1.0)
                raise SystemExit('B recebeu pose de robô que não é dele!')
            except AssertionError:
                ok('B (sem robô) não recebe pose')

            await A.send_json({'type': 'bt_scan'})
            await asyncio.sleep(0.4)
            assert any('scan' in x for x in fake.got['cmd']); ok('bt_scan vira comando no tópico')

            await B.send_json({'type': 'release_all'})
            await recv_until(A, lambda d: d['type'] == 'robots' and robots(d)[0] == 'free')
            await asyncio.sleep(0.3)
            assert fake.got['stop0']; ok('release_all liberou e mandou stop')

        async with Sess(app) as A:
            await A.send_json({'type': 'hello', 'token': 'tokA'})
            await A.send_json({'type': 'select_robot', 'id': 1})
            await recv_until(A, lambda d: d['type'] == 'robots' and robots(d)[1] == 'mine')
        await asyncio.sleep(0.5)
        assert len(fake.got['stop1']) > 0; ok('queda de A: stop imediato no robô 1')
        async with Sess(app) as B:
            await B.send_json({'type': 'hello', 'token': 'tokB'})
            d, _ = await recv_until(B, lambda d: d['type'] == 'robots' and len(d['robots']) == 3)
            assert robots(d)[1] == 'busy'; ok('robô 1 segue reservado durante a tolerância')
            await recv_until(B, lambda d: d['type'] == 'robots' and robots(d)[1] == 'free', timeout=6)
            ok('após a tolerância (2 s no teste) o robô 1 foi liberado')

        async with Sess(app) as A:
            await A.send_json({'type': 'hello', 'token': 'tokC'})
            await A.send_json({'type': 'select_robot', 'id': 1})
            await recv_until(A, lambda d: d['type'] == 'robots' and robots(d)[1] == 'mine')
        async with Sess(app) as A2:
            await A2.send_json({'type': 'hello', 'token': 'tokC'})
            await recv_until(A2, lambda d: d['type'] == 'robots' and robots(d).get(1) == 'mine')
            await asyncio.sleep(3)
            await A2.send_json({'type': 'goal', 'x': 5, 'y': 5})
            d, _ = await recv_until(A2, lambda d: d['type'] in ('goal_ack', 'error'))
            assert d['type'] == 'goal_ack'; ok('mesmo token voltou a tempo e retomou o robô 1')

        async with Sess(app) as A:
            await A.send_json({'type': 'hello', 'token': 'tokD'})
            await A.send_json({'type': 'select_robot', 'id': 0})
            await recv_until(A, lambda d: d['type'] == 'robots' and robots(d)[0] == 'mine')
            fake.publish_state(bt0=False)
            await recv_until(A, lambda d: d['type'] == 'robots' and robots(d)[0] == 'free' and not d['robots'][0]['bluetooth'])
            ok('robô sem Bluetooth é liberado e aparece indisponível')
            fake.ev_pub.publish(String(data=json.dumps({'event': 'robot_connected', 'message': 'Robô X conectado.'})))
            d, _ = await recv_until(A, lambda d: d['type'] == 'bt_event')
            ok(f'evento de Bluetooth chega ao front: {d["message"]}')
    print('TUDO OK')
    sys.stdout.flush()
    os._exit(0)  # evita o core dump do rclpy ao encerrar as threads

asyncio.run(main())
