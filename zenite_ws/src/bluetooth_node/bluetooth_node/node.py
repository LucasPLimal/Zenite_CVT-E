"""Nó ROS 2 do gerenciador de Bluetooth.

Substitui o transmission_node + `rfcomm bind` manual: o servidor procura,
pareia e conecta os robôs sozinho e repassa as velocidades das rodas.

Tópicos:
  assina  /robot_<id>/wheel_velocities (geometry_msgs/Vector3)  x=dir., y=esq.
          -> vira o pacote de 3 bytes enviado ao ESP32 do robô <id>
  assina  /robot_<id>/stop            (std_msgs/Bool)
          -> envia parada direto ao robô (redundante com o control_node)
  assina  /bluetooth/command          (std_msgs/String, JSON)
          {"action": "scan"} | {"action": "connect", "mac": "AA:..."}
          | {"action": "disconnect", "id": 0}
  publica /bluetooth/state            (std_msgs/String, JSON, transient_local)
          {"scanning": bool, "devices": [...], "robots": [...]}
  publica /bluetooth/events           (std_msgs/String, JSON)
          avisos pontuais ("robô conectado", "não é um robô" etc.)
"""

import json
import os
import time

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Vector3
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String

from .manager import BluetoothManager
from .registry import KnownStore, Registry


class BluetoothNode(Node):
    def __init__(self):
        super().__init__('bluetooth_node')

        padrao = os.path.join(
            get_package_share_directory('bluetooth_node'),
            'config', 'robots.yaml')
        robots_file = self.declare_parameter('robots_file', padrao).value
        known_file = self.declare_parameter(
            'known_file', '~/.zenite/bluetooth_known.json').value
        self.min_period = 1.0 / float(
            self.declare_parameter('max_send_hz', 30.0).value)
        auto_reconnect = bool(
            self.declare_parameter('auto_reconnect', True).value)
        reconnect_s = float(self.declare_parameter('reconnect_s', 10.0).value)

        self.registry = Registry.from_yaml(robots_file)
        self.manager = BluetoothManager(
            self.registry, KnownStore(known_file),
            reconnect_s=reconnect_s,
            on_change=self._publish_state,
            on_event=self._publish_event)

        latched = QoSProfile(depth=1,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self._state_pub = self.create_publisher(String, '/bluetooth/state', latched)
        self._event_pub = self.create_publisher(String, '/bluetooth/events', 10)
        self.create_subscription(
            String, '/bluetooth/command', self._command_callback, 10)

        self._last_sent = {}
        for spec in self.registry.robots:
            self.create_subscription(
                Vector3, f'/robot_{spec.id}/wheel_velocities',
                lambda msg, rid=spec.id: self._velocity_callback(rid, msg), 10)
            self.create_subscription(
                Bool, f'/robot_{spec.id}/stop',
                lambda msg, rid=spec.id: self.manager.send_stop(rid), 10)

        self._publish_state()
        if auto_reconnect:
            self.manager.start()

        self.get_logger().info(
            'bluetooth_node iniciado com os robôs: '
            + ', '.join(f'{r.id}={r.name}' for r in self.registry.robots))

    # ------------------------------------------------------------------ #

    def _velocity_callback(self, robot_id: int, msg: Vector3):
        parado = msg.x == 0.0 and msg.y == 0.0
        agora = time.monotonic()
        if not parado and agora - self._last_sent.get(robot_id, 0.0) < self.min_period:
            return  # limita a taxa; a parada nunca é descartada
        self._last_sent[robot_id] = agora
        self.manager.send_velocity(robot_id, msg.x, msg.y)

    def _command_callback(self, msg: String):
        try:
            cmd = json.loads(msg.data)
            acao = cmd.get('action')
            if acao == 'scan':
                self.manager.scan_async()
            elif acao == 'connect':
                self.manager.connect_async(str(cmd['mac']))
            elif acao == 'disconnect':
                self.manager.disconnect(int(cmd['id']))
            else:
                self.get_logger().warn(f'Comando desconhecido: {msg.data}')
        except (ValueError, KeyError, TypeError) as e:
            self.get_logger().warn(f'Comando inválido ({e}): {msg.data}')

    def _publish_state(self):
        self._state_pub.publish(
            String(data=json.dumps(self.manager.state(), ensure_ascii=False)))

    def _publish_event(self, evento: dict):
        self.get_logger().info(evento.get('message', str(evento)))
        self._event_pub.publish(
            String(data=json.dumps(evento, ensure_ascii=False)))

    def shutdown(self):
        self.manager.stop()


def main(args=None):
    rclpy.init(args=args)
    node = BluetoothNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.shutdown()  # manda parada a todos os robôs antes de sair
        try:
            node.destroy_node()
        except Exception:
            pass
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
