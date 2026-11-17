"""Nó ponte entre o sistema ROS 2 do Zênite e o backend web (FastAPI).

O rclpy roda em uma thread própria (ver main.py); o servidor asyncio consome
os últimos dados (frame JPEG e pose) de forma thread-safe. Cada dado carrega
um número de sequência para o servidor detectar novidade sem fila crescer:
frames antigos são naturalmente descartados (sempre se envia só o mais novo).

Tópicos (para cada robô <id> de 0 a MAX_ROBOTS-1):
  assina  camera_frame               (sensor_msgs/Image)          -> stream p/ web
  assina  /robot_<id>/current_pose   (std_msgs/Float64MultiArray) -> telemetria
                                     [x, y, theta], do localization_node
  publica /robot_<id>/desired_position (geometry_msgs/Point)      <- clique
  publica /robot_<id>/stop           (std_msgs/Bool)              <- parada
  publica image_params               (image_adjust_msgs/ImageParams) <- sliders
Bluetooth (bluetooth_node):
  assina  /bluetooth/state           (std_msgs/String, JSON)  -> lista de robôs
  assina  /bluetooth/events          (std_msgs/String, JSON)  -> avisos
  publica /bluetooth/command         (std_msgs/String, JSON)  <- buscar/conectar
"""

import json
import threading
import time
from collections import deque

import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from std_msgs.msg import Bool, Float64MultiArray, String
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from image_adjust_msgs.msg import ImageParams

from .pixel_converter import PixelConverter


MAX_ROBOTS = 8
MARKER_TIMEOUT_S = 1.0  # sem pose por mais que isso = marcador fora de vista


class RosBridgeNode(Node):
    # Mesma arena hardcoded do calibration_node (C++): 2,75 x 2,25 m.
    # Ordem esperada dos cliques: (0,0) -> (2.75,0) -> (2.75,2.25) -> (0,2.25).
    CALIBRATION_WORLD_POINTS = [
        (0.0, 0.0), (2.75, 0.0), (2.75, 2.25), (0.0, 2.25),
    ]

    def __init__(self):
        super().__init__('web_interface_node')

        self.scale_yaml_path = self.declare_parameter(
            'scale_yaml_path', '/tmp/scale.yaml').value
        self.jpeg_quality = int(self.declare_parameter('jpeg_quality', 70).value)
        self.host = self.declare_parameter('host', '0.0.0.0').value
        self.port = int(self.declare_parameter('port', 8000).value)
        # Tempo (s) que o robô segue reservado ao navegador depois que ele cai.
        self.reservation_grace_s = float(
            self.declare_parameter('reservation_grace_s', 30.0).value)

        self.converter = PixelConverter()
        self.try_load_homography()

        self._lock = threading.Lock()
        self._latest_jpeg = None
        self._frame_seq = 0
        # Pose mais recente de cada robô: id -> dict / seq / instante.
        self._poses = {}
        self._pose_seqs = {}
        self._pose_times = {}

        # Estado vindo do bluetooth_node (None = o nó não está rodando).
        self._bt_state = None
        self._bt_version = 0
        self._bt_events = deque(maxlen=50)

        self.create_subscription(
            Image, 'camera_frame', self._image_callback, 10)

        self._goal_pubs = {}
        self._stop_pubs = {}
        for robot_id in range(MAX_ROBOTS):
            self.create_subscription(
                Float64MultiArray, f'/robot_{robot_id}/current_pose',
                lambda msg, rid=robot_id: self._pose_callback(rid, msg), 10)
            self._goal_pubs[robot_id] = self.create_publisher(
                Point, f'/robot_{robot_id}/desired_position', 10)
            self._stop_pubs[robot_id] = self.create_publisher(
                Bool, f'/robot_{robot_id}/stop', 10)

        self._params_pub = self.create_publisher(ImageParams, 'image_params', 10)

        latched = QoSProfile(depth=1,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(
            String, '/bluetooth/state', self._bt_state_callback, latched)
        self.create_subscription(
            String, '/bluetooth/events', self._bt_event_callback, 10)
        self._bt_cmd_pub = self.create_publisher(
            String, '/bluetooth/command', 10)

        self.get_logger().info('web_interface_node iniciado!')

    # ------------------------------------------------------------------ #
    # Homografia
    # ------------------------------------------------------------------ #

    def try_load_homography(self) -> bool:
        try:
            self.converter.load_from_yaml(self.scale_yaml_path)
            self.get_logger().info(
                f'Homografia carregada de {self.scale_yaml_path}')
            return True
        except Exception as e:
            self.get_logger().warn(
                f'Homografia não carregada ({self.scale_yaml_path}): {e}. '
                'Rode o calibration_node e recarregue pela interface web.')
            return False

    # ------------------------------------------------------------------ #
    # Callbacks ROS (thread do rclpy)
    # ------------------------------------------------------------------ #

    def _image_callback(self, msg: Image):
        frame = self._image_msg_to_bgr(msg)
        if frame is None:
            return

        ok, jpeg = cv2.imencode(
            '.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality])
        if not ok:
            return

        with self._lock:
            self._latest_jpeg = jpeg.tobytes()
            self._frame_seq += 1

    def _image_msg_to_bgr(self, msg: Image):
        try:
            data = np.frombuffer(msg.data, dtype=np.uint8)
            if msg.encoding in ('bgr8', 'rgb8'):
                frame = data.reshape((msg.height, msg.step // 3, 3))
                frame = frame[:, :msg.width, :]
                if msg.encoding == 'rgb8':
                    frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                return frame
            if msg.encoding == 'mono8':
                frame = data.reshape((msg.height, msg.step))[:, :msg.width]
                return cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
            self.get_logger().warn(
                f'Encoding de imagem não suportado: {msg.encoding}',
                throttle_duration_sec=5.0)
            return None
        except Exception as e:
            self.get_logger().error(f'Erro ao converter imagem: {e}')
            return None

    def _pose_callback(self, robot_id: int, msg: Float64MultiArray):
        if len(msg.data) < 2:
            return

        x_m, y_m = float(msg.data[0]), float(msg.data[1])
        pose = {'robot_id': robot_id, 'x': x_m, 'y': y_m}
        if len(msg.data) >= 3:
            pose['theta'] = float(msg.data[2])

        if self.converter.ready:
            px, py = self.converter.meter_to_pixel(x_m, y_m)
            pose['px'] = px
            pose['py'] = py

        with self._lock:
            self._poses[robot_id] = pose
            self._pose_seqs[robot_id] = self._pose_seqs.get(robot_id, 0) + 1
            self._pose_times[robot_id] = time.monotonic()

    def _bt_state_callback(self, msg: String):
        try:
            state = json.loads(msg.data)
        except ValueError:
            return
        with self._lock:
            self._bt_state = state
            self._bt_version += 1

    def _bt_event_callback(self, msg: String):
        try:
            evento = json.loads(msg.data)
        except ValueError:
            return
        with self._lock:
            self._bt_events.append(evento)

    # ------------------------------------------------------------------ #
    # API consumida pelo servidor web (thread do asyncio)
    # ------------------------------------------------------------------ #

    def get_frame(self, last_seq: int):
        """Retorna (jpeg_bytes, seq) se houver frame mais novo que last_seq."""
        with self._lock:
            if self._latest_jpeg is None or self._frame_seq == last_seq:
                return None, last_seq
            return self._latest_jpeg, self._frame_seq

    def get_pose(self, robot_id: int, last_seq: int):
        """Retorna (pose_dict, seq) se o robô tiver pose mais nova que last_seq."""
        with self._lock:
            seq = self._pose_seqs.get(robot_id, 0)
            if seq == 0 or seq == last_seq:
                return None, last_seq
            return dict(self._poses[robot_id]), seq

    def visible_markers(self) -> set:
        """IDs de robôs cujo marcador foi visto há pouco (pose recente)."""
        limite = time.monotonic() - MARKER_TIMEOUT_S
        with self._lock:
            return {i for i, t in self._pose_times.items() if t >= limite}

    # --- Bluetooth ---

    def has_bluetooth(self) -> bool:
        with self._lock:
            return self._bt_state is not None

    def get_bluetooth(self):
        """Retorna (estado_ou_None, versão). A versão muda a cada novo estado."""
        with self._lock:
            return self._bt_state, self._bt_version

    def pop_bluetooth_events(self) -> list:
        with self._lock:
            eventos = list(self._bt_events)
            self._bt_events.clear()
        return eventos

    def bluetooth_command(self, comando: dict) -> None:
        self._bt_cmd_pub.publish(String(data=json.dumps(comando)))

    def publish_stop(self, robot_id: int) -> None:
        """Manda o robô parar (cancela o alvo no control_node)."""
        pub = self._stop_pubs.get(robot_id)
        if pub is not None:
            pub.publish(Bool(data=True))

    def publish_goal(self, robot_id: int, px: float, py: float) -> dict:
        """Converte clique (pixel) para metros e publica o alvo do robô."""
        if not self.converter.ready:
            raise RuntimeError(
                'Homografia não carregada. Rode o calibration_node primeiro.')

        x_m, y_m = self.converter.pixel_to_meter(px, py)

        msg = Point()
        msg.x = x_m
        msg.y = y_m
        msg.z = 0.0
        pub = self._goal_pubs.get(robot_id)
        if pub is None:
            raise RuntimeError(f'Robô {robot_id} inválido.')
        pub.publish(msg)

        self.get_logger().info(
            f'Robô {robot_id}: clique ({px:.1f}, {py:.1f}) -> '
            f'Mundo ({x_m:.2f}, {y_m:.2f})')
        return {'px': px, 'py': py, 'x': x_m, 'y': y_m}

    def calibrate(self, pixel_points) -> None:
        """Calibra a homografia a partir de 4 pontos clicados na imagem.

        Equivalente ao calibration_node (C++): assume a mesma arena de
        2,75 x 2,25 m e a mesma ordem de cliques nos cantos.
        """
        if len(pixel_points) != 4:
            raise ValueError('É necessário enviar exatamente 4 pontos')

        self.converter.set_reference_points(pixel_points, self.CALIBRATION_WORLD_POINTS)
        self.converter.save_to_yaml(self.scale_yaml_path)

        self.get_logger().info(
            f'Calibração concluída via interface web. Homografia salva em {self.scale_yaml_path}')

    def publish_params(self, brightness: float, saturation: float, hue: float):
        """Publica parâmetros de imagem normalizados (0.0 a 1.0)."""
        msg = ImageParams()
        msg.brightness = float(np.clip(brightness, 0.0, 1.0))
        msg.saturation = float(np.clip(saturation, 0.0, 1.0))
        msg.hue = float(np.clip(hue, 0.0, 1.0))
        self._params_pub.publish(msg)
