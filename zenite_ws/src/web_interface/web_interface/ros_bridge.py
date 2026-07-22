"""Nó ponte entre o sistema ROS 2 do Zênite e o backend web (FastAPI).

O rclpy roda em uma thread própria (ver main.py); o servidor asyncio consome
os últimos dados (frame JPEG e pose) de forma thread-safe. Cada dado carrega
um número de sequência para o servidor detectar novidade sem fila crescer:
frames antigos são naturalmente descartados (sempre se envia só o mais novo).

Tópicos:
  assina  camera_frame        (sensor_msgs/Image)          -> stream p/ web
  assina  /current_position   (std_msgs/Float64MultiArray) -> telemetria p/ web
  publica /desired_position   (geometry_msgs/Point)        <- clique do usuário
  publica image_params        (image_adjust_msgs/ImageParams) <- sliders
"""

import threading

import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from std_msgs.msg import Float64MultiArray
from image_adjust_msgs.msg import ImageParams

from .pixel_converter import PixelConverter


class RosBridgeNode(Node):
    def __init__(self):
        super().__init__('web_interface_node')

        self.scale_yaml_path = self.declare_parameter(
            'scale_yaml_path', '/tmp/scale.yaml').value
        self.jpeg_quality = int(self.declare_parameter('jpeg_quality', 70).value)
        self.host = self.declare_parameter('host', '0.0.0.0').value
        self.port = int(self.declare_parameter('port', 8000).value)

        self.converter = PixelConverter()
        self.try_load_homography()

        self._lock = threading.Lock()
        self._latest_jpeg = None
        self._frame_seq = 0
        self._latest_pose = None
        self._pose_seq = 0

        self.create_subscription(
            Image, 'camera_frame', self._image_callback, 10)
        self.create_subscription(
            Float64MultiArray, '/current_position', self._pose_callback, 10)

        self._goal_pub = self.create_publisher(Point, '/desired_position', 10)
        self._params_pub = self.create_publisher(ImageParams, 'image_params', 10)

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

    def _pose_callback(self, msg: Float64MultiArray):
        if len(msg.data) < 2:
            return

        x_m, y_m = float(msg.data[0]), float(msg.data[1])
        pose = {'x': x_m, 'y': y_m}

        if self.converter.ready:
            px, py = self.converter.meter_to_pixel(x_m, y_m)
            pose['px'] = px
            pose['py'] = py

        with self._lock:
            self._latest_pose = pose
            self._pose_seq += 1

    # ------------------------------------------------------------------ #
    # API consumida pelo servidor web (thread do asyncio)
    # ------------------------------------------------------------------ #

    def get_frame(self, last_seq: int):
        """Retorna (jpeg_bytes, seq) se houver frame mais novo que last_seq."""
        with self._lock:
            if self._latest_jpeg is None or self._frame_seq == last_seq:
                return None, last_seq
            return self._latest_jpeg, self._frame_seq

    def get_pose(self, last_seq: int):
        """Retorna (pose_dict, seq) se houver pose mais nova que last_seq."""
        with self._lock:
            if self._latest_pose is None or self._pose_seq == last_seq:
                return None, last_seq
            return dict(self._latest_pose), self._pose_seq

    def publish_goal(self, px: float, py: float) -> dict:
        """Converte clique (pixel) para metros e publica em /desired_position."""
        if not self.converter.ready:
            raise RuntimeError(
                'Homografia não carregada. Rode o calibration_node primeiro.')

        x_m, y_m = self.converter.pixel_to_meter(px, py)

        msg = Point()
        msg.x = x_m
        msg.y = y_m
        msg.z = 0.0
        self._goal_pub.publish(msg)

        self.get_logger().info(
            f'Clique na imagem: ({px:.1f}, {py:.1f}) -> Mundo: ({x_m:.2f}, {y_m:.2f})')
        return {'px': px, 'py': py, 'x': x_m, 'y': y_m}

    def publish_params(self, brightness: float, saturation: float, hue: float):
        """Publica parâmetros de imagem normalizados (0.0 a 1.0)."""
        msg = ImageParams()
        msg.brightness = float(np.clip(brightness, 0.0, 1.0))
        msg.saturation = float(np.clip(saturation, 0.0, 1.0))
        msg.hue = float(np.clip(hue, 0.0, 1.0))
        self._params_pub.publish(msg)
