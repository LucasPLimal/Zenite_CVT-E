"""Nó ponte entre o sistema ROS 2 do Zênite e o backend web (FastAPI).

O rclpy roda em uma thread própria (ver main.py); o servidor asyncio consome
os últimos dados (frame JPEG e pose) de forma thread-safe. Cada dado carrega
um número de sequência para o servidor detectar novidade sem fila crescer:
frames antigos são naturalmente descartados (sempre se envia só o mais novo).

Tópicos:
  assina  camera_frame        (sensor_msgs/Image)          -> stream p/ web
  assina  /current_position   (std_msgs/Float64MultiArray) -> telemetria p/ web
                                                                (também publicado
                                                                por este nó quando
                                                                o rastreamento via
                                                                web está ativo)
  publica /desired_position   (geometry_msgs/Point)        <- clique do usuário
  publica /current_position   (std_msgs/Float64MultiArray) <- rastreamento CSRT
                                                                ("Localizar robô")
  publica image_params        (image_adjust_msgs/ImageParams) <- sliders

O rastreamento ("Localizar robô") equivale ao localization_node (C++): mesmo
algoritmo (cv2.TrackerCSRT), mesmo cálculo de centro e mesmo cooldown de
publicação (50ms). A diferença é a inicialização: em vez de cv::selectROI
(janela OpenCV local), o retângulo inicial vem da seleção por marquee feita
no navegador.
"""

import threading

import numpy as np
import cv2
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from std_msgs.msg import Float64MultiArray
from image_adjust_msgs.msg import ImageParams

from .pixel_converter import PixelConverter


def _create_csrt_tracker():
    """Cria um cv2.TrackerCSRT, cobrindo as duas localizações possíveis da API
    conforme a versão do opencv-contrib-python (legacy.* a partir da 4.5.1)."""
    if hasattr(cv2, 'legacy') and hasattr(cv2.legacy, 'TrackerCSRT_create'):
        return cv2.legacy.TrackerCSRT_create()
    if hasattr(cv2, 'TrackerCSRT_create'):
        return cv2.TrackerCSRT_create()
    raise RuntimeError(
        'cv2.TrackerCSRT não disponível. Verifique se o opencv-contrib-python '
        'está instalado (não o opencv-python) — veja o README.')


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

        self.converter = PixelConverter()
        self.try_load_homography()

        self._lock = threading.Lock()
        self._latest_jpeg = None
        self._frame_seq = 0
        self._latest_pose = None
        self._pose_seq = 0
        self._latest_frame = None  # frame BGR bruto, para inicializar o tracker

        # Rastreamento CSRT ("Localizar robô") — só acessado na thread do rclpy,
        # exceto a inicialização em locate_robot() (thread do asyncio via WS).
        self._tracker = None
        self._tracking = False
        self._last_track_publish = self.get_clock().now()
        self._track_cooldown = Duration(seconds=0.05)  # mesmo cooldown do localization_node

        self.create_subscription(
            Image, 'camera_frame', self._image_callback, 10)
        self.create_subscription(
            Float64MultiArray, '/current_position', self._pose_callback, 10)

        self._goal_pub = self.create_publisher(Point, '/desired_position', 10)
        self._params_pub = self.create_publisher(ImageParams, 'image_params', 10)
        self._pos_pub = self.create_publisher(Float64MultiArray, '/current_position', 10)

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

        with self._lock:
            self._latest_frame = frame

        self._update_tracking(frame)

        ok, jpeg = cv2.imencode(
            '.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality])
        if not ok:
            return

        with self._lock:
            self._latest_jpeg = jpeg.tobytes()
            self._frame_seq += 1

    def _update_tracking(self, frame):
        """Equivalente ao loop principal do localization_node (C++):
        tracker->update() a cada frame, converte o centro do ROI para metros
        e publica em /current_position respeitando o cooldown."""
        if not self._tracking or self._tracker is None:
            return

        success, roi = self._tracker.update(frame)
        if not success:
            # Mesmo comportamento do C++: só pula a publicação nesse frame,
            # não desliga o rastreamento (pode se recuperar no próximo frame).
            return

        x, y, w, h = roi
        cx, cy = x + w / 2.0, y + h / 2.0

        if not self.converter.ready:
            return

        now = self.get_clock().now()
        if now - self._last_track_publish < self._track_cooldown:
            return
        self._last_track_publish = now

        x_m, y_m = self.converter.pixel_to_meter(cx, cy)

        pos_msg = Float64MultiArray()
        pos_msg.data = [x_m, y_m]
        self._pos_pub.publish(pos_msg)

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

    def locate_robot(self, x: float, y: float, width: float, height: float) -> None:
        """Inicializa o rastreamento CSRT a partir do retângulo selecionado no navegador.

        Equivalente ao cv::selectROI + tracker->init() do localization_node (C++),
        usando o frame mais recente da câmera em vez de uma janela local.
        """
        if width <= 0 or height <= 0:
            raise ValueError('Seleção inválida: largura e altura precisam ser maiores que zero')

        with self._lock:
            frame = self._latest_frame

        if frame is None:
            raise RuntimeError('Nenhum frame de câmera disponível ainda')

        tracker = _create_csrt_tracker()
        tracker.init(frame, (float(x), float(y), float(width), float(height)))

        self._tracker = tracker
        self._tracking = True

        self.get_logger().info(
            f'Rastreamento do robô iniciado via interface web: '
            f'ROI=({x:.0f}, {y:.0f}, {width:.0f}, {height:.0f})')

    def reset_tracking(self) -> None:
        self._tracker = None
        self._tracking = False
        self.get_logger().info('Rastreamento do robô resetado.')

    def publish_params(self, brightness: float, saturation: float, hue: float):
        """Publica parâmetros de imagem normalizados (0.0 a 1.0)."""
        msg = ImageParams()
        msg.brightness = float(np.clip(brightness, 0.0, 1.0))
        msg.saturation = float(np.clip(saturation, 0.0, 1.0))
        msg.hue = float(np.clip(hue, 0.0, 1.0))
        self._params_pub.publish(msg)
