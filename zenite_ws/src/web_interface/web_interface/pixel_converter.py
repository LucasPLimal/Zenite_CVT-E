"""Conversão pixel <-> metro via homografia.

Versão Python do zenite_utils::PixelConverter (C++). Lê o mesmo arquivo
scale.yaml gerado pelo calibration_node (pixel_points / real_points, 4 pontos
cada) e recalcula a homografia com cv2.findHomography, exatamente como o C++.
"""

import numpy as np
import cv2
import yaml


class PixelConverter:
    def __init__(self):
        self.H = None
        self.H_inv = None

    @property
    def ready(self) -> bool:
        return self.H is not None

    def load_from_yaml(self, path: str) -> None:
        with open(path, 'r') as f:
            data = yaml.safe_load(f)

        pixel_points = np.array(data['pixel_points'], dtype=np.float32)
        real_points = np.array(data['real_points'], dtype=np.float32)

        if pixel_points.shape != (4, 2) or real_points.shape != (4, 2):
            raise ValueError('scale.yaml precisa conter 4 pontos de referência (pixel_points e real_points)')

        H, _ = cv2.findHomography(pixel_points, real_points)
        if H is None:
            raise ValueError('Não foi possível calcular a homografia a partir dos pontos fornecidos')

        self.H = H
        self.H_inv = np.linalg.inv(H)

    def _transform(self, x: float, y: float, matrix) -> tuple:
        src = np.array([[[x, y]]], dtype=np.float32)
        dst = cv2.perspectiveTransform(src, matrix)
        return float(dst[0, 0, 0]), float(dst[0, 0, 1])

    def pixel_to_meter(self, x: float, y: float) -> tuple:
        if not self.ready:
            raise RuntimeError('Homografia não carregada')
        return self._transform(x, y, self.H)

    def meter_to_pixel(self, x: float, y: float) -> tuple:
        if not self.ready:
            raise RuntimeError('Homografia não carregada')
        return self._transform(x, y, self.H_inv)
