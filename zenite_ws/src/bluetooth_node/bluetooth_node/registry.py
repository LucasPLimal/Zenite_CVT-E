"""Cadastro de robôs (robots.yaml) e MACs já descobertos."""

import json
import os
from dataclasses import dataclass
from typing import Dict, Optional

import yaml

MAX_ROBOTS = 8


@dataclass(frozen=True)
class RobotSpec:
    id: int
    name: str    # nome Bluetooth / apresentação do ESP32
    label: str   # nome mostrado na interface


class Registry:
    def __init__(self, robots):
        robots = list(robots)
        if len(robots) > MAX_ROBOTS:
            raise ValueError(f'O cadastro aceita no máximo {MAX_ROBOTS} robôs.')
        ids = [r.id for r in robots]
        nomes = [r.name for r in robots]
        if len(set(ids)) != len(ids):
            raise ValueError('IDs de robô repetidos no cadastro.')
        if len(set(nomes)) != len(nomes):
            raise ValueError('Nomes de robô repetidos no cadastro.')
        self.robots = sorted(robots, key=lambda r: r.id)
        self._by_name = {r.name: r for r in self.robots}
        self._by_id = {r.id: r for r in self.robots}

    @classmethod
    def from_yaml(cls, path: str) -> 'Registry':
        with open(path, encoding='utf-8') as f:
            dados = yaml.safe_load(f) or {}
        robots = [RobotSpec(id=int(r['id']), name=str(r['name']),
                            label=str(r.get('label', r['name'])))
                  for r in dados.get('robots', [])]
        return cls(robots)

    def by_name(self, name: str) -> Optional[RobotSpec]:
        return self._by_name.get(name)

    def by_id(self, robot_id: int) -> Optional[RobotSpec]:
        return self._by_id.get(robot_id)


class KnownStore:
    """Guarda {nome do robô: MAC} em um JSON, para reconectar sem procurar."""

    def __init__(self, path: str):
        self.path = os.path.expanduser(path)
        self._data: Dict[str, str] = {}
        try:
            with open(self.path, encoding='utf-8') as f:
                self._data = {str(k): str(v) for k, v in json.load(f).items()}
        except (OSError, ValueError):
            pass

    def get(self, name: str) -> Optional[str]:
        return self._data.get(name)

    def items(self):
        return dict(self._data)

    def set(self, name: str, mac: str):
        if self._data.get(name) == mac:
            return
        self._data[name] = mac
        os.makedirs(os.path.dirname(self.path), exist_ok=True)
        with open(self.path, 'w', encoding='utf-8') as f:
            json.dump(self._data, f, indent=2)
