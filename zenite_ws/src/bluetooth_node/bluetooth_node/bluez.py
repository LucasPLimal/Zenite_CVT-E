"""Acesso ao Bluetooth do Linux: busca e pareamento via `bluetoothctl` (BlueZ)
e conexão RFCOMM por socket (sem `rfcomm bind`, sem root e sem /dev/rfcomm).

Nada aqui depende de ROS. Não foi possível testar com hardware no ambiente de
desenvolvimento: valide no Raspberry Pi com tools/bluetooth_poc.py.
"""

import re
import socket
import subprocess
import threading
import time
from typing import Dict

ANSI = re.compile(r'\x1b\[[0-9;?]*[A-Za-z]|\r')
DEVICE_LINE = re.compile(
    r'Device\s+((?:[0-9A-Fa-f]{2}:){5}[0-9A-Fa-f]{2})\s*(?:Name:\s*)?(.*)$')

RFCOMM_CHANNEL = 1  # canal do serviço SPP do ESP32 (padrão do BluetoothSerial)


def limpar(texto: str) -> str:
    return ANSI.sub('', texto)


def parse_dispositivos(texto: str) -> Dict[str, str]:
    """Extrai {MAC: nome} da saída do bluetoothctl."""
    achados: Dict[str, str] = {}
    for linha in texto.splitlines():
        linha = linha.strip()
        # Linhas [CHG] só valem quando trazem o nome (ignora RSSI, UUIDs etc.).
        if linha.startswith('[CHG]') and 'Name:' not in linha:
            continue
        m = DEVICE_LINE.search(linha)
        if not m:
            continue
        mac, nome = m.group(1).upper(), m.group(2).strip()
        # Sem nome o BlueZ repete o próprio MAC (com '-' no lugar de ':').
        if nome and nome.replace('-', ':').upper() != mac:
            achados[mac] = nome
        else:
            achados.setdefault(mac, '')
    return achados


def _bluetoothctl(*args, timeout=15) -> str:
    try:
        r = subprocess.run(['bluetoothctl', *args], capture_output=True,
                           text=True, timeout=timeout)
        return limpar(r.stdout + r.stderr)
    except subprocess.TimeoutExpired as e:
        saida = e.stdout or ''
        if isinstance(saida, bytes):
            saida = saida.decode(errors='replace')
        return limpar(saida)
    except FileNotFoundError:
        raise RuntimeError('bluetoothctl não encontrado (instale o pacote bluez)')


def buscar(segundos: int = 8) -> Dict[str, str]:
    """Procura dispositivos por `segundos` e devolve {MAC: nome}.

    Atenção: a busca degrada as conexões Bluetooth ativas; só faça sob demanda.
    """
    achados = parse_dispositivos(_bluetoothctl('devices'))
    saida = _bluetoothctl('--timeout', str(segundos), 'scan', 'on',
                          timeout=segundos + 10)
    achados.update({m: n for m, n in parse_dispositivos(saida).items() if n})
    return achados


def esta_pareado(mac: str) -> bool:
    return 'Paired: yes' in _bluetoothctl('info', mac)


def parear(mac: str, limite: float = 40.0) -> bool:
    """Pareia sem PIN (agente NoInputNoOutput) e marca como confiável."""
    p = subprocess.Popen(['bluetoothctl'], stdin=subprocess.PIPE,
                         stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                         text=True, bufsize=1)
    log = []

    def ler():
        for linha in p.stdout:
            log.append(limpar(linha))

    threading.Thread(target=ler, daemon=True).start()

    def enviar(cmd, espera=1.0):
        p.stdin.write(cmd + '\n')
        p.stdin.flush()
        time.sleep(espera)

    def pareado():
        texto = ''.join(log)
        return ('Pairing successful' in texto or 'AlreadyExists' in texto
                or 'Paired: yes' in texto)

    try:
        enviar('agent NoInputNoOutput')
        enviar('default-agent')
        enviar('pairable on')
        enviar(f'pair {mac}', espera=0)
        t0 = time.time()
        while time.time() - t0 < limite:
            if pareado() or 'Failed to pair' in ''.join(log):
                break
            time.sleep(0.5)
        ok = pareado()
        if ok:
            enviar(f'trust {mac}')
        enviar('quit', espera=0.5)
        return ok
    finally:
        try:
            p.terminate()
        except OSError:
            pass


def esquecer(mac: str):
    """Remove o pareamento (usado quando o dispositivo não é um robô)."""
    try:
        _bluetoothctl('remove', mac, timeout=10)
    except (RuntimeError, OSError):
        pass


def conectar_rfcomm(mac: str, canal: int = RFCOMM_CHANNEL, timeout: float = 8.0):
    """Abre um socket RFCOMM com o robô e o devolve."""
    s = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM,
                      socket.BTPROTO_RFCOMM)
    s.settimeout(timeout)
    try:
        s.connect((mac, canal))
    except OSError:
        s.close()
        raise
    return s
