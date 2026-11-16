#!/usr/bin/env python3
"""Prova de conceito: conexão Bluetooth automática com os robôs (ESP32).

Roda no Raspberry Pi 5 (precisa do BlueZ e do comando `bluetoothctl`).
Não depende de ROS nem de pacotes extras: só da biblioteca padrão do Python.

Objetivo: responder, antes de construir o gerenciador de Bluetooth do servidor,
 1. o Pi enxerga o robô pelo nome e consegue parear sem digitar PIN?
 2. o robô envia a apresentação "ZENIT,<nome>" ao conectar?
 3. o Pi aguenta 3 robôs conectados ao mesmo tempo, enviando pacotes?

Etapas (rode nesta ordem, com os robôs ligados e o firmware novo gravado):

  python3 tools/bluetooth_poc.py scan
      Lista os dispositivos Bluetooth vistos em ~10 s (nome e endereço).

  python3 tools/bluetooth_poc.py pair zenitinho parnadiloJR Jerimum
      Procura cada nome, pareia sem PIN e marca como confiável.
      Imprime o endereço (MAC) descoberto de cada um.

  python3 tools/bluetooth_poc.py run zenitinho parnadiloJR Jerimum --segundos 60
      Conecta aos robôs já pareados (por nome), espera a apresentação e envia
      pacotes de 3 bytes [dir, PWM_E, PWM_D] a 20 Hz para todos ao mesmo tempo.
      Por padrão os PWM são 0 (robô parado): é seguro. Mostra estatísticas.

Perda de pacotes: o firmware não confirma o recebimento, então a perda real é
vista no Monitor Serial do ESP32 (linhas "RX DIR=..." e "LastPkt(ms)"). Este
script mede o lado do Pi: erros de envio, quedas de conexão e atraso entre
envios.
"""
import argparse
import re
import socket
import subprocess
import sys
import threading
import time

ANSI = re.compile(r'\x1b\[[0-9;?]*[A-Za-z]|\r')
DEVICE_LINE = re.compile(
    r'Device\s+((?:[0-9A-Fa-f]{2}:){5}[0-9A-Fa-f]{2})\s*(?:Name:\s*)?(.*)$')
MAC_RE = re.compile(r'^(?:[0-9A-Fa-f]{2}:){5}[0-9A-Fa-f]{2}$')

HELLO_PREFIX = 'ZENIT,'
HELLO_TIMEOUT = 5.0   # s para o robô enviar a apresentação
SEND_HZ = 20          # taxa de envio de pacotes por robô
RFCOMM_CHANNEL = 1    # canal do serviço SPP do ESP32 (padrão)


def limpar(texto):
    return ANSI.sub('', texto)


def bluetoothctl(*args, timeout=15):
    """Executa um comando único do bluetoothctl e devolve a saída limpa."""
    try:
        r = subprocess.run(['bluetoothctl', *args], capture_output=True,
                           text=True, timeout=timeout)
    except FileNotFoundError:
        sys.exit('bluetoothctl não encontrado. Instale: sudo apt install bluez')
    except subprocess.TimeoutExpired as e:
        return limpar((e.stdout or b'').decode() if isinstance(e.stdout, bytes)
                      else (e.stdout or ''))
    return limpar(r.stdout + r.stderr)


def parse_dispositivos(texto):
    """Extrai {MAC: nome} de linhas '[NEW] Device MAC Nome' ou 'Device MAC Nome'."""
    achados = {}
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


def conhecidos():
    """Dispositivos que o BlueZ já conhece (pareados ou vistos antes)."""
    return parse_dispositivos(bluetoothctl('devices'))


def buscar(segundos):
    """Faz uma busca de `segundos` e devolve {MAC: nome} (inclui já conhecidos)."""
    achados = conhecidos()
    print(f'Procurando por {segundos} s...')
    saida = bluetoothctl('--timeout', str(segundos), 'scan', 'on',
                         timeout=segundos + 10)
    achados.update({m: n for m, n in parse_dispositivos(saida).items() if n})
    return achados


def mac_por_nome(nome, mapa):
    for mac, n in mapa.items():
        if n == nome:
            return mac
    return None


def cmd_scan(args):
    achados = buscar(args.segundos)
    if not achados:
        print('Nenhum dispositivo encontrado.')
        return
    for mac, nome in sorted(achados.items(), key=lambda kv: kv[1]):
        print(f'  {mac}  {nome or "(sem nome)"}')


def parear_mac(mac, limite=40):
    """Pareia sem PIN usando um agente NoInputNoOutput e marca como confiável."""
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

    try:
        enviar('agent NoInputNoOutput')
        enviar('default-agent')
        enviar('pairable on')
        enviar(f'pair {mac}', espera=0)
        t0 = time.time()
        while time.time() - t0 < limite:
            texto = ''.join(log)
            if 'Pairing successful' in texto or 'AlreadyExists' in texto \
                    or 'Paired: yes' in texto:
                break
            if 'Failed to pair' in texto:
                break
            time.sleep(0.5)
        texto = ''.join(log)
        ok = ('Pairing successful' in texto or 'AlreadyExists' in texto
              or 'Paired: yes' in texto)
        if ok:
            enviar(f'trust {mac}')
        enviar('quit', espera=0.5)
    finally:
        try:
            p.terminate()
        except OSError:
            pass
    return ok, ''.join(log)


def cmd_pair(args):
    mapa = buscar(args.segundos)
    for nome in args.nomes:
        mac = mac_por_nome(nome, mapa)
        if not mac:
            print(f'[{nome}] NÃO encontrado. Ligado, perto e anunciando esse nome?')
            continue
        print(f'[{nome}] encontrado em {mac}. Pareando...')
        ok, log = parear_mac(mac)
        print(f'[{nome}] pareamento: {"OK" if ok else "FALHOU"}')
        if not ok or args.verbose:
            print('--- saída do bluetoothctl ---')
            print(log)
            print('-----------------------------')


class Robo(threading.Thread):
    """Conecta por RFCOMM a um robô e envia pacotes a SEND_HZ até `fim`."""

    def __init__(self, nome, mac, canal, fim, pwm_e, pwm_d, direcao):
        super().__init__(daemon=True)
        self.nome, self.mac, self.canal, self.fim = nome, mac, canal, fim
        self.pacote = bytes([direcao, pwm_e, pwm_d])
        self.sock = None
        self.conectou_em = None   # s para conectar
        self.apresentacao = None  # texto recebido do robô
        self.enviados = 0
        self.erros = 0
        self.queda = None
        self.max_intervalo = 0.0
        self.erro_inicial = None

    def conectar(self):
        t0 = time.time()
        s = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM,
                          socket.BTPROTO_RFCOMM)
        s.settimeout(15)
        s.connect((self.mac, self.canal))
        self.conectou_em = time.time() - t0
        self.sock = s

    def esperar_apresentacao(self):
        buf = b''
        self.sock.settimeout(HELLO_TIMEOUT)
        try:
            while b'\n' not in buf:
                dados = self.sock.recv(64)
                if not dados:
                    break
                buf += dados
        except socket.timeout:
            pass
        linha = buf.split(b'\n')[0].decode(errors='replace').strip()
        self.apresentacao = linha or None

    def run(self):
        try:
            self.conectar()
            self.esperar_apresentacao()
        except OSError as e:
            self.erro_inicial = str(e)
            return
        self.sock.settimeout(2)
        periodo = 1.0 / SEND_HZ
        proximo = time.time()
        ultimo = None
        while time.time() < self.fim:
            try:
                self.sock.send(self.pacote)
                self.enviados += 1
            except OSError as e:
                self.erros += 1
                self.queda = str(e)
                break
            agora = time.time()
            if ultimo is not None:
                self.max_intervalo = max(self.max_intervalo, agora - ultimo)
            ultimo = agora
            proximo += periodo
            time.sleep(max(0.0, proximo - time.time()))
        # Pacote de parada final por segurança (se ainda conectado).
        try:
            self.sock.send(bytes([0, 0, 0]))
        except OSError:
            pass
        self.sock.close()


def cmd_run(args):
    mapa = conhecidos()
    robos = []
    # Todos começam ao mesmo tempo; o prazo conta a partir de agora + folga
    # para a conexão (as conexões são abertas em paralelo).
    fim = time.time() + 20 + args.segundos
    for nome in args.nomes:
        mac = nome if MAC_RE.match(nome) else mac_por_nome(nome, mapa)
        if not mac:
            sys.exit(f'"{nome}" não é um dispositivo conhecido. Rode o "pair" antes.')
        robos.append(Robo(nome, mac.upper(), args.canal, fim,
                          args.pwm_e, args.pwm_d, args.direcao))

    print(f'Conectando a {len(robos)} robô(s) em paralelo...')
    for r in robos:
        r.start()

    # Espera todos conectarem e se apresentarem para só então medir os envios.
    # (O prazo de cada thread já inclui a folga de 20 s.)
    inicio = time.time()
    while time.time() - inicio < args.segundos + 20 and any(r.is_alive() for r in robos):
        time.sleep(1)
        vivos = sum(1 for r in robos if r.sock and r.queda is None)
        print(f'\r  conectados e enviando: {vivos}/{len(robos)}   ', end='', flush=True)
        if time.time() >= fim:
            break
    for r in robos:
        r.join(timeout=5)
    print()
    relatorio(robos)


def relatorio(robos):
    print('\n=== RESULTADO ===')
    for r in robos:
        print(f'\n[{r.nome}]  {r.mac}')
        if r.erro_inicial:
            print(f'  conexão FALHOU: {r.erro_inicial}')
            continue
        print(f'  conexão em {r.conectou_em:.2f} s')
        esperado = f'{HELLO_PREFIX}{r.nome}'
        if r.apresentacao is None:
            print('  apresentação: NÃO RECEBIDA (firmware novo gravado?)')
        elif r.apresentacao == esperado:
            print(f'  apresentação: OK ("{r.apresentacao}")')
        else:
            print(f'  apresentação: recebida "{r.apresentacao}" '
                  f'(esperado "{esperado}")')
        print(f'  pacotes enviados: {r.enviados}')
        print(f'  maior intervalo entre envios: {r.max_intervalo * 1000:.0f} ms '
              f'(ideal ~{1000 / SEND_HZ:.0f} ms)')
        if r.queda:
            print(f'  CONEXÃO CAIU: {r.queda}')
    print('\nConfira também o Monitor Serial de cada ESP32: as linhas "RX DIR=..."'
          '\ne o "LastPkt(ms)" mostram os pacotes que realmente chegaram.')


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest='cmd', required=True)

    p = sub.add_parser('scan', help='lista dispositivos Bluetooth próximos')
    p.add_argument('--segundos', type=int, default=10)
    p.set_defaults(fn=cmd_scan)

    p = sub.add_parser('pair', help='procura pelo nome, pareia e confia')
    p.add_argument('nomes', nargs='+', help='nomes dos robôs (ex.: zenitinho)')
    p.add_argument('--segundos', type=int, default=10)
    p.add_argument('--verbose', action='store_true')
    p.set_defaults(fn=cmd_pair)

    p = sub.add_parser('run', help='conecta, espera apresentação e envia pacotes')
    p.add_argument('nomes', nargs='+', help='nomes (ou MACs) dos robôs pareados')
    p.add_argument('--segundos', type=int, default=30, help='duração do envio')
    p.add_argument('--canal', type=int, default=RFCOMM_CHANNEL)
    p.add_argument('--pwm-e', type=int, default=0, help='PWM esquerdo (0-255)')
    p.add_argument('--pwm-d', type=int, default=0, help='PWM direito (0-255)')
    p.add_argument('--direcao', type=int, default=3, help='byte de direção')
    p.set_defaults(fn=cmd_run)

    args = ap.parse_args()
    args.fn(args)


if __name__ == '__main__':
    main()
