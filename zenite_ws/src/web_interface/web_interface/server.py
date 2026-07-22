"""Backend FastAPI da interface web do Zênite (HTTP + WebSocket).

Um único WebSocket (/ws) por cliente transporta tudo:
  servidor -> cliente : frames JPEG (mensagens binárias) e JSON
                        ({"type": "pose" | "status" | "goal_ack" | "error"})
  cliente -> servidor : JSON ({"type": "goal" | "params" | "reload_homography"})

Todo o envio ao cliente é feito por uma única task (_stream_to_client), que
drena uma fila asyncio de mensagens JSON e envia o frame/pose mais recentes —
duas tasks nunca escrevem no mesmo WebSocket ao mesmo tempo.
"""

import asyncio

from ament_index_python.packages import get_package_share_directory
from fastapi import FastAPI, Request, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse
from fastapi.staticfiles import StaticFiles
from fastapi.templating import Jinja2Templates

from .ros_bridge import RosBridgeNode

STREAM_RATE_HZ = 30.0


def create_app(bridge: RosBridgeNode) -> FastAPI:
    share_dir = get_package_share_directory('web_interface')

    app = FastAPI(title='Zênite — Interface Web')
    app.mount('/static', StaticFiles(directory=f'{share_dir}/static'), name='static')
    templates = Jinja2Templates(directory=f'{share_dir}/templates')

    @app.get('/', response_class=HTMLResponse)
    async def index(request: Request):
        return templates.TemplateResponse(request, 'index.html')

    @app.websocket('/ws')
    async def websocket_endpoint(ws: WebSocket):
        await ws.accept()

        out_queue: asyncio.Queue = asyncio.Queue()
        await out_queue.put({'type': 'status', 'homography': bridge.converter.ready})

        sender = asyncio.create_task(_stream_to_client(ws, out_queue))
        try:
            while True:
                data = await ws.receive_json()
                await _handle_client_message(out_queue, data)
        except WebSocketDisconnect:
            pass
        finally:
            sender.cancel()

    async def _stream_to_client(ws: WebSocket, out_queue: asyncio.Queue):
        """Única task que escreve no WebSocket: fila JSON + frame/pose mais recentes."""
        frame_seq = 0
        pose_seq = 0
        period = 1.0 / STREAM_RATE_HZ
        while True:
            while not out_queue.empty():
                await ws.send_json(out_queue.get_nowait())

            jpeg, frame_seq = bridge.get_frame(frame_seq)
            if jpeg is not None:
                await ws.send_bytes(jpeg)

            pose, pose_seq = bridge.get_pose(pose_seq)
            if pose is not None:
                await ws.send_json({'type': 'pose', **pose})

            await asyncio.sleep(period)

    async def _handle_client_message(out_queue: asyncio.Queue, data: dict):
        msg_type = data.get('type')

        if msg_type == 'goal':
            try:
                ack = bridge.publish_goal(float(data['x']), float(data['y']))
                await out_queue.put({'type': 'goal_ack', **ack})
            except Exception as e:
                await out_queue.put({'type': 'error', 'message': str(e)})

        elif msg_type == 'params':
            bridge.publish_params(
                brightness=float(data.get('brightness', 0.5)),
                saturation=float(data.get('saturation', 0.5)),
                hue=float(data.get('hue', 0.5)),
            )

        elif msg_type == 'reload_homography':
            ok = bridge.try_load_homography()
            await out_queue.put({'type': 'status', 'homography': ok})

    return app
