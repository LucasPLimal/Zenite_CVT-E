"""Ponto de entrada: sobe o nó ponte (rclpy em thread própria) e o uvicorn.

    ros2 run web_interface web_interface
    ros2 run web_interface web_interface --ros-args -p port:=8080 -p scale_yaml_path:=/tmp/scale.yaml
"""

import threading

import rclpy
import uvicorn

from .ros_bridge import RosBridgeNode
from .server import create_app


def main(args=None):
    rclpy.init(args=args)
    bridge = RosBridgeNode()

    ros_thread = threading.Thread(target=rclpy.spin, args=(bridge,), daemon=True)
    ros_thread.start()

    app = create_app(bridge)

    bridge.get_logger().info(
        f'Interface web disponível em http://{bridge.host}:{bridge.port}')

    try:
        uvicorn.run(app, host=bridge.host, port=bridge.port, log_level='warning')
    except KeyboardInterrupt:
        pass
    finally:
        bridge.destroy_node()
        rclpy.shutdown()
        ros_thread.join(timeout=2.0)


if __name__ == '__main__':
    main()
