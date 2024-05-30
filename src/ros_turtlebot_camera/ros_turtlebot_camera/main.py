import asyncio
import base64
import datetime
import json
import threading
from typing import Literal

import cv2
import rclpy
import uvicorn
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse
from geometry_msgs.msg import Twist
from InquirerPy.prompts.input import InputPrompt
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_srvs.srv import Empty


class Robot:
    def __init__(self):
        self.node = rclpy.create_node('ros_turtlebot_camera') # type: ignore

        self.publisher = self.node.create_publisher(Twist, '/cmd_vel', 10)
        self.node.create_subscription(Odometry, '/odom', self.odometry_callback, 10)

        self.node.create_service(Empty, '/emergency_stop', self.emergency_stop_callback)
        self.node.create_service(Empty, '/stop', self.stop_callback)
        self.emergency_service = self.node.create_client(Empty, '/emergency_stop')
        self.stop_service = self.node.create_client(Empty, '/stop')

        self.reported_speed = Twist()

        self.console = CustomPrint()
        self.timer = self.node.create_timer(0.1, self.timer_callback)
        self.state = 'stopped'

        self.ready = False

        self.node.get_logger().info('Aguardando o estado de prontidão do robô.')
        self.loop = asyncio.get_event_loop()
        self.loop.create_task(self.await_ready_then_start())

    async def await_ready_then_start(self):
        while not self.ready:
            await asyncio.sleep(0.1)
        self.node.get_logger().info('Robô disponível! Iniciando teleoperação...')

        await teleoperate_robot(self)

    def emergency_stop_callback(self, request, response):
        try:
            return response
        finally:
            self.emergency()

    def stop_callback(self, request, response):
        try:
            return response
        finally:
            self.stop()

    def timer_callback(self):
        twist = Twist()

        match self.state:
            case 'stopped':
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.console.log(f"Atualmente PARADO\nVelocidades atuais:\nVel Linear X: {round(self.reported_speed.linear.x, 2)}\nVel Angular Y: {round(self.reported_speed.angular.z, 2)}")
            case 'forward':
                twist.linear.x = 0.2
                twist.angular.z = 0.0
                self.console.log(f"Atualmente ANDANDO para a FRENTE\nVelocidades atuais:\nVel Linear X: {round(self.reported_speed.linear.x, 2)}\nVel Angular Y: {round(self.reported_speed.angular.z, 2)}")
            case 'left':
                twist.linear.x = 0.0
                twist.angular.z = 1.0
                self.console.log(f"Atualmente VIRANDO para a ESQUERDA\nVelocidades atuais:\nVel Linear X: {round(self.reported_speed.linear.x, 2)}\nVel Angular Y: {round(self.reported_speed.angular.z, 2)}")
            case 'right':
                twist.linear.x = 0.0
                twist.angular.z = -1.0
                self.console.log(f"Atualmente VIRANDO para a DIREITA\nVelocidades atuais:\nVel Linear X: {round(self.reported_speed.linear.x, 2)}\nVel Angular Y: {round(self.reported_speed.angular.z, 2)}")
            case 'backward':
                twist.linear.x = -0.2
                twist.angular.z = 0.0
                self.console.log(f"Atualmente ANDANDO para TRÁS\nVelocidades atuais:\nVel Linear X: {round(self.reported_speed.linear.x, 2)}\nVel Angular Y: {round(self.reported_speed.angular.z, 2)}")
            case _:
                self.node.get_logger().warn(f'Invalid state: {self.state}')

        self.publisher.publish(twist)

    def odometry_callback(self, msg):
        self.reported_speed = msg.twist.twist

        if not self.ready:
            self.ready = True

    def emergency(self):
        print('\n')
        self.node.get_logger().info('PARADA DE EMERGÊNCIA ATIVADA')
        self.stop()

    def stop(self):
        self.publisher.publish(Twist())
        self.node.get_logger().info('Parando o robô...\n')
        self.node.destroy_node()

        for task in asyncio.all_tasks(loop=self.loop):
            task_name = task.get_coro().__name__ # type: ignore

            # make sure we dont kill ourselves lol
            if task_name != 'await_ready_then_start' and task_name != 'spin':
                task.cancel()

        self.loop.call_soon_threadsafe(self.loop.stop)

class WebSocketServer:
    def __init__(self,
        robot: Robot,
        app: FastAPI = FastAPI(),
        framerate: int = 60,
    ):
        self.app = app
        self.image_clients = set()
        self.status_clients = set()
        self.loop = robot.loop
        self.robot = robot
        self.capture = None
        self.sleep_time = 1 / framerate
        self.framerate = framerate

    async def _broadcast(self, message, clients: set):
        await asyncio.gather(*[client.send_text(message) for client in clients], return_exceptions=False)

    def broadcast(self, message: str, clients: set):
        """Synchronous wrapper for the asynchronous _broadcast function."""
        if self.loop:
            asyncio.run_coroutine_threadsafe(self._broadcast(message, clients), self.loop)

    def start_server(self):
        @self.app.websocket("/image")
        async def image_endpoint(websocket: WebSocket):
            await websocket.accept()
            self.image_clients.add(websocket)

            try:
                while True:
                    await websocket.receive_text()
            except WebSocketDisconnect:
                self.image_clients.remove(websocket)
                await websocket.close()

        @self.app.websocket("/status")
        async def status_endpoint(websocket: WebSocket):
            await websocket.accept()
            self.status_clients.add(websocket)

            try:
                while True:
                    await asyncio.sleep(1)
            except WebSocketDisconnect:
                self.status_clients.remove(websocket)
                await websocket.close()

    async def broadcast_image(self):
        if not self.capture:
            self.capture = cv2.VideoCapture(0)

        ret, frame = self.capture.read()

        timestamp = int(datetime.datetime.now(datetime.timezone.utc).timestamp() * 1000)

        if ret:
            _, buffer = cv2.imencode('.jpg', frame)

            broadcast = {
                "bytes": base64.b64encode(buffer.tobytes()).decode('utf-8'),
                "timestamp": str(timestamp)
            }

            self.broadcast(json.dumps(broadcast), self.image_clients)

    async def broadcast_image_forever(self):
        while not self.loop:
            await asyncio.sleep(0.1)

        while True:
            if len(self.image_clients) == 0:
                if self.capture:
                    self.capture.release()
                    self.capture = None

                await asyncio.sleep(0.1)
                continue

            self.loop.create_task(self.broadcast_image())
            await asyncio.sleep(self.sleep_time)

    async def broadcast_status(self):
        broadcast = {
            "state": self.robot.state,
            "speed": {
                "linear": round(self.robot.reported_speed.linear.x, 2),
                "angular": round(self.robot.reported_speed.angular.z, 2),
            }
        }

        self.broadcast(json.dumps(broadcast), self.status_clients)

    async def broadcast_status_forever(self):
        while not self.loop:
            await asyncio.sleep(0.1)

        while True:
            if len(self.status_clients) == 0:
                await asyncio.sleep(0.1)
                continue

            self.loop.create_task(self.broadcast_status())
            await asyncio.sleep(0.1)

    def run(self):
        self.start_server()
        self.loop.create_task(self.broadcast_image_forever())
        self.loop.create_task(self.broadcast_status_forever())

class APIServer:
    def __init__(self,
        robot: Robot,
        host='0.0.0.0',
        port=8000,
    ):
        self.host = host
        self.port = port
        self.loop = robot.loop
        self.robot = robot
        self.app = FastAPI()

        self.bind_routes()

    def bind_routes(self):
        @self.app.get('/')
        async def root():
            return FileResponse('index.html')

        @self.app.get('/emergency')
        async def emergency():
            self.robot.emergency_service.call_async(Empty.Request())

            return {"status": "success"}

        @self.app.get('/stop')
        async def stop():
            self.robot.stop_service.call_async(Empty.Request())

            return {"status": "success"}

        @self.app.get('/control')
        async def control(state: Literal['forward', 'left', 'right', 'backward', 'stopped'] = 'stopped'):
            self.robot.state = state

            return {"status": "success"}


    async def start_server(self):
        config = uvicorn.Config(self.app, host=self.host, port=self.port, log_level="critical")
        server = uvicorn.Server(config)
        self.robot.node.get_logger().info(f"Dashboard do robô disponível em http://{self.host}:{self.port}")
        await server.serve()

    def run(self):
        self.loop.create_task(self.start_server())

class CustomPrint:
    def __init__(self):
        self.last_line_count = 0
        self.activated = False

    def log(self, text):
        if not self.activated:
            return

        for x in range(self.last_line_count):
            print('\033[1A', end='\x1b[2K')

        print(text)

        self.last_line_count = len(text.split('\n'))

async def teleoperate_robot(robot: Robot):
    app = InputPrompt(message="", qmark="", amark="")
    print("""
Bem-vindo ao teleoperador do robô!

Controles:
    - WASD para movimentar o robô ou setas direcionais (cima, baixo, esquerda, direita)
    - Espaço para parar o robô
    - Q para ativar a PARADA DE EMERGÊNCIA e imediatamente parar o robô
""")
    robot.console.activated = True

    @app.register_kb("up")
    def forward(_): robot.state = 'forward'
    @app.register_kb("w")
    def forward_clone(_): forward(None) # type: ignore

    @app.register_kb("left")
    def left(_): robot.state = 'left'
    @app.register_kb("a")
    def left_clone(_): left(None) # type: ignore

    @app.register_kb("right")
    def right(_): robot.state = 'right'
    @app.register_kb("d")
    def right_clone(_): right(None) # type: ignore

    @app.register_kb("down")
    def backwards(_): robot.state = 'backward'
    @app.register_kb("s")
    def backwards_clone(_): backwards(None) # type: ignore

    @app.register_kb("space")
    def stop(_): robot.state = 'stopped'

    @app.register_kb("q")
    def emergency(_): robot.emergency_service.call_async(Empty.Request())

    try:
        await app.execute_async()
    except KeyboardInterrupt:
        robot.stop_service.call_async(Empty.Request())

async def spin(node: Node):
    cancel = node.create_guard_condition(lambda: None)
    def _spin(
        node: Node,
        future: asyncio.Future,
        event_loop: asyncio.AbstractEventLoop
    ):
        try:
            while not future.cancelled():
                rclpy.spin_once(node)
            if not future.cancelled():
                event_loop.call_soon_threadsafe(future.set_result, None)
        except Exception as e:
            if not future.cancelled():
                event_loop.call_soon_threadsafe(future.set_exception, e)

    event_loop = asyncio.get_event_loop()
    spin_task = event_loop.create_future()
    spin_thread = threading.Thread(target=_spin, args=(node, spin_task, event_loop), daemon=True)
    spin_thread.start()

    try:
        await spin_task
    except asyncio.CancelledError:
        cancel.trigger()

    spin_thread.join()
    node.destroy_guard_condition(cancel)

def main(args=None):
    rclpy.init(args=args)
    robot = Robot()
    api_server = APIServer(robot)
    websocket_server = WebSocketServer(robot, api_server.app)

    try:
        robot.loop.create_task(spin(robot.node))
        websocket_server.run()
        api_server.run()
        robot.loop.run_forever()
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()