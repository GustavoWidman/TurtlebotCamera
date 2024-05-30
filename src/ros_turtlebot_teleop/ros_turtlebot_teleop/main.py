import signal
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from InquirerPy.prompts.input import InputPrompt
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty


class Robot:
    def __init__(self):
        self.node = rclpy.create_node('ros_turtlebot_teleop') # type: ignore

        self.publisher = self.node.create_publisher(Twist, '/cmd_vel', 10)
        self.node.create_subscription(Odometry, '/odom', self.odometry_callback, 10)
        self.node.create_service(Empty, '/emergency_stop_teleop', self.emergency_stop_external)
        self.reported_speed = Twist()

        self.console = CustomPrint()
        self.timer = self.node.create_timer(0.1, self.timer_callback)
        self.state = 'stopped'

        self.ready = False
        self.thread = threading.Thread(target=self.await_ready_then_start)
        self.thread.start()

    def await_ready_then_start(self):
        self.node.get_logger().info('Aguardando o estado de prontidão do robô...')
        while not self.ready: time.sleep(0.1)
        self.node.get_logger().info('Robô disponível! Iniciando teleoperação...')

        teleoperate_robot(self)

    def emergency_stop_external(self, request, response):
        self.node.get_logger().info('Recebido pedido de parada de emergência externa')
        print('\n')
        self.node.get_logger().info('PARADA DE EMERGÊNCIA ATIVADA')
        signal.pthread_kill(self.thread.ident, signal.SIGUSR1) # type: ignore
        return response

    def timer_callback(self):
        twist = Twist()

        match self.state:
            case 'stopped':
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.console.log(f"Atualmente PARADO\nVelocidades atuais:\nVel Linear X: {round(self.reported_speed.linear.x, 2)}\nVel Angular Y: {round(self.reported_speed.angular.z, 2)}")
            case 'forward':
                twist.linear.x = 0.5
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
                twist.linear.x = -0.5
                twist.angular.z = 0.0
                self.console.log(f"Atualmente ANDANDO para TRÁS\nVelocidades atuais:\nVel Linear X: {round(self.reported_speed.linear.x, 2)}\nVel Angular Y: {round(self.reported_speed.angular.z, 2)}")
            case _:
                self.node.get_logger().warn(f'Invalid state: {self.state}')

        self.publisher.publish(twist)

    def odometry_callback(self, msg):
        self.reported_speed = msg.twist.twist

        if not self.ready: self.ready = True

    def emergency(self):
        print('\n')
        self.node.get_logger().info('PARADA DE EMERGÊNCIA ATIVADA')
        self.stop()

    def stop(self):
        self.publisher.publish(Twist())
        self.node.get_logger().info('Parando o robô...\n')
        self.node.destroy_node()
        rclpy.shutdown()

class CustomPrint:
    def __init__(self):
        self.last_line_count = 0
        self.activated = False

    def log(self, text):
        if not self.activated: return

        for x in range(self.last_line_count):
            print('\033[1A', end='\x1b[2K')

        print(text)

        self.last_line_count = len(text.split('\n'))

def teleoperate_robot(robot: Robot):
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
    def emergency(_):
        robot.emergency()
        time.sleep(0.1)
        exit()

    try: app.execute()
    except KeyboardInterrupt: robot.stop()


def main(args=None):
    rclpy.init(args=args)
    robot = Robot()

    try:  rclpy.spin(robot.node)
    except KeyboardInterrupt: pass

if __name__ == '__main__':
    main()