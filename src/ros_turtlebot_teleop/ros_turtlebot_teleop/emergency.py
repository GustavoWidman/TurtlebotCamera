import time
import rclpy
from std_srvs.srv import Empty


class RobotStopper2000:
	def __init__(self):
		self.node = rclpy.create_node('robot_stopper_2000') # type: ignore
		self.client = self.node.create_client(Empty, '/emergency_stop_teleop')

	def stop_robot(self):
		self.node.get_logger().info('Waiting for the service to be available...')

		while not self.client.wait_for_service(timeout_sec=1.0): time.sleep(0.1)

		self.node.get_logger().info('Service is available')

		self.client.call_async(Empty.Request())

		self.node.get_logger().info('Robot stopped successfully')

def main(args=None):
	rclpy.init(args=args)

	stopper = RobotStopper2000()
	stopper.stop_robot()

	rclpy.shutdown()

if __name__ == '__main__':
	main()