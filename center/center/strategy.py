import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class CenterSubscriber(Node):
    def __init__(self):
        super().__init__('center_subscriber')
        self.subscription = self.create_subscription(String, 'center_data', self.center_callback, 10)
        self.get_logger().info('CenterSubscriber has been started and is subscribing to center_data.')
        self.publisher = self.create_publisher(String, 'Calibrated point', 10)  # 傳出需校正的點位
      

    def center_callback(self, msg):
        # 當收到中心點數據時，記錄該信息。
        self.get_logger().info(f'Received center data: {msg.data}')

        #僅發布中心點
        center_data = f'Center of {label}: ({center_x:.2f}, {center_y:.2f})'
        self.publisher.publish(String(data=center_data))  # 發布處理後的數據
        self.get_logger().info(f'Published: {center_data}')

def main(args=None):
    rclpy.init(args=args)
    node = CenterSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
