import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import tableball
import sys
from threading import Thread


class BoxSubscriber(Node):
    def __init__(self, game):
        super().__init__('pygame_node')
        self.subscription_coords = self.create_subscription(Float64MultiArray, 'center_data_coords', self.coords_callback, 10)
        self.game = game

    def coords_callback(self, msg):
        # 在这里处理从 'center_data_coords' 主题接收到的数据
        coords_data = msg.data
        # 将接收到的参数传递给 TableBall 游戏
        self.game.update_ball_position(coords_data)


class TableBallNode(Node):
    def __init__(self):
        super().__init__('tableball_node')
        self.initialize_pygame()
        self.timer = self.create_timer(0.1, self.update_game)  # 10 Hz game update

    def initialize_pygame(self):
        tableball.init()
        self.width, self.height = 1000, 500
        self.screen = tableball.display.set_mode((self.width, self.height))
        tableball.display.set_caption("Table Ball")

    def update_game(self):
        for event in tableball.event.get():
            if event.type == tableball.QUIT:
                tableball.quit()
                sys.exit()
            # Add more event handling here

        # Update game logic here
        # 可以在这里使用接收到的参数来更新游戏逻辑


def start_ros_node(node):
    rclpy.spin(node)


def main(args=None):
    rclpy.init(args=args)
    tableball_node = TableBallNode()

    # 启动 TableBall 节点的 ROS 运行循环
    ros_thread = Thread(target=start_ros_node, args=(tableball_node,))
    ros_thread.start()

    # 创建 BoxSubscriber 节点并传递 TableBallNode 的实例
    pygame_node = BoxSubscriber(tableball_node)

    try:
        rclpy.spin(pygame_node)
    except KeyboardInterrupt:
        pass
    finally:
        pygame_node.destroy_node()
        tableball_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
