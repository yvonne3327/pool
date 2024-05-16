import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String

class BoxSubscriber(Node):
    def __init__(self):
        super().__init__('box_subscriber')
        self.subscription_coords = self.create_subscription(
            Float64MultiArray,
            'center_data_coords',
            self.coords_callback,
            10
        )
        self.subscription_labels = self.create_subscription(
            String,
            'center_data_labels',
            self.labels_callback,
            10
        )
        self.coords_data = []
        self.labels_data = []
        self.label_order = ['1', '2', '3', '4', '5', '6', '7', '8', '9', 'white']
        print('BoxSubscriber has been started and is subscribing.')

    def coords_callback(self, msg):
        self.coords_data = msg.data
        print(msg.data[0])
        # 重置标签数据以等待新的完整数据集
        self.labels_data = []

    def labels_callback(self, msg):
        if msg.data in self.label_order:
            self.labels_data.append(msg.data)
            if len(self.coords_data) // 2 == len(self.labels_data):
                self.display_data()

    def display_data(self):
        if self.coords_data and self.labels_data:
            combined_data = []
            for label in self.label_order:
                if label in self.labels_data:
                    index = self.labels_data.index(label)
                    x = self.coords_data[index * 2]
                    y = self.coords_data[index * 2 + 1]
                    combined_data.append(f'{label}, ({x:.2f}, {y:.2f})')
            print('\n'.join(combined_data))
            # Reset data after displaying
            self.coords_data = []
            self.labels_data = []

def main(args=None):
    rclpy.init(args=args)
    node = BoxSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
