import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import sys
import center.tableball as table

# from . import tableball as table  # Adjust this import based on your file structure
# import pygame as pg

class BoxSubscriber(Node):
    def __init__(self):
        super().__init__('pygame_node')
        self.subscription = self.create_subscription(Float64MultiArray, 'center_data_coords', self.strategy_callback, 10)
        self.publisher_hitpoint = self.create_publisher(Float64MultiArray, 'strategy_hitpoint', 10)
        self.get_logger().info('BoxSubscriber has been started and is subscribing to data.')
        self.max=0
        self.all_10_data = []
        self.return_value=[]
    def strategy_callback(self, msg):
        # self.get_logger().info(f'Received coordinates: {msg.data}')
        temp_max = 0
        data_flag=0
        if self.max==5:
            max_data = []
            for data in self.all_10_data:
                if len(data) > temp_max:
                    temp_max = len(data)
                    max_data = data
            self.max = 0
            data_flag=1
            self.get_logger().info(f'max_data: {max_data}')

        else:
            self.max+=1
            self.all_10_data.append(msg.data)
        #[0,2,3,6,5,7]
        ballx_set=[]
        bally_set=[]
        if data_flag==1:
            for i in range(len(max_data)):
                if i%2==0:
                    ballx_set.append(max_data[i])
                else:
                    bally_set.append(max_data[i])
            ballcount=int((len(max_data)-2)/2)
            cuex=ballx_set[-1]
            cuey=bally_set[-1]

            self.return_value=table.main(ballx_set, bally_set, ballcount, cuex, cuey)
            self.get_logger().info(f'maxaaa: {self.return_value}')
            data_flag=0

            # # 發布中心點數據
            # self.publisher_hitpoint.publish(self.return_value)
            # self.get_logger().info('hitpoint_success')

def main(args=None):
    rclpy.init(args=args)
    node = BoxSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()