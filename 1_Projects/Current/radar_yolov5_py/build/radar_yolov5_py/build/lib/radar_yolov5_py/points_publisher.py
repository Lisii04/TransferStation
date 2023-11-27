#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import array

import time

class PointsPublisher(Node):
    def __init__(self,name):
        super().__init__(name)
        self.get_logger().info("[start]%s" % name)
        self.command_publisher_ = self.create_publisher(Float32MultiArray,"points_data", 10) 
    
    def send_points(self,points):
        msg = Float32MultiArray()
        msg.data = points
        self.command_publisher_.publish(msg) 
        self.get_logger().info(f'传输：\n{msg.data}\n')    #打印一下发布的数据

def main(args=None):
    points = array.array('f',[22.114514,23.114514,34.114514,41.114514,23.114513])
    rclpy.init(args=args) # 初始化rclpy
    node = PointsPublisher("points_publisher")  # 新建一个节点
    for i in range(1,100):
        PointsPublisher.send_points(node,points)
        time.sleep(1)
    rclpy.spin(node) # 保持节点运行，检测是否收到退出指令（Ctrl+C）
    rclpy.shutdown() # 关闭rclpy
