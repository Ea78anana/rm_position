import time
import math
from tkinter import *
import multiprocessing
from geometry_msgs.msg import Twist, Vector3
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool

SQUARE = 0
CROSS = 1
CIRCLE = 2
TRIANGLE = 3
OPTIONS = 9
L1 = 4
R1 = 5
L2 = 6
R2 = 7

class controller(Node):
    def __init__(self, name=None,triangle_publisher_topic=None, circle_publisher_topic=None):
        self.name = name if name is not None else "controller"
        self.triangle_publisher_topic = triangle_publisher_topic if triangle_publisher_topic is not None else "triangle_cmd"
        self.circle_publisher_topic = circle_publisher_topic if circle_publisher_topic is not None else "cross_cmd"

        #publisher
        self.triangle_publisher = self.create_publisher(Bool, self.triangle_publisher_topic, 10)
        self.cross_publisher = self.create_publisher(Bool, self.cross_publisher_topic, 10)

        self.triangle = Bool()
        self.triangle.data = False
        self.cross = Bool()
        self.cross.data = False
        #subsriber
        self.subscription = self.create_subscription(Joy, "joy", self.joyCallback, 10)

    def joyCallback(self, msg):
        if msg.axes[7] == 1:
            self.Dup.data = not self.Dup.data
            self.get_logger().info(f'Reset: {self.Reset.data}')
            self.Dup_publisher.publish(self.Dup)
        
        if msg.axes[7] == -1:
            self.Ddow.data = not self.Ddow.data
            self.get_logger().info(f'Right: {self.Right.data}')
            self.Ddow_publisher.publish(self.Ddow)

        if msg.axes[6] == 1:
            self.Dup.data = not self.Dup.data
            self.get_logger().info(f'Reset: {self.Reset.data}')
            self.Dup_publisher.publish(self.Dup)
        
        if msg.axes[6] == -1:
            self.Ddow.data = not self.Ddow.data
            self.get_logger().info(f'Left: {self.Left.data}')
            self.Ddow_publisher.publish(self.Ddow)
