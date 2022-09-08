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
        if msg.buttons[CROSS] != self.cross.data:
            self.cross.data = not self.cross.data
            self.get_logger().info(f'cross: {self.cross.data}')
            self.cross_publisher.publish(self.cross)
        
        if msg.buttons[TRIANGLE] != self.triangle.data:
            self.triangle.data = not self.triangle.data
            self.get_logger().info(f'triangle: {self.triangle.data}')
            self.triangle_publisher.publish(self.triangle)
