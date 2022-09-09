from audioop import cross
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
    def __init__(self, name=None,Right_publisher_topic=None, Left_publisher_topic=None, Reset_publisher_topic=None):
        self.name = name if name is not None else "controller"
        self.Right_publisher_topic = Right_publisher_topic if Right_publisher_topic is not None else "Right_cmd"
        self.Left_publisher_topic = Left_publisher_topic if Left_publisher_topic is not None else "Left_cmd"
        self.Reset_publisher_topic = Reset_publisher_topic if Reset_publisher_topic is not None else "Reset_cmd"


        #publisher
        self.Right_publisher = self.create_publisher(Bool, self.Right_publisher_topic, 10)
        self.Left_publisher = self.create_publisher(Bool, self.Left_publisher_topic, 10)
        self.Reset_publisher = self.create_publisher(Bool, self.Reset_publisher_topic, 10)


        self.Right = Bool()
        self.Right.data = False
        self.Left = Bool()
        self.Left.data = False
        self.Reset = Bool()
        self.Reset.data = False

        #subsriber
        self.subscription = self.create_subscription(Joy, "joy", self.joyCallback, 10)

    def joyCallback(self, msg):
        if (msg.axes[7] and msg.axes[6] == 1):
            self.Reset.data = not self.Reset.data
            self.get_logger().info(f'Reset: {self.Reset.data}')
            self.Reset_publisher.publish(self.Reset)

        if msg.axes[7] == -1:
            self.Right.data = not self.Right.data
            self.get_logger().info(f'Right: {self.Right.data}')
            self.Right_publisher.publish(self.Right)
        
        if msg.axes[6] == -1:
            self.Left.data = not self.Left.data
            self.get_logger().info(f'Left: {self.Left.data}')
            self.Left_publisher.publish(self.Left)
