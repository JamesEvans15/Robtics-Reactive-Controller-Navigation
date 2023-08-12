#!/usr/bin/env python3
import rclpy
import math
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32, Twist
import numpy as np

from example_interfaces.msg import Int64

from nav_msgs.msg import OccupancyGrid

from project_4.disc_robot import load_disc_robot

from std_msgs.msg import Float64

import yaml

from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import tf2_geometry_msgs
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import PointCloud


class nav_controller(Node):
    def __init__(self):
        super().__init__('nav_node')
        self.robot = load_disc_robot('/home/parallels/ros2_ws/src/project_4/project_4/robots/normal.robot')

        self.vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        self.scan_sub = self.create_subscription(LaserScan, "/scan", self.handle_scan, 10)

        self.ranges = []

    def handle_scan(self, msg: LaserScan):

        if(msg.ranges == []):
            return

        twist_obj = Twist()

        self.ranges = msg.ranges
        
        ranges = self.ranges
        midpoint = math.floor(0.5 * len(ranges))

        try:
            spot = ranges.index(max(ranges))
            closest_spot = ranges.index(min(ranges))
        except:
            return

        twist_obj.linear.x = 1.0 - ( abs( spot - (midpoint)) / len(ranges) )

        twist_obj.linear.y = 0.0
        twist_obj.linear.z = 0.0

        twist_obj.angular.x = 0.0
        twist_obj.angular.y = 0.0

    
        if(spot != midpoint):
            twist_obj.angular.z = (spot - (midpoint)) / len(ranges)
        elif(spot):
            twist_obj.angular.z = 0.0


        for k in range(len(ranges)):
            angle = (msg.angle_min + k * msg.angle_increment) * 180/math.pi
            if ranges[k] <= self.robot['body']['radius'] + 0.16 and abs(angle) < 85:
                twist_obj.angular.z += (-(k - (midpoint)) / len(ranges)) * 0.5
                twist_obj.linear.x = 0.1

        self.vel_pub.publish(twist_obj)

class velocity_translator(Node):
    def __init__(self):
        super().__init__('translator_node')
        self.robot = load_disc_robot('/home/parallels/ros2_ws/src/project_4/project_4/robots/normal.robot')

        self.vl_pub = self.create_publisher(Float64, "/vl", 10)
        self.vr_pub = self.create_publisher(Float64, "/vr", 10)

        self.vel_sub = self.create_subscription(Twist, "/cmd_vel", self.handle_cmd, 10)

        self.linear_scale = 0.25
        self.angular_scale = 1.2
        self.robot_width = self.robot['wheels']['distance']
        

    def handle_cmd(self, msg):
        linear_velocity = msg.linear.x * self.linear_scale
        angular_velocity = msg.angular.z * self.angular_scale

        vr = Float64()
        vr.data = linear_velocity + angular_velocity * self.robot_width / 2

        vl = Float64()
        vl.data = linear_velocity - angular_velocity * self.robot_width / 2

        self.vl_pub.publish(vl)
        self.vr_pub.publish(vr)

        


def nav_main(args=None):
    rclpy.init(args=None)
    
    node = nav_controller()
    rclpy.spin(node)
    node.destroy_node()

    rclpy.shutdown()

def translator_main(args=None):
    rclpy.init(args=None)
    
    node = velocity_translator()
    rclpy.spin(node)
    node.destroy_node()

    rclpy.shutdown()