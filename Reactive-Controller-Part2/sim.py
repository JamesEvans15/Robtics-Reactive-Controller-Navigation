#!/usr/bin/env python3
import rclpy
import math
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
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

class sim_node(Node):
    def __init__(self):
        super().__init__('sim_node')

        with open("/home/parallels/ros2_ws/src/project_4/project_4/worlds/snake.world") as f:
            map = yaml.safe_load(f)

        self.robot_x = map['initial_pose'][0] 
        self.robot_y = map['initial_pose'][1] 
        self.robot_z = map['initial_pose'][2]

        self.vl = 0.0
        self.vr = 0.0

        self.map_publisher = self.create_publisher(OccupancyGrid, "/map", 10)

        self.laser_publisher = self.create_publisher(LaserScan, "/scan", 10)
        self.pc_publisher = self.create_publisher(PointCloud, '/laser_pc', 10)

        timer_period = .001

        self.og = OccupancyGrid()

        self.timer = self.create_timer(timer_period,self.transmit_map)

        self.robot = load_disc_robot('/home/parallels/ros2_ws/src/project_4/project_4/robots/normal.robot')

        self.laser_timer = self.create_timer(self.robot['laser']['rate'], self.handle_scan)

        self.world_to_base = TransformBroadcaster(self)

        self.vr_sub = self.create_subscription(Float64, '/vr', self.handle_vr, 10)
        self.vl_sub = self.create_subscription(Float64, '/vl', self.handle_vl, 10)

        self.error_timer = self.create_timer(self.robot['wheels']['error_update_rate'], self.handle_v_error)

        self.lin_vel = 0.0
        self.ang_vel = 0.0

        self.prev_t = self.get_clock().now()
        
        # self.init_robot()

        self.most_recent_call = self.get_clock().now()
        self.mover = self.create_timer(0.001, self.init_robot)

        # self.laser_to_heading = TransformBroadcaster(self)
        # self.mover = self.create_timer(0.001, self.laser_to_base)


    def move_robot(self):
        curr_t = self.get_clock().now()
        dt = (curr_t.nanoseconds - self.prev_t.nanoseconds) / int(1e9) 
        self.prev_t = curr_t

        self.lin_vel = (self.vl + self.vr) / 2
        self.ang_vel = (self.vr - self.vl) / self.robot['wheels']['distance']

        self.dz = self.ang_vel * dt
        self.dx = self.lin_vel * math.cos(self.robot_z + self.dz / 2) * dt
        self.dy = self.lin_vel * math.sin(self.robot_z + self.dz / 2) * dt

        if(self.og.info.resolution != 0):
            if(self.does_robot_collide_x(self.robot_x + self.dx)):
                self.dx = 0.0
            
            if(self.does_robot_collide_y(self.robot_y + self.dy)):
                self.dy = 0.0

        self.robot_x += self.dx
        self.robot_y += self.dy
        self.robot_z += self.dz

    
    def handle_scan(self):
        if(self.og.data is None):
            return
        
        ls = LaserScan()
        ls.header.frame_id = "laser"
        ls.angle_min = self.robot['laser']['angle_min']
        ls.angle_max = self.robot['laser']['angle_max']
        ls.range_min = self.robot['laser']['range_min']
        ls.range_max = self.robot['laser']['range_max']
        
        count = self.robot['laser']['count']
        
        ls.angle_increment = (ls.angle_max - ls.angle_min) / (count-1)

        robot_orientation = self.robot_z

        lidar_x = self.robot_x + math.cos(robot_orientation) * (self.robot['body']['radius'] / 2)
        lidar_y = self.robot_y + math.sin(robot_orientation) * (self.robot['body']['radius'] / 2)

        ranges = []

        lines = self.get_map_lines()

        # self.get_logger().info(str(lines))

        t_between = 1
        curr_t = 0

        if(abs(self.ang_vel) > 0):
            t_between = 1 / self.ang_vel
       
        
        for i in range(count):
            angle = robot_orientation + ls.angle_min + i * ls.angle_increment

            distances = []
            for j in lines:

                intersect = self.ray_line_intersection(lidar_x, lidar_y, angle, j[0][0], j[0][1], j[1][0], j[1][1])
                if(intersect is not None):
                    dist = math.sqrt((intersect[0] - lidar_x)**2 + (intersect[1] - lidar_y)**2)
                    distances.append(dist)
                
            if(len(distances) > 0):
                min_dist = min(distances)
                error_variance = self.robot['laser']['error_variance']
                noise = np.random.normal(0, math.sqrt(error_variance), 1)
                min_dist += noise[0]
                
                fail_probability = self.robot['laser']['fail_probability']
                rand = np.random.random()

                if(rand <= fail_probability):
                    ranges.append(np.nan)
                else:
                    ranges.append(min_dist)
            

        ls.ranges = ranges

        pc = self.laser_scan_to_point_cloud(ls)

        self.laser_trans(lidar_x, lidar_y)
        self.laser_publisher.publish(ls)
        self.pc_publisher.publish(pc)
    

    def ray_line_intersection(self, x1, y1, theta, x2, y2, x3, y3):        
        ray_m = math.tan(theta)
        ray_b = y1 - ray_m * x1 

        dir_ray = np.array([np.cos(theta), np.sin(theta)])
        dir_line = np.array([x3 - x2, y3 - y2])

        cross_prod = np.cross(dir_ray, dir_line)
        if(cross_prod < 0):
            return None
        
        dot_product = (x2-x1)*(y3-y1) - (x3-x1)*(y2-y1)
        angle = math.atan2(dot_product, x2-x1)

        if(angle < 0):
            return None

        if(abs(y3-y2) <= 0.05): # horizontal slope
            ray_x = (y3 - ray_b) / ray_m 
            if( (x2 <= ray_x <= x3) or (x3 <= ray_x <= x2) ):       
                if(ray_x == -0.0):
                    return None    
                return (ray_x, y2)
            
        else: # vertical slope
            ray_y = ray_m * x2 + ray_b
            if( (y2 <= ray_y <= y3) or (y3 <= ray_y <= y2) ):
                return (x2, ray_y)  
          

    def laser_scan_to_point_cloud(self, scan):
        return PointCloud(header=scan.header, points = [ Point32(x=scan.ranges[i]*math.cos(scan.angle_min+i*scan.angle_increment), y=scan.ranges[i]*math.sin(scan.angle_min+i*scan.angle_increment), z=0.0) for i in range(len(scan.ranges)) ])

    def get_map_obstacle_locations(self):
        map_data = np.array(self.og.data).reshape((self.og.info.height, self.og.info.width))

        coords = []
        for r in range(len(map_data)):
            for c in range(len(map_data[0])):
                if map_data[r][c] == 100:
                    coords.append((c * self.og.info.resolution, r * self.og.info.resolution))

        return coords
    

    def get_map_obstacle_locations_int(self):
        map_data = np.array(self.og.data).reshape((self.og.info.height, self.og.info.width))

        coords = []
        for r in range(len(map_data)):
            for c in range(len(map_data[0])):
                if map_data[r][c] == 100:
                    coords.append((c, r))

        return coords
    
    def get_map_lines(self):
        points = self.get_map_obstacle_locations_int()
        r = self.og.info.resolution
        adjacent_lines = set()
        for point in points:
            x, y = point

            adjacent_lines.add((((x+1)* r,y* r), ((x+1)* r,(y+1)* r)))
            adjacent_lines.add(((x * r,(y+1)* r), ((x+1)* r,(y+1)* r)))
            
            adjacent_lines.add((((x+1)* r,(y+1)* r), ((x+1)* r,y* r)))
            adjacent_lines.add((((x+1)* r,(y+1)* r), (x * r,(y+1)* r)))

            adjacent_lines.add((((x)* r,y* r), ((x)* r,(y+1)* r)))
            adjacent_lines.add((((x) * r,(y+1)* r), ((x)* r,(y)* r)))
            
            adjacent_lines.add((((x)* r,(y)* r), ((x+1)* r,y* r)))
            adjacent_lines.add((((x+1)* r,(y)* r), (x * r,(y)* r)))

        return adjacent_lines


    def init_robot(self):
        curr_t = self.get_clock().now()
        dt = (curr_t.nanoseconds - self.most_recent_call.nanoseconds) / int(1e9)
        if(dt > 1):
            self.dx = 0.0
            self.dy = 0.0
            self.dz = 0.0
            self.lin_vel = 0.0
            self.ang_vel = 0.0
        else:
            self.move_robot()
        
        transform_stamped = TransformStamped()
        
        transform_stamped.header.stamp = self.get_clock().now().to_msg()
        transform_stamped.header.frame_id = 'world'
        transform_stamped.child_frame_id = 'base_link'
        transform_stamped.transform.translation.x = self.robot_x
        transform_stamped.transform.translation.y = self.robot_y
        transform_stamped.transform.translation.z = 0.0

        q = self.get_quaternion(self.robot_z)
        transform_stamped.transform.rotation.x = q[0]
        transform_stamped.transform.rotation.y = q[1]
        transform_stamped.transform.rotation.z = q[2]
        transform_stamped.transform.rotation.w = q[3]

        self.world_to_base.sendTransform(transform_stamped)

    def laser_trans(self, x, y):        
        transform_stamped = TransformStamped()
        
        transform_stamped.header.stamp = self.get_clock().now().to_msg()
        transform_stamped.header.frame_id = 'world'
        transform_stamped.child_frame_id = 'laser'
        transform_stamped.transform.translation.x = x
        transform_stamped.transform.translation.y = y
        transform_stamped.transform.translation.z = 0.0

        q = self.get_quaternion(self.robot_z)
        transform_stamped.transform.rotation.x = q[0]
        transform_stamped.transform.rotation.y = q[1]
        transform_stamped.transform.rotation.z = q[2]
        transform_stamped.transform.rotation.w = q[3]

        self.world_to_base.sendTransform(transform_stamped)

    def does_robot_collide_x(self, x):
        lines = self.get_map_lines()
        r = self.robot['body']['radius']

        for k in lines:
            x1 = k[0][0]
            y1 = k[0][1]
            
            x2 = k[1][0]
            y2 = k[1][1]

            if(abs(x1 - x2) < 0.05): # vertical line
                dx = x2 - x1
                dy = y2 - y1
                segment_length_squared = dx*dx + dy*dy

                if segment_length_squared == 0:
                    return math.sqrt((x-x1)*(x-x1) + (self.robot_y-y1)*(self.robot_y-y1))

                t = ((x - x1) * dx + (self.robot_y - y1) * dy) / segment_length_squared
                t = max(0, min(1, t))
                closest_x = x1 + t * dx
                closest_y = y1 + t * dy

                distance_squared = (x - closest_x) * (x - closest_x) + (self.robot_y - closest_y) * (self.robot_y - closest_y)
                distance = math.sqrt(distance_squared)

                if(distance < r):
                    return True
        
        return False

    def does_robot_collide_y(self, y):
        lines = self.get_map_lines()
        r = self.robot['body']['radius']

        for k in lines:
            x1 = k[0][0]
            y1 = k[0][1]
            
            x2 = k[1][0]
            y2 = k[1][1]

            if(abs(y1 - y2) < 0.05): # horizontal line
                dx = x2 - x1
                dy = y2 - y1
                segment_length_squared = dx*dx + dy*dy

                if segment_length_squared == 0:
                    return math.sqrt((self.robot_x-x1)*(self.robot_x-x1) + (y-y1)*(y-y1))

                t = ((self.robot_x - x1) * dx + (y - y1) * dy) / segment_length_squared
                t = max(0, min(1, t))
                closest_x = x1 + t * dx
                closest_y = y1 + t * dy

                distance_squared = (self.robot_x - closest_x) * (self.robot_x - closest_x) + (y - closest_y) * (y - closest_y)
                distance = math.sqrt(distance_squared)

                if(distance < r):
                    return True
        
        return False

    
    def get_quaternion(self, theta):
        q = [0.0, 0.0, 0.0, 1.0]
        q[2] = math.sin(theta / 2)
        q[3] = math.cos(theta / 2)
        return q
    
    def handle_vr(self, msg):
        self.most_recent_call = self.get_clock().now()
        noise_r = np.random.normal(1, math.sqrt(self.robot['wheels']['error_variance_right']), 1)
        self.vr = float(msg.data * noise_r)
        # self.init_robot()
        self.prev_t = self.get_clock().now()
        self.move_robot()
    
    def handle_vl(self, msg:Float64):
        self.most_recent_call = self.get_clock().now()
        noise_l = np.random.normal(1, math.sqrt(self.robot['wheels']['error_variance_left']), 1)  
        self.vl = float(msg.data * noise_l)
        # self.init_robot()
        self.prev_t = self.get_clock().now()
        self.move_robot()

    def handle_v_error(self):
        noise_l = np.random.normal(1, math.sqrt(self.robot['wheels']['error_variance_left']), 1)
        self.vl = float(self.vl * noise_l)

        noise_r = np.random.normal(1, math.sqrt(self.robot['wheels']['error_variance_right']), 1)
        self.vr = float(self.vr * noise_r)


    def transmit_map(self):
        with open("/home/parallels/ros2_ws/src/project_4/project_4/worlds/snake.world") as f:
            map = yaml.safe_load(f)

        og = OccupancyGrid()
        og.header.frame_id = "world"

        d = []
        h = 1
        w = 0

        for k in map["map"]:
            # self.get_logger().info(k)
            if(k == "\n"):
                h += 1
                w = 0
                continue
            if(k == "#"):
                d.append(100)
            else:
                d.append(0)
            w += 1

        d.reverse()

        for k in range(h):
            temp = d[k*w: k*w + w]
            temp.reverse()
            d[k*w: k*w + w] = temp

        og.data = d
        og.info.height = h
        og.info.width = w
        og.info.resolution = map['resolution']

        self.og = og

        self.map_publisher.publish(og)

class vel_node(Node):
    def __init__(self):
        super().__init__('sim_node')

        self.timer = self.create_timer(0.01, self.handle_drive)

        self.vr_publisher = self.create_publisher(Float64, "/vr", 10)
        self.vl_publisher = self.create_publisher(Float64, "/vl", 10)

    def handle_drive(self):
        vr = Float64()
        vr.data = 0.3

        vl = Float64()
        vl.data = -0.3
        self.vr_publisher.publish(vr)
        self.vl_publisher.publish(vl)


def sim_main(args=None):
    rclpy.init(args=None)

    node = sim_node()
    rclpy.spin(node)
    node.destroy_node()

    rclpy.shutdown()


def vel_main(args=None):
    rclpy.init(args=None)
    
    node = vel_node()
    rclpy.spin(node)
    node.destroy_node()

    rclpy.shutdown()