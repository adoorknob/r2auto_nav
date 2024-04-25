import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from collections import deque

#for plotting
import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
import matplotlib.pyplot as plt
from PIL import Image

import numpy as np
import math
import cmath
import time
import scipy.stats
import heapq, math, random
import sys
from typing import Tuple, Set

angle_range = 20
PID_THRESHOLD = 0.3
LINEAR_VEL = 0.05
ANGULAR_VEL = 0.5

class WallFollower(Node):
    def __init__(self):
        super().__init__('wallfollower')

        self.scan_subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            qos_profile_sensor_data)
        # self.get_logger().info('Created scan subscriber')
        self.scan_subscription  # prevent unused variable warning
        self.laser_range = np.array([])

        self.publisher_ = self.create_publisher(Twist,'cmd_vel',10)
        self.initial_distance = None

    def scan_callback(self, msg):
        # self.get_logger().info('In scan_callback')
        # create numpy array
        self.laser_range = np.array(msg.ranges)
        # print to file
        # np.savetxt(scanfile, self.laser_range)
        # replace 0's with nan
        self.laser_range[self.laser_range==0] = np.nan

    def stopbot(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.publisher_.publish(twist)

    def ang_to_laser(self, angle):
        return int(angle/360 * len(self.laser_range)) 

    def mover(self):
        try:
            # initialize variable to write elapsed time to file
            # contourCheck = 1

            # find direction with the largest distance from the Lidar,
            # rotate to that direction, and start moving

            while rclpy.ok():
                if self.laser_range.size != 0:
                    min_distance = min(self.laser_range[range(self.ang_to_laser(270 - angle_range), self.ang_to_laser(270 + angle_range))])

                    if self.initial_distance is None:
                        self.initial_distance = min_distance 

                    path_error = min_distance - self.initial_distance
                    print(f'path error: {path_error}') 

                    if abs(path_error) < PID_THRESHOLD and path_error != np.nan:
                        t = Twist()
                        t.linear.x = LINEAR_VEL
                        t.angular.z = -ANGULAR_VEL * 2 * path_error/self.initial_distance
                        self.publisher_.publish(t)
                        print('correcting path')

                # allow the callback functions to run
                rclpy.spin_once(self)

        except Exception as e:
            print(e)
        
        # Ctrl-c detected
        finally:
            # stop moving
            self.stopbot()

def main(args=None):
    rclpy.init(args=args)

    wallfollower = WallFollower()
    wallfollower.mover()


    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    wallfollower.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
