# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


# NOTE
# occ_data: 0 = black, 1 = unexplored,  
# yaw starts from positive x direction, anti-clockwise is positive, clockwise is negative

from operator import ne
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseStamped
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
import random
from typing import Tuple, Set, Optional

from astar import AStar

# constants
rotatechange = 0.5
speedchange = 0.05
#occ_bins = [-1, 0, 100, 101]
occ_bins = [-1, 0, 50, 100]
map_bg_color = 1
stop_distance = 0.50
front_angle = 30
front_angles = range(-front_angle,front_angle+1,1)
left_front_angles = range(0, front_angle + 1, 1)
right_front_angles = range(-front_angle, 0, 1)
scanfile = 'lidar.txt'
mapfile = 'map.txt'
occfile = 'occ.txt'
lookahead_distance = 0.24
target_error = 0.15
speed = 0.05
robot_r = 0.4
avoid_angle = math.pi/3
TURTLEBOT_WIDTH = 0.3
PID_ANG_VEL_SCALE_FACTOR = 1
PID_DEST_ERROR_THRESHOLD = TURTLEBOT_WIDTH / 5
PURGE_RADIUS = TURTLEBOT_WIDTH / 2
LINEAR_VEL = 0.08
DIRS_WITH_DIAGONALS = [(0,1),(0,-1),(1,0),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1)]
DIRS_WITHOUT_DIAGONALS = [(0,1),(0,-1),(1,0),(-1,0)]
OBSTACLE_AVOIDANCE_RANGE = TURTLEBOT_WIDTH * 0.75
NUM_STEPS_BEFORE_REPLAN = 1000
BFS_WALL_PADDING = 2

def euclidean_dist(a, b):
    return np.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2)

class PathFinder(AStar):
    def __init__(self, traversable: Set[int], matrix: np.ndarray,
            allow_diagonals: bool=True, heuristic=euclidean_dist):
        # traversable is a set of cell values that the finder is allowed to walk on.
        # this check is done in neighbors()
        self.traversable = traversable
        self.matrix = matrix
        if allow_diagonals:
            self.dirs = DIRS_WITH_DIAGONALS
        else:
            self.dirs = DIRS_WITHOUT_DIAGONALS
        self.heuristic = euclidean_dist

    def neighbors(self, n):
        output = []
        dirs = [(0,1),(0,-1),(1,0),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1)]
        for dir in dirs:
            neighbor_x = n[0] + dir[0]
            neighbor_y = n[1] + dir[1]
            if 0 <= neighbor_y < self.matrix.shape[0] \
                and 0 <= neighbor_x < self.matrix.shape[1] \
                and self.matrix[neighbor_y][neighbor_x] in self.traversable:
                output.append((neighbor_x, neighbor_y))
        return output

    def distance_between(self, n1, n2):
        return 1
            
    def heuristic_cost_estimate(self, current, goal):
        return self.heuristic(current, goal)
    
    def is_goal_reached(self, current, goal):
        return current == goal

    def get_path(self, start, goal):
        return self.astar(start, goal)

def save_np_arr(name, arr):
    np.savetxt(name, arr, fmt="%i")

def euler_from_quaternion(x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z # in radians

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')

        # create publisher for moving TurtleBot
        self.publisher_ = self.create_publisher(Twist,'cmd_vel',10)

        # NOTE: publisher for nav2
        # create goal_pose publisher
        self.goal_publisher_ = self.create_publisher(PoseStamped, 'goal_pose', 10)

        # create subscription to track occupancy
        self.occ_subscription = self.create_subscription(
            OccupancyGrid,
            'map',
            self.occ_callback,
            qos_profile_sensor_data)
        # self.get_logger().info('Created occ subscriber')
        self.occ_subscription  # prevent unused variable warning
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer, self)
        self.occdata = np.array([])
        self.occ_count = np.array([])

        self.curr_x = 0
        self.curr_y = 0

        self.odom_subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10)
        # self.get_logger().info('Created odom subscriber')
        self.odom_subscription  # prevent unused variable warning
        # initialize variables
        self.roll = 0
        self.pitch = 0
        self.yaw = 0

        self.scan_subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            qos_profile_sensor_data)
        # self.get_logger().info('Created scan subscriber')
        self.scan_subscription  # prevent unused variable warning
        self.laser_range = np.array([])

        # self.i = 0
        self.has_target = False
        self.path = []

        self.curr_pos_raw = None
        self.dest = None
        self.map_origin = None
        self.map_res = None
        self.y_max = None
        self.x_max = None
        self.tries = 0

    def to_true_scale(self, val) -> Optional[float]:
        if self.map_res is None:
            return None
        return val * self.map_res

    def to_map_scale(self, val) -> Optional[float]:
        if self.map_res is None:
            return None
        return val / self.map_res

    def coords_to_real(self, coordinates):
        if coordinates is None or self.map_origin is None or self.map_res is None:
            return None
        new_coordinates_x = self.to_true_scale(coordinates[0]) + self.map_origin.x 
        new_coordinates_y = self.to_true_scale(coordinates[1]) + self.map_origin.y 
        return (new_coordinates_x, new_coordinates_y)

    def real_to_coords(self, real_position):
        if real_position is None or self.map_origin is None or self.map_res is None:
            return None
        new_coordinates_x = self.to_map_scale(real_position[0] - self.map_origin.x)
        if new_coordinates_x:
            new_coordinates_x = int(new_coordinates_x)
        new_coordinates_y = self.to_map_scale(real_position[1] - self.map_origin.y)
        if new_coordinates_y:
            new_coordinates_y = int(new_coordinates_y)
        return (new_coordinates_x, new_coordinates_y)

    def scan_callback(self, msg):
        # self.get_logger().info('In scan_callback')
        # create numpy array
        self.laser_range = np.array(msg.ranges)
        # print to file
        # np.savetxt(scanfile, self.laser_range)
        # replace 0's with nan
        self.laser_range[self.laser_range==0] = np.nan

    def odom_callback(self, msg):
        # self.get_logger().info('In odom_callback')
        orientation_quat =  msg.pose.pose.orientation
        self.roll, self.pitch, self.yaw = euler_from_quaternion(orientation_quat.x, orientation_quat.y, orientation_quat.z, orientation_quat.w)

    def occ_callback(self, msg):
        print("receiving occ data")
        occdata = np.array(msg.data)
        # calculate total number of bins

        # FROM r2occupancy2
        #     TO REMOVE when actually running (only for checking robot movement)
        # find transform to obtain base_link coordinates in the map frame
        # lookup_transform(target_frame, source_frame, time)
        _, _, binnum = scipy.stats.binned_statistic(occdata, np.nan, statistic='count', bins=occ_bins)

        # binnum go from 1 to 3 so we can use uint8
        # --> 0 = black, 1 = dark gray (unexplored), 2 = light gray (explored), 3 = white (obstacle)
        # convert into 2D array using column order
        odata = np.uint8(binnum.reshape(msg.info.height,msg.info.width))

        # get map resolution
        self.map_res = msg.info.resolution
        # get map origin struct has fields of x, y, and z
        self.map_origin = msg.info.origin.position

        if self.curr_pos_raw is not None:
            grid_x = self.to_map_scale(self.curr_pos_raw[0] - self.map_origin.x)
            if grid_x:
                grid_x = round(grid_x)
            grid_y = self.to_map_scale(self.curr_pos_raw[1] - self.map_origin.y)
            if grid_y:
                grid_y = round(grid_y)
            # set current robot location to 0
            odata[grid_y][grid_x] = 0

        r = int(self.to_map_scale(OBSTACLE_AVOIDANCE_RANGE))
        self.adjusted_map = np.copy(odata)
        for i in range(self.adjusted_map.shape[0]):
            for j in range(self.adjusted_map.shape[1]):
                if odata[i][j] == 3:
                    # make all points of radius r around this point an obstacle as well
                    for k in range(i - r, i + r + 1):
                        for l in range(j - r, j + r + 1):
                            if 0 <= k < self.adjusted_map.shape[0] \
                                and 0 <= l < self.adjusted_map.shape[1] \
                                and euclidean_dist((k, l), (i, j)) <= r:
                                self.adjusted_map[k][l] = 3

        # make msgdata go from 0 instead of -1, reshape into 2D
        # oc2 = occdata + 1
        # reshape to 2D array using column order
        # self.occdata = np.uint8(oc2.reshape(msg.info.height,msg.info.width,order='F'))
        self.occdata = np.uint8(odata.reshape(msg.info.height,msg.info.width))
        # self.occ_count = np.histogram2d(occdata,occ_bins)
        self.occ_count = odata

        self.y_max, self.x_max = self.occdata.shape

        iwidth = msg.info.width
        iheight = msg.info.height
        total_bins = iwidth * iheight
        # FROM r2occupancy2
        #     TO REMOVE when actually running (only for checking robot movement)
        # find transform to obtain base_link coordinates in the map frame
        # lookup_transform(target_frame, source_frame, time)
        occ_counts, edges, binnum = scipy.stats.binned_statistic(occdata, np.nan, statistic='count', bins=occ_bins)
        try:
            trans = self.tfBuffer.lookup_transform('map', 'base_link', rclpy.time.Time())
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().info('No transformation found')
            return
            
        cur_pos = trans.transform.translation
        cur_rot = trans.transform.rotation
        # self.get_logger().info('Trans: %f, %f' % (cur_pos.x, cur_pos.y))
        # convert quaternion to Euler angles
        roll, pitch, yaw = euler_from_quaternion(cur_rot.x, cur_rot.y, cur_rot.z, cur_rot.w)
        # self.get_logger().info('Rot-Yaw: R: %f D: %f' % (yaw, np.degrees(yaw)))

        # get map resolution
        map_res = msg.info.resolution
        # get map origin struct has fields of x, y, and z
        map_origin = msg.info.origin.position
        # get map grid positions for x, y position
        grid_x = round((cur_pos.x - map_origin.x) / map_res)
        grid_y = round(((cur_pos.y - map_origin.y) / map_res))

        # self.get_logger().info('Grid Y: %i Grid X: %i' % (grid_y, grid_x))

        # binnum go from 1 to 3 so we can use uint8
        # --> 0 = black, 1 = dark gray (unexplored), 2 = light gray (explored), 3 = white (obstacle)
        # convert into 2D array using column order
        odata = np.uint8(binnum.reshape(msg.info.height,msg.info.width))
        # set current robot location to 0
        odata[grid_y][grid_x] = 0

        #
        #
        # MAIN

        to_print = np.copy(odata)
        if hasattr(self, 'path'):
            for p in self.path:
                to_print[p[1]][p[0]] = 0

        # odata[0][0] = 3
        # self.get_logger().info('origin: %i, %i' % (round(map_origin.x),round(map_origin.y)))
        # create image from 2D array using PIL
        img = Image.fromarray(to_print)
        # find center of image
        i_centerx = iwidth/2
        i_centery = iheight/2
        # find how much to shift the image to move grid_x and grid_y to center of image
        shift_x = round(grid_x - i_centerx)
        shift_y = round(grid_y - i_centery)
        # self.get_logger().info('Shift Y: %i Shift X: %i' % (shift_y, shift_x))

        # pad image to move robot position to the center
        # adapted from https://note.nkmk.me/en/python-pillow-add-margin-expand-canvas/ 
        left = 0
        right = 0
        top = 0
        bottom = 0
        if shift_x > 0:
            # pad right margin
            right = 2 * shift_x
        else:
            # pad left margin
            left = 2 * (-shift_x)
            
        if shift_y > 0:
            # pad bottom margin
            bottom = 2 * shift_y
        else:
            # pad top margin
            top = 2 * (-shift_y)
            
        # create new image
        new_width = iwidth + right + left
        new_height = iheight + top + bottom
        img_transformed = Image.new(img.mode, (new_width, new_height), map_bg_color)
        img_transformed.paste(img, (left, top))

        # rotate by 90 degrees so that the forward direction is at the top of the image
        # rotated = img_transformed.rotate(np.degrees(yaw)-90, expand=True, fillcolor=map_bg_color)

        # show the image using grayscale map
        # plt.imshow(img, cmap='gray', origin='lower')
        # plt.imshow(img_transformed, cmap='gray', origin='lower')
        plt.imshow(img_transformed, cmap='gray', origin='lower')
        plt.draw_all()
        # pause to make sure the plot gets created
        plt.pause(0.00000000001)
        #END FROM r2occupancy2

    def bfs(self, curr_pos):
        matrix = self.occ_count
        x = curr_pos[0]
        y = curr_pos[1]
        directions = DIRS_WITH_DIAGONALS
        queue = deque([(x, y, 0)])  # (x, y, distance)
        visited = set()
        visited.add((x, y))

        while queue:
            curr_x, curr_y, distance = queue.popleft()

            if matrix[curr_y][curr_x] == 1 and distance > 10:
                # if random.randint(0, 2) == 0:
                return (curr_x, curr_y)

            for dx, dy in directions:
                new_x, new_y = curr_x + dx, curr_y + dy

                if 0 <= new_y < len(matrix) \
                    and 0 <= new_x < len(matrix[0]) \
                    and (new_x, new_y) not in visited \
                    and matrix[new_y][new_x] < 3 \
                    and not self.has_neighboring_wall((new_x, new_y), BFS_WALL_PADDING):
                        visited.add((new_x, new_y))
                        queue.append((new_x, new_y, distance + 1))
        return None

    def has_neighboring_wall(self, point, padding):
        for x in range(point[0] - padding, point[0] + padding):
            for y in range(point[1] - padding, point[1] + padding):
                if self.occ_count[y][x] == 3:
                    return True
        return False
    
    # checks the line from the robot's current position to the given point
    # returns True if there is an obstacle along this line, else returns False
    def is_occluded(self, point):
        curr = self.get_curr_pos()
        delta_x, delta_y = point[0] - curr[0], point[1] - curr[1]
        dydx = delta_y / delta_x
        for dx in range(delta_x):
            x = int(curr[0] + dx)
            y = int(curr[1] + dx * dydx)
            if self.occ_count[y][x] == 3:
                return True
        return False

    ## finds the nearest undiscovered cell around the robot
    def get_next_unknown_cell(self, is_random=False):
        if not is_random:
            curr_pos = self.get_curr_pos()
            return self.bfs(curr_pos)
        x = random.randint(0, self.x_max)
        y = random.randint(0, self.y_max)
        while self.occ_count[y][x] != 1:
            x = random.randint(0, self.x_max)
            y = random.randint(0, self.y_max)
        return (x, y)

    def stopbot(self):
        t = Twist()
        self.publisher_.publish(t)

    def get_curr_pos(self):
        return self.real_to_coords(self.curr_pos_raw)
        
    def get_transform(self):
        trans = None
        try:
            trans = self.tfBuffer.lookup_transform('map', 'base_link', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.01))
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().info('No transformation found')
            print(f"Transform error: {e}")
        return trans

    # NOTE: publishing to nav2
    # expects a (x, y) tuple for position
    def set_goal_position(self, position):
        if not position:
            return
        poseStamped = PoseStamped()
        poseStamped.header.frame_id = 'map'
        poseStamped.header.stamp.sec = 0
        poseStamped.pose.position.x = float(position[0])
        poseStamped.pose.position.y = float(position[1])
        # poseStamped.pose.orientation.z = 1.0
        self.goal_publisher_.publish(poseStamped)

    def mover(self):
        try:
            while rclpy.ok():
                rclpy.spin_once(self)

                trans = self.get_transform()
                if trans is not None:
                    cur_pos = trans.transform.translation
                    cur_rot = trans.transform.rotation
                    _, _, self.yaw = euler_from_quaternion(cur_rot.x, cur_rot.y, cur_rot.z, cur_rot.w)

                    # This is the raw position, not scaled yet cos we might not have the scale factor yet
                    self.curr_pos_raw = (cur_pos.x, cur_pos.y)

                if self.curr_pos_raw is None or self.map_origin is None or self.map_res is None:
                    continue
                
                # NOTE: main loop
                if self.tries > 10000 or not self.dest or euclidean_dist(self.get_curr_pos(), self.dest) < 1:
                    cell_to_explore = self.get_next_unknown_cell(True)
                    self.dest = cell_to_explore
                    if cell_to_explore is None:
                        print("exploration complete")
                        self.stopbot()
                        return
                    cell_to_explore = self.coords_to_real(cell_to_explore)
                    self.set_goal_position(cell_to_explore)
                    self.tries = 0
                self.tries += 1
                print(f"CELL VALUES next goal: {self.dest}; curr pos: {self.get_curr_pos()}")
                print(f"COORDS VALUES next goal: {self.coords_to_real(self.dest)}; curr pos: {self.curr_pos_raw}")
        except Exception as e:
            print(e)
        finally:
            self.stopbot()

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    for _ in range(10):
        rclpy.spin_once(minimal_subscriber)
    minimal_subscriber.mover()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
