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
robot_r = 0.2
avoid_angle = math.pi/3
TURTLEBOT_WIDTH = 0.2
PID_ANG_VEL_SCALE_FACTOR = 1
PID_DEST_ERROR_THRESHOLD = TURTLEBOT_WIDTH / 5
PURGE_RADIUS = TURTLEBOT_WIDTH / 2
LINEAR_VEL = 0.08
DIRS_WITH_DIAGONALS = [(0,1),(0,-1),(1,0),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1)]
DIRS_WITHOUT_DIAGONALS = [(0,1),(0,-1),(1,0),(-1,0)]
OBSTACLE_AVOIDANCE_RANGE = TURTLEBOT_WIDTH * 0.25
NUM_STEPS_BEFORE_REPLAN = 1000
NUM_RETRIES = 100

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

        # create subscriber for check for when to kick in astar
        # purely for the rpi to tell the pc to start astar once it is done line following 
        self.start_subscription = self.create_subscription(
            Twist,
            'start_astar',
            self.set_bool,
            qos_profile_sensor_data)
        self.start_subscription
        self.start_exploring = False

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
        self.retries = 0

    def to_true_scale(self, val):
        if self.map_res is None:
            return None
        return val * self.map_res

    def to_map_scale(self, val):
        if self.map_res is None:
            return None
        return val / self.map_res

    def coords_to_real(self, coordinates):
        if real_position is None or self.map_origin is None or self.map_res is None:
            return None
        new_coordinates_x = self.to_true_scale(coordinates[0]) + self.map_origin.x 
        new_coordinates_y = self.to_true_scale(coordinates[1]) + self.map_origin.y 
        return (new_coordinates_x, new_coordinates_y)

    def real_to_coords(self, real_position):
        if real_position is None or self.map_origin is None or self.map_res is None:
            return None
        new_coordinates_x = int(self.to_map_scale(real_position[0] - self.map_origin.x))
        new_coordinates_y = int(self.to_map_scale(real_position[1] - self.map_origin.y))
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
        occdata = np.array(msg.data)
        # calculate total number of bins

        # FROM r2occupancy2
        #     TO REMOVE when actually running (only for checking robot movement)
        # find transform to obtain base_link coordinates in the map frame
        # lookup_transform(target_frame, source_frame, time)
        occ_counts, edges, binnum = scipy.stats.binned_statistic(occdata, np.nan, statistic='count', bins=occ_bins)

        # binnum go from 1 to 3 so we can use uint8
        # --> 0 = black, 1 = dark gray (unexplored), 2 = light gray (explored), 3 = white (obstacle)
        # convert into 2D array using column order
        odata = np.uint8(binnum.reshape(msg.info.height,msg.info.width))

        # get map resolution
        self.map_res = msg.info.resolution
        # get map origin struct has fields of x, y, and z
        self.map_origin = msg.info.origin.position

        if self.curr_pos_raw is not None:
            grid_x = round(self.to_map_scale(self.curr_pos_raw[0] - self.map_origin.x))
            grid_y = round(self.to_map_scale(self.curr_pos_raw[1] - self.map_origin.y))
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
        oc2 = occdata + 1
        # reshape to 2D array using column order
        # self.occdata = np.uint8(oc2.reshape(msg.info.height,msg.info.width,order='F'))
        self.occdata = np.uint8(oc2.reshape(msg.info.height,msg.info.width))
        # self.occ_count = np.histogram2d(occdata,occ_bins)
        self.occ_count = odata

    def set_bool(self, msg):
        self.start_exploring = True

    def find_non_obstacle(self, start):
        matrix = self.adjusted_map
        x = start[0]
        y = start[1]
        directions = [(0, 1), (0, -1), (1, 0), (-1, 0)]
        queue = deque([(x, y, 0)])  # (x, y, distance)
        visited = set()
        visited.add((x, y))

        while queue:
            curr_x, curr_y, distance = queue.popleft()

            if matrix[curr_y][curr_x] < 3:
                return (curr_x, curr_y)

            for dx, dy in directions:
                new_x, new_y = curr_x + dx, curr_y + dy

                if 0 <= new_y < len(matrix) \
                    and 0 <= new_x < len(matrix[0]) \
                    and (new_x, new_y) not in visited:
                        visited.add((new_x, new_y))
                        queue.append((new_x, new_y, distance + 1))
        return None

    def bfs(self, curr_pos):
        matrix = self.occ_count
        x = curr_pos[0]
        y = curr_pos[1]
        directions = [(0, 1), (0, -1), (1, 0), (-1, 0)]
        queue = deque([(x, y, 0)])  # (x, y, distance)
        visited = set()
        visited.add((x, y))

        while queue:
            curr_x, curr_y, distance = queue.popleft()

            if matrix[curr_y][curr_x] == 1:
                return (curr_x, curr_y)

            for dx, dy in directions:
                new_x, new_y = curr_x + dx, curr_y + dy

                if 0 <= new_y < len(matrix) and 0 <= new_x < len(matrix[0]) and (new_x, new_y) not in visited and matrix[new_y][new_x] < 3:
                        visited.add((new_x, new_y))
                        queue.append((new_x, new_y, distance + 1))
        return None

    def astar(self, array, start, goal):
        # array: occ_count data
        # print('currently in astar')
        #neighbors = [(0,1),(0,-1),(1,0),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1)]
        neighbors = [(0,1),(0,-1),(1,0),(-1,0)]
        close_set = set()
        came_from = {}
        gscore = {start:0}
        fscore = {start:euclidean_dist(start, goal)}
        oheap = []
        heapq.heappush(oheap, (fscore[start], start))
        while oheap:
            current = heapq.heappop(oheap)[1] # pops top of heap, accesses second element ((row, col) coordinates of point)
            if current == goal:
                data = []
                while current in came_from:
                    data.append(current)
                    current = came_from[current]
                # data holds the path taken to the goal
                data = data + [start]
                data = data[::-1] # reverses order of elements so data returns a list [start, ..., goal] of the path taken
                # data = [self.coords_to_real(p) for p in data]
                # print('exiting astar')
                return data
            # if goal has not been reached ...
            close_set.add(current)
            for i, j in neighbors:
                neighbor = current[0] + i, current[1] + j
                tentative_g_score = gscore[current] + euclidean_dist(current, neighbor)
                if 0 <= neighbor[0] < array.shape[0]: # checks if neighbour's y coordinate is within range
                    if 0 <= neighbor[1] < array.shape[1]:                # checks if neighbour's x coordinate is within range
                        # if array[neighbor[0]][neighbor[1]] == 1:
                        if array[neighbor[0]][neighbor[1]] == 3:
                            continue
                    else:
                        # array bound y walls
                        continue
                else:
                    # array bound x walls
                    continue
                # skips to next neighbour if not within image or is an obstacle
                if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, 0):
                    # neighbour already seen and not a shorter path
                    continue
                if  tentative_g_score < gscore.get(neighbor, 0) or neighbor not in [i[1]for i in oheap]:
                    # there exists a shorter path to neighbour, or neighbour has not been visited
                    # then save all your shit
                    came_from[neighbor] = current
                    gscore[neighbor] = tentative_g_score
                    fscore[neighbor] = tentative_g_score + euclidean_dist(neighbor, goal)
                    heapq.heappush(oheap, (fscore[neighbor], neighbor))
        # If no path to goal was found, return closest path to goal
        if goal not in came_from:
            closest_node = None
            closest_dist = float('inf')
            for node in close_set:
                # runs through every node that has had a path calculated, returns the path to the node closest to the goal
                dist = euclidean_dist(node, goal)
                if dist < closest_dist:
                    closest_node = node
                    closest_dist = dist
            if closest_node is not None:
                data = []
                while closest_node in came_from:
                    data.append(closest_node)
                    closest_node = came_from[closest_node]
                data = data + [start]
                data = data[::-1]
                # data = [self.coords_to_real(p) for p in data]
                # print('exiting astar')
                return data
        # print('exiting astar')
        return False

    ## finds the nearest undiscovered cell around the robot
    def get_next_unknown_cell(self):
        curr_pos = self.get_curr_pos()
        goal_values = set([1])
        traversable = set([2])
        return self.bfs(curr_pos)

    def get_path_to(self, dest: Tuple[int, int]) -> Tuple[int, int]:
        curr_pos = self.get_curr_pos()
        if self.adjusted_map[curr_pos[1]][curr_pos[0]] == 3:
            curr_pos = self.find_non_obstacle(curr_pos)
        if self.adjusted_map[dest[1]][dest[0]] == 3:
            dest = self.find_non_obstacle(dest)
        traversable = set([1, 2])
        path = PathFinder(traversable, self.adjusted_map).get_path(curr_pos, dest)
        if path is None:
            print(f"curr pos val: {self.adjusted_map[curr_pos[1]][curr_pos[0]]}")
            print(f"dest val: {self.adjusted_map[dest[1]][dest[0]]}")
            print(f"path from {curr_pos} to {dest} could not be plotted")
        return list(path)

    def stopbot(self):
        t = Twist()
        self.publisher_.publish(t)

    def get_curr_pos(self):
        return self.real_to_coords(self.curr_pos_raw)
        
    def go_towards(self, dest):
        if self.curr_pos_raw is None or dest is None:
            return
        curr = self.get_curr_pos()
        if curr is None:
            return
        if euclidean_dist(curr, dest) > self.to_map_scale(PID_DEST_ERROR_THRESHOLD):
            dest_yaw = math.atan2(dest[1] - curr[1], dest[0] - curr[0])
            ang_delta = dest_yaw - self.yaw

            # keep the magnitude of ang_delta less than pi
            if ang_delta > math.pi:
                ang_delta -= (2 * math.pi)
            elif ang_delta < -math.pi:
                ang_delta += (2 * math.pi)

            # print(f"curr_yaw: {self.yaw}; dest_yaw: {dest_yaw}; ang_delta: {ang_delta}")

            twist = Twist()
            twist.linear.x = LINEAR_VEL
            twist.angular.z = ang_delta * PID_ANG_VEL_SCALE_FACTOR
            self.publisher_.publish(twist)
        else:
            self.dest = None
            self.stopbot()

    def get_transform(self):
        trans = None
        try:
            trans = self.tfBuffer.lookup_transform('map', 'base_link', rclpy.time.Time())
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().info('No transformation found')
            print(f"Transform error: {e}")
        return trans

    def purge_traversed_cells_from_path(self):
        new_path = []
        curr_pos = self.get_curr_pos()
        for cell in self.path:
            if euclidean_dist(curr_pos, cell) > self.to_map_scale(PURGE_RADIUS):
                new_path.append(cell)
        self.path = new_path

    def laser_range_conv(self, angle):
        return int(angle / 360 * len(self.laser_range))

    def object_avoidance(self):
        # print('in object avoidance')
        v = None
        w = None
        for i in range(self.laser_range_conv(60)):
        # for i in range(45): # to account for funky lidar
            if self.laser_range[i] < robot_r:
                print('OBJECT: avoiding front left')
                v = speed
                w = -math.pi/4 
                break
        if v == None:
            for i in range(self.laser_range_conv(300),self.laser_range_conv(360)):
            # for i in range(225, 270):
                if self.laser_range[i] < robot_r:
                    print('OBJECT: avoiding front right')
                    v = speed
                    w = math.pi/4
                    break
        return v,w

    def mover(self):
        try:
            while rclpy.ok():
            # while rclpy.ok() and self.retries < NUM_RETRIES:
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

                if self.laser_range is not None:
                    v, w = self.object_avoidance()
                    if v != None:
                        twist = Twist()
                        twist.linear.x = v
                        twist.angular.z = w
                        self.publisher_.publish(twist)
                        time.sleep(0.2)
                        continue
                
                try:

                    self.purge_traversed_cells_from_path()

                    if not self.path or self.steps_taken > NUM_STEPS_BEFORE_REPLAN:
                        print("REPLANNING")
                        cell_to_explore = self.get_next_unknown_cell()
                        if cell_to_explore is None:
                            print("exploration complete")
                            return
                        self.path = self.get_path_to(cell_to_explore)
                        self.steps_taken = 0

                    if self.path is None:
                        # failed, so retry
                        self.retries += 1
                        continue
                    else:
                        # succeeded, so reset num_retries
                        self.retries = 0

                    if self.dest is None:
                        self.dest = self.path[0]
                    self.go_towards(self.dest)
                    self.steps_taken += 1
                except:
                    self.retries += 1
                """
                if self.dest is None:
                    dest_x = int(input("dest_x: "))
                    dest_y = int(input("dest_y: "))
                    self.dest = (dest_x, dest_y)
                self.go_towards(self.dest)
                """
            if self.retries >= NUM_RETRIES:
                print(f"Failed {NUM_RETRIES} times. Going forward.")
                twist = Twist()
                twist.linear.x = 0.05
                twist.angular.z = 0
                self.publisher_.publish(twist)
        except Exception as e:
            print(e)
        finally:
            self.stopbot()
    
    def starter(self):
        while rclpy.ok():
            rclpy.spin_once(self)
            if (self.start_exploring):
                break
            else:
                print('ur dad')

    def decider(self):
        # starting: get initial position and turn into maze
        # self.starter()
        # the killer maze mapping part :")
        self.mover()
        # end of maze, http call to server and move to door based on saved initial coords

        # in room, find bucket and activate servo

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin_once(minimal_subscriber)
    minimal_subscriber.decider()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
