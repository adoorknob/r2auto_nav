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
speed = 0.08
avoid_speed = 0.02
robot_r = 0.2
robot_r_angle = math.pi/4
NUM_RETRIES = 50000

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
        self.tries = 0

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

    def coords_to_real(self, coordinates):
        new_coordinates_x = coordinates[0]*self.map_res + self.map_origin.x 
        new_coordinates_y = coordinates[1]*self.map_res + self.map_origin.y 
        return (new_coordinates_x, new_coordinates_y)

    def funky_lidar_eqv_of(self, angle):
        return int(angle/360 * len(self.laser_range))

    def get_transform(self):
        trans = None
        try:
            trans = self.tfBuffer.lookup_transform('map', 'base_link', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.01))
            # trans = self.tfBuffer.lookup_transform('map', 'base_link', rclpy.time.Time())
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().info('No transformation found')
            # print(f"Transform error: {e}")
        return trans

        # get map resolution
        map_res = self.map_res
        # get map origin struct has fields of x, y, and z
        map_origin = self.map_origin
        # get map grid positions for x, y position
        self.cur_pos = cur_pos

    def draw_map(self, draw_path):
        to_print = np.copy(self.occ_count)
        to_print[self.curr_y][self.curr_x] = 0
        iwidth = self.iwidth
        iheight = self.iheight
        
        # check if path exists and want to draw path
        if hasattr(self, 'path') and draw_path:
            for p in self.path:
                to_print[p[1]][p[0]] = 0

        img = Image.fromarray(to_print)
        # find center of image
        i_centerx = iwidth/2
        i_centery = iheight/2
        # find how much to shift the image to move grid_x and grid_y to center of image
        shift_x = round(self.curr_x - i_centerx)
        shift_y = round(self.curr_y - i_centery)
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
        plt.imshow(img_transformed, cmap='gray', origin='lower')
        # plt.imshow(rotated, cmap='gray', origin='lower')
        plt.draw_all()
        # pause to make sure the plot gets created
        plt.pause(0.00000000001)

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
        self.get_logger().info('In occ_callback')
        # create numpy array
        occdata = np.array(msg.data)
        # compute histogram to identify percent of bins with -1
        # self.occ_count = np.histogram(msgdata,occ_bins)
        # calculate total number of bins
        iwidth = msg.info.width
        iheight = msg.info.height
        total_bins = iwidth * iheight

        self.iwidth = iwidth
        self.iheight = iheight
        # log the info
        # self.get_logger().info('Unmapped: %i Unoccupied: %i Occupied: %i Total: %i' % (occ_counts[0][0], occ_counts[0][1], occ_counts[0][2], total_bins))

        # FROM r2occupancy2
        #     TO REMOVE when actually running (only for checking robot movement)
        # find transform to obtain base_link coordinates in the map frame
        # lookup_transform(target_frame, source_frame, time)
        occ_counts, edges, binnum = scipy.stats.binned_statistic(occdata, np.nan, statistic='count', bins=occ_bins)
        # get map resolution
        map_res = msg.info.resolution
        # get map origin struct has fields of x, y, and z
        map_origin = msg.info.origin.position
        self.map_res = map_res
        self.map_origin = map_origin

        # self.get_logger().info('Grid Y: %i Grid X: %i' % (grid_y, grid_x))

        # binnum go from 1 to 3 so we can use uint8
        # --> 0 = black, 1 = dark gray (unexplored), 2 = light gray (explored), 3 = white (obstacle)
        # convert into 2D array using column order
        odata = np.uint8(binnum.reshape(msg.info.height,msg.info.width))
        # set current robot location to 0
        self.occ_count = odata

    def bfs(self):
        matrix = self.occ_count
        x = self.curr_x
        y = self.curr_y
        exclude = self.exclude
    #def bfs(matrix, x, y):
        # print('currently in bfs')
        # np.savetxt('matrix.txt', matrix)
        directions = [(0, 1), (0, -1), (1, 0), (-1, 0)]
        queue = deque([(x, y, 0)])  # (x, y, distance)
        visited = set()
        visited.add((x, y))

        while queue:
            curr_x, curr_y, distance = queue.popleft()
            # print('bfs check on (' + str(curr_x) + ', ' + str(curr_y) + '): ' + str(matrix[curr_y][curr_x]))

            if matrix[curr_y][curr_x] == 1 and (curr_x, curr_y) not in exclude:
            # if matrix[curr_y][curr_x] < 0 and (curr_x, curr_y) not in exclude:
            #if matrix[curr_y][curr_x] < 0 and (curr_x, curr_y):
                # return (curr_x, curr_y), distance
                # print('exiting bfs')
                # print('target value: ' + str(matrix[curr_y][curr_x]))
                return (curr_x, curr_y)

            for dx, dy in directions:
                new_x, new_y = curr_x + dx, curr_y + dy

                if 0 <= new_y < len(matrix) and 0 <= new_x < len(matrix[0]) and (new_x, new_y) not in visited and (new_x, new_y) not in exclude and matrix[new_y][new_x] < 3:
                # if 0 <= new_x < len(matrix) and 0 <= new_y < len(matrix[0]) and (new_x, new_y) not in visited and (new_x, new_y) and matrix[new_y][new_x] < 50:
                    # print('passed if statement, new_x = ' + str(new_x) + ', new_y = ' + str(new_y))
                        visited.add((new_x, new_y))
                        queue.append((new_x, new_y, distance + 1))
        return None

    def heuristic(self, a, b):
        return np.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2)

    def astar(self):
        array = self.occ_count
        start = (self.curr_x, self.curr_y)
        goal = self.target
        # array: occ_count data
        # print('currently in astar')
        #neighbors = [(0,1),(0,-1),(1,0),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1)]
        neighbors = [(0,1),(0,-1),(1,0),(-1,0)]
        close_set = set()
        came_from = {}
        gscore = {start:0}
        fscore = {start:self.heuristic(start, goal)}
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
                tentative_g_score = gscore[current] + self.heuristic(current, neighbor)
                '''
                if 0 <= neighbor[0] < array.shape[0]: # checks if neighbour's y coordinate is within range
                    if 0 <= neighbor[1] < array.shape[1]:                # checks if neighbour's x coordinate is within range
                        # if array[neighbor[0]][neighbor[1]] == 1:
                        if array[neighbor[0]][neighbor[1]] == 3:
                '''
                if 0 <= neighbor[1] < array.shape[0]: # checks if neighbour's y coordinate is within range
                    if 0 <= neighbor[0] < array.shape[1]:                # checks if neighbour's x coordinate is within range
                        # if array[neighbor[0]][neighbor[1]] == 1:
                        if array[neighbor[1]][neighbor[0]] == 3:
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
                    fscore[neighbor] = tentative_g_score + self.heuristic(neighbor, goal)
                    heapq.heappush(oheap, (fscore[neighbor], neighbor))
        # If no path to goal was found, return closest path to goal
        if goal not in came_from:
            closest_node = None
            closest_dist = float('inf')
            for node in close_set:
                # runs through every node that has had a path calculated, returns the path to the node closest to the goal
                dist = self.heuristic(node, goal)
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

    def pure_pursuit(self):
        # print('in pure pursuit')
        current_x = self.curr_x
        current_y = self.curr_y
        current_heading = self.yaw
        path = self.path
        global lookahead_distance
        closest_point = None
        v = speed
        closest_distance = float('inf')
        index = self.i

        '''
        for i in range(self.i,len(path)):
            # looks for point in path that distance > lookahead to current position
            x = path[i][1]
            y = path[i][0]
            distance = math.hypot(current_x - x, current_y - y) # euclidean distance from point that robot is heading to
            if lookahead_distance < distance:
                closest_point = (x, y)
                self.i = i
                print('updated self.i to ' + str(i) + ': ' + str(self.i))
                break
        '''
        for i in range(index, len(path)):
        #for i in range(self.i,len(path)):
            # looks for point in path that distance > lookahead to current position
            x = path[i][0]
            y = path[i][1]
            distance = math.hypot(current_x - x, current_y - y) # euclidean distance from point that robot is heading to
            if distance < closest_distance:
                closest_point = (x, y)
                closest_distance = distance
                index = i
        
        # print('closest point: ' + str(path[index]) + ' with distance ' + str(closest_distance))
        self.i = index
        if closest_point is not None:
            target_heading = math.atan2(closest_point[1] - current_y, closest_point[0] - current_x)
            desired_steering_angle = target_heading - current_heading
        else:
            # no point within lookahead_distance, just go towards path target
            target_heading = math.atan2(path[-1][1] - current_y, path[-1][0] - current_x)
            desired_steering_angle = target_heading - current_heading
            self.i = len(path)-1
        # desired_steering_angle += math.pi

        # keeps angles within 360
        if desired_steering_angle > math.pi:
            desired_steering_angle -= 2 * math.pi
        elif desired_steering_angle < -math.pi:
            desired_steering_angle += 2 * math.pi

        if desired_steering_angle > math.pi/6 or desired_steering_angle < -math.pi/6:
            sign = 1 if desired_steering_angle > 0 else -1
            desired_steering_angle = sign * math.pi/4
            v = 0.0
        # print('ending pure pursuit')
        return v,desired_steering_angle

    def mover(self):
        try:
            while rclpy.ok():
                rclpy.spin_once(self)
                '''
                if self.laser_range.size != 0:
                    # check distances in front of TurtleBot and find values less
                    # than stop_distance
                    lri = (self.laser_range[front_angles]<float(stop_distance)).nonzero()
                    # self.get_logger().info('Distances: %s' % str(lri))

                    # if the list is not empty
                    if(len(lri[0])>0):
                        # stop moving
                        self.stopbot()
                        # find direction with the largest distance from the Lidar
                        # rotate to that direction
                        # start moving
                        self.pick_direction()
                '''
                if np.size(self.occ_count) != 0 and np.size(self.laser_range)!= 0:
                    self.integration()
        except Exception as e:
            print(e)
        
        # Ctrl-c detected
        finally:
            # stop moving
            self.stopbot()

    '''        
    def avoid(self, left):
        if left:
            self.rotatebot(-rotatechange)
            # self.rotatebot(-avoid_angle)
            print('avoiding left')
        else:
            self.rotatebot(rotatechange)
            # self.rotatebot(avoid_angle)
            print('avoiding right')
        twist = Twist()
        twist.linear.x = speedchange
        self.publisher_.publish(twist)
        time.sleep(0.1)
        rclpy.spin_once(self)
        self.path = self.astar()
    '''

    def object_avoidance(self):
        # print('in object avoidance')
        v = None
        w = None
        for i in range(self.funky_lidar_eqv_of(60)):
            if self.laser_range[i] < robot_r:
                print('OBJECT: avoiding front left')
                v = avoid_speed
                w = -robot_r_angle 
                break
        if v == None:
            for i in range(self.funky_lidar_eqv_of(300),self.funky_lidar_eqv_of(360)):
                if self.laser_range[i] < robot_r:
                    print('OBJECT: avoiding front right')
                    v = avoid_speed
                    w = robot_r_angle
                    break
        return v,w
    '''
    def pick_direction(self):
        # self.get_logger().info('In pick_direction')
        if self.laser_range.size != 0:
            # use nanargmax as there are nan's in laser_range added to replace 0's
            lr2i = np.nanargmax(self.laser_range)
            self.get_logger().info('Picked direction: %d %f m' % (lr2i, self.laser_range[lr2i]))
        else:
            lr2i = 0
            self.get_logger().info('No data!')

        # rotate to that direction
        self.rotatebot(float(lr2i))

        # start moving
        self.get_logger().info('Start moving')
        twist = Twist()
        twist.linear.x = speedchange
        twist.angular.z = 0.0
        # not sure if this is really necessary, but things seem to work more
        # reliably with this
        time.sleep(1)
        self.publisher_.publish(twist)
    '''

    def stopbot(self):
        # self.get_logger().info('In stopbot')
        # publish to cmd_vel to move TurtleBot
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        # time.sleep(1)
        self.publisher_.publish(twist)

    def rotatebot(self, rot_angle):
        # self.get_logger().info('In rotatebot')
        # create Twist object
        twist = Twist()
        
        # get current yaw angle
        current_yaw = self.yaw
        # log the info
        self.get_logger().info('Current: %f' % math.degrees(current_yaw))
        # we are going to use complex numbers to avoid problems when the angles go from
        # 360 to 0, or from -180 to 180
        c_yaw = complex(math.cos(current_yaw),math.sin(current_yaw))
        # calculate desired yaw
        target_yaw = current_yaw + math.radians(rot_angle)
        # convert to complex notation
        c_target_yaw = complex(math.cos(target_yaw),math.sin(target_yaw))
        self.get_logger().info('Desired: %f' % math.degrees(cmath.phase(c_target_yaw)))
        # divide the two complex numbers to get the change in direction
        c_change = c_target_yaw / c_yaw
        # get the sign of the imaginary component to figure out which way we have to turn
        c_change_dir = np.sign(c_change.imag)
        # set linear speed to zero so the TurtleBot rotates on the spot
        twist.linear.x = 0.0
        # set the direction to rotate
        twist.angular.z = c_change_dir * rotatechange
        # start rotation
        self.publisher_.publish(twist)

        # we will use the c_dir_diff variable to see if we can stop rotating
        c_dir_diff = c_change_dir
        # self.get_logger().info('c_change_dir: %f c_dir_diff: %f' % (c_change_dir, c_dir_diff))
        # if the rotation direction was 1.0, then we will want to stop when the c_dir_diff
        # becomes -1.0, and vice versa
        while(c_change_dir * c_dir_diff > 0):
            # allow the callback functions to run
            rclpy.spin_once(self)
            current_yaw = self.yaw
            # convert the current yaw to complex form
            c_yaw = complex(math.cos(current_yaw),math.sin(current_yaw))
            # self.get_logger().info('Current Yaw: %f' % math.degrees(current_yaw))
            # get difference in angle between current and target
            c_change = c_target_yaw / c_yaw
            # get the sign to see if we can stop
            c_dir_diff = np.sign(c_change.imag)
            # self.get_logger().info('c_change_dir: %f c_dir_diff: %f' % (c_change_dir, c_dir_diff))

        self.get_logger().info('End Yaw: %f' % math.degrees(current_yaw))
        # set the rotation speed to 0
        twist.angular.z = 0.0
        # stop the rotation
        self.publisher_.publish(twist)

    def integration(self):
        print('in main')
        '''
        right = (self.laser_range[right_front_angles]<float(stop_distance)).nonzero()
        left = (self.laser_range[left_front_angles]<float(stop_distance)).nonzero()
        # self.get_logger().info('Distances: %s' % str(lri))

        # if the list is not empty
        if(len(right[0])>0):
            # stop moving
            self.stopbot()
            self.avoid(False) 
        elif(len(left[0])>0):
            self.stopbot()
            self.avoid(True)
        '''
        trans = self.get_transform()
        if trans is not None:
            cur_pos = trans.transform.translation
            cur_rot = trans.transform.rotation
            _, _, self.yaw = euler_from_quaternion(cur_rot.x, cur_rot.y, cur_rot.z, cur_rot.w)

            grid_x = round((cur_pos.x - self.map_origin.x) / self.map_res)
            grid_y = round(((cur_pos.y - self.map_origin.y) / self.map_res))
            self.curr_x = grid_x
            self.curr_y = grid_y
            # This is the raw position, not scaled yet cos we might not have the scale factor yet
            self.cur_pos = cur_pos

        if self.cur_pos is None or self.map_origin is None or self.map_res is None:
            continue

        if hasattr(self, 'curr_x'):
            self.draw_map(True)

        v, w = self.object_avoidance()
        # checks if there are objects in the way

        if v != None:
            # if there are obstacles
            twist = Twist()
            twist.linear.x = v
            twist.angular.z = w
            self.publisher_.publish(twist)
        else:
            # if no obstacles
            if not self.has_target or self.tries > NUM_RETRIES:
                # if robot doesn't have target, or if it tries too many iterations
                self.exclude = set()
                if self.has_target:
                    # has target but tried too many times
                    print('exceeded num tries')
                    self.exclude.add(self.target)
                print('finding new target')
                self.target = self.bfs()
                if not self.target:
                    # bfs only returns no target if all unexplored are unreachable
                    print('exploration complete')
                    sys.exit
                # grid_target_x = round(((target[0] - map_origin.x) / map_res))
                # grid_target_y = round(((target[1] - map_origin.y) / map_res))
                # print('target value: ' + str(odata[grid_target_y][grid_target_x]))
                # odata[grid_target_y][grid_target_x] = 0
                self.path = self.astar()
                '''
                while self.path_is_blocked():
                    self.stopbot()
                    self.exclude.add(self.target)
                    self.target = self.bfs()
                    self.path = self.astar()
                    rclpy.spin_once(self)
                '''
                self.i = 0
                self.has_target = True
            else:
                # self.path = self.astar()
                v, w = self.pure_pursuit()
                twist = Twist()
                twist.linear.x = v
                twist.angular.z = w
                self.publisher_.publish(twist)
                self.tries += 1
                # if self.i == (len(self.path) - 1):
                # print('target data: ' + str(self.occ_count[self.target[1]][self.target[0]])) 
                if self.occ_count[self.target[1]][self.target[0]] != 1:
                    self.has_target = False

    def path_is_blocked(self):
        directions = [(0, 1), (0, -1), (1, 0), (-1, 0)]
        for p in self.path:
            for dx, dy in directions:
                check_x, check_y = p[0] + dx, p[1] + dy
                if 0 <= check_x < len(self.occ_count) and 0 <= check_x < len(self.occ_count[0]):
                    if self.occ_count[check_y][check_x] == 3:
                        self.path = self.astar()
                        print('path blocked at ' + str(check_x) + ' ' + str(check_y))
                        return True
        return False

            

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin_once(minimal_subscriber)
    minimal_subscriber.mover()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
