#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Mar 28 04:33:40 2024

@author: john
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import OccupancyGrid, Odometry
import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
import numpy as np
import matplotlib.pyplot as plt
from PIL import Image
import math
import scipy.stats
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import cmath
import time
import heapq


# constants
occ_bins = [-1, 0, 50, 100]
map_bg_color = 1
threshold = 8
proximity_limit = 0.2
rotatechange = 0.30
stop_distance = 0.23
front_angle = 35
precission = 0.3
angleChange = 10
testing = False
    
#waitTime = 60

class Cell:
    def __init__(self,parent,pos):
        self.parent = parent
        self.pos = pos
        self.f = 0
        self.g = 0
        self.h= 0
        
def distance(a,b):
    return ((a.pos[0]-b.pos[0])**2 + (a.pos[1]-b.pos[1])**2)**0.5
    
def astar(maze,start,stop,orstep,callback):
    print('B* Running')
    startTime = time.time()
    step = orstep
    startCell = Cell(None,start)
    endCell = Cell(None,stop)
    goaldis = 1.5
    if distance(startCell,endCell) < goaldis* step:
        return[stop]
    def wallinGrid(pos):
        size = round(step/2)
        count = 0
        for i in range(-size,size):
            for j in range(-size,size):
                if count >= 0.1 * size:
                    return True
                if pos[0]+i > len(maze)-1 or pos[1] + j > len(maze[0]) -1:
                    count += 1
                    continue
                if maze[pos[0]+i,pos[1]+j] == 3:
                    count += 1
        if maze[pos[0],pos[1]] in (1,3):
             return True
        return False
        
    def addToOpen(cell):
        open_list[cell.f] = cell
        openPos.append(cell.pos)
        
    open_list = {}
    openPos = []
    closed_list = []
    reset = True
    addToOpen(startCell)

    while open_list:
        rclpy.spin_once(callback)
        if time.time() - startTime > 15 and reset:
            print('wtf')        
            reset = False
            open_list = {}
            addToOpen(startCell)
            # step = round(0.2 * orstep)
            print('new step', step)
            print(start,stop,step)
            goaldis *= 2
        if time.time() - startTime > 30:
            return [startCell.pos]
        currentNode = open_list.pop(min(open_list.keys()))
        openPos.pop(openPos.index(currentNode.pos))
        closed_list.append(currentNode)

        if distance(currentNode,endCell) <= goaldis * step:
            path = []
            current = currentNode
            while current is not None:
                path.append(current.pos)
                current = current.parent
            return path[::-1]
    
        x,y = currentNode.pos[0],currentNode.pos[1]
        for i in range(-1,2):
            for j in range(-1,2):
                a = i * step
                b = j * step
                if (i == 0 and j == 0) or (not (0 <= x + a < len(maze))) or (not (0 <= y + b < len(maze[0]) -1)):
                    continue
                if wallinGrid((x+a,y+b)): # or any([maze[z[0],z[1]] == 3 for z in Adj((x+i,y+j),maze)])
                    continue
                newNode = Cell(currentNode, (x+a,y+b))
                if newNode in closed_list:
                    continue
                newNode.g = currentNode.g + distance(currentNode,newNode)
                newNode.h = distance(newNode,endCell)
                newNode.f = newNode.g + newNode.h

                
                if newNode.pos in openPos:
                    for item in open_list.items():
                        if item == newNode.pos and newNode.g > item.g:
                            continue             
                addToOpen(newNode)
            

def Adj(p,tmap):
    ans = []
    for i in range(-1,2):
        for j in range(-1,2):
            if (i==0 and j ==0) or p[0]+i < 0 or p[1] + j < 0 or p[1] + j > len(tmap[0]) -1 or p[0]+i > len(tmap) -1:
                continue
            ans.append((p[0]+i,p[1]+j))
    return ans

def ableToTravel(map_odata, curr_x_grid, curr_y_grid, target_x_grid, target_y_grid, resolution):
    # return False 
    margin_grid = (int)(0.1/resolution)
    #print(margin_grid)
    if (target_y_grid == curr_y_grid):
        for x in range (curr_x_grid, target_x_grid, 1):
            y = target_y_grid
            if (np.any(map_odata[math.ceil(y)-margin_grid:math.ceil(y)+margin_grid+1, x] in (1,3)) or np.any(map_odata[math.floor(y)-margin_grid:math.floor(y)+margin_grid, x] in (1,3))):
                return False    # meet a wall
    elif (target_x_grid == curr_x_grid):
        for y in range (curr_y_grid, target_y_grid, 1):
            x = target_x_grid
            if (np.any(map_odata[math.ceil(y)-margin_grid:math.ceil(y)+margin_grid+1, x] in (1,3)) or np.any(map_odata[math.floor(y)-margin_grid:math.floor(y)+margin_grid, x] in (1,3))):
                return False    # meet a wall
    else:
        gradient = (float)(target_y_grid - curr_y_grid)/(float)(target_x_grid - curr_x_grid)
        x_init = curr_x_grid #for iteration
        y_init = curr_y_grid
        x_dest = target_x_grid
        if curr_x_grid > target_x_grid:
            x_init = target_x_grid
            y_init = target_y_grid
            x_dest = curr_x_grid
        for x in range (x_init + 1, x_dest, 1):
            y = gradient * (x - x_init) + y_init
            if (np.any(map_odata[math.ceil(y), x] == 3) or np.any(map_odata[math.floor(y), x] == 3)):
                return False    # meet a wall
    return True

        

def median(arr):
    ind = round(len(arr)/2)
    return arr[ind]

def sortpos(arr,x,y):
    distancedic = {}
    for i in arr:
        distancedic[(i[0] - x)**2 + (i[1]-y)**2] = i
    keys = sorted(list(distancedic.keys()))
    ans =[]
    for i in keys:
        ans.append(distancedic[i])
    return ans

def findfronteirs(tmap,posi):
    frontiers = []
    markmap ={}

    #Function to check if a position is a fronteir or not
    def isFronteir(pos):
        if tmap[pos[0],pos[1]]   == 2:
            if pos[0] ==0:
                temp = tmap[:2,:]
            elif pos[0] == len(tmap)-1:
                temp = tmap[pos[0]-1:,:]
            else:
                temp = tmap[pos[0]-1:pos[0]+2,:]

            if pos[1] == 0:
                a = temp[:,:2]
            elif pos[1] == len(tmap[0])-1:
                a = temp[:,pos[1]-1:]
            else:
                a = temp[:,pos[1]-1:pos[1]+2]
            return np.any(a==1)
        return False
    
    def mark(p):
        return markmap.get(p,'Unmarked')
    
    def adj(p):
        return Adj(p,tmap)

    #Defining basic parameters
    qm = []
    qm.append(posi)
    markmap[posi] = "Map-Open-List"

    while qm:
        p = qm.pop(0)

        if mark(p) == 'Map-Close-List':
            print('I am running')
            continue
        
        if isFronteir(p):
            qf = []
            nf = []
            qf.append(p)
            markmap[p] = "Frontier-Open-List"

            while qf:
                q = qf.pop(0)
                if mark(q) in ["Map-Close-List","Frontier-Close-List"]:
                    continue
                if isFronteir(q):
                    nf.append(q)
                    for w in adj(q):
                        if mark(w) not in ["Frontier-Open-List",'Frontier-Close-List','Map-Close-List']:
                            qf.append(w)
                            markmap[w] = "Frontier-Open-List"
                markmap[q] = 'Frontier-Close-List'
            if len(nf) > threshold:
                frontiers.append(nf)
            for i in nf:
                markmap[i] = "Map-Close-List"
                
        for v in adj(p):
            if mark(v) not in ["Map-Open-List","Map-Close-List"]:
                if any([tmap[x[0],x[1]] == 2 for x in adj(v)]):
                       qm.append(v)
                       markmap[v] = "Map-Open-List"
        markmap[p] = "Map-Close-List"
    return frontiers
    


# code from https://automaticaddison.com/how-to-convert-a-quaternion-into-euler-angles-in-python/
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


class Occupy(Node):
    def __init__(self):
        super().__init__('occupy')
        
        # create subscription to track occupancy
        self.subscription = self.create_subscription(
            OccupancyGrid,
            'map',
            self.listener_callback,
            qos_profile_sensor_data)
        #create subscription for odometry
        # self.odom_subscription = self.create_subscription(
        #     Odometry,
        #     'odom',
        #     self.odom_callback,
        #     10)
        
        # create subscription to track lidar
        self.scan_subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            qos_profile_sensor_data)
        self.scan_subscription  # prevent unused variable warning
        self.laser_range = np.array([])
        
        # create publisher for moving TurtleBot
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscription  # prevent unused variable warning
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer, self)
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self.x = 0
        self.y = 0
        self.target = []
        self.isCrashing = False
        self.crashAngle = 0
        self.isSpinning = False
        self.path = []
        self.currentFrontier = []
        self.shouldScan = False
        self.doneMapping = True
        self.mapgrid= []
        # self.angularspeed = 0
        #self.startTime = time.time()
        #self.pastTargets = []
        
    # def odom_callback(self, msg):
    #     # self.get_logger().info('In odom_callback')
    #     # position = msg.pose.pose.position
    #     # orientation_quat =  msg.pose.pose.orientation
    #     self.angularspeed = msg.twist.twist.angular.z
    #     # self.x,self.y,self.z = position.x,position.y,position.z
    #     # self.roll, self.pitch, self.yaw = euler_from_quaternion(orientation_quat.x, orientation_quat.y, orientation_quat.z, orientation_quat.w)
        
    def scan_callback(self, msg):
        # self.get_logger().info('In scan_callback')
        # create numpy array
        self.laser_range = np.array(msg.ranges)
        # replace 0's with nan
        self.laser_range[self.laser_range==0] = np.nan
        
        for i in range(front_angle):
            if (self.laser_range[i] != np.nan and self.laser_range[i] < float(stop_distance)) and (not self.isSpinning) and self.target:
                self.crashAngle = i
                self.isCrashing = True
                print("CRASHHHH")
                break
            elif (self.laser_range[-1*i] != np.nan and self.laser_range[-1*i] < float(stop_distance)) and (not self.isSpinning) and self.target:
                self.crashAngle = 360 - i
                self.isCrashing = True
                print("CRASHHHH")
                break
            else:
                self.isCrashing = False
        
        
    def stopbot(self):
        self.get_logger().info('In stopbot')
        # publish to cmd_vel to move TurtleBot
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        # time.sleep(1)
        self.publisher.publish(twist)
        
    def rotatebot(self, rot_angle):
        self.get_logger().info('In rotatebot')
        if abs(rot_angle) < 5:
            rotatechange = 0.05
        elif abs(rot_angle) < 20:
            rotatechange = 0.1
        else:
            rotatechange = 0.2
        
        # create Twist object
        twist = Twist()
        
        # get current yaw angle
        current_yaw = self.yaw
        # log the info
        # we are going to use complex numbers to avoid problems when the angles go from
        # 360 to 0, or from -180 to 180
        c_yaw = complex(math.cos(current_yaw),math.sin(current_yaw))
        # calculate desired yaw
        target_yaw = current_yaw + math.radians(rot_angle)
        # convert to complex notation
        c_target_yaw = complex(math.cos(target_yaw),math.sin(target_yaw))
        # divide the two complex numbers to get the change in direction
        c_change = c_target_yaw / c_yaw
        # get the sign of the imaginary component to figure out which way we have to turn
        c_change_dir = np.sign(c_change.imag)
        # set linear speed to zero so the TurtleBot rotates on the spot
        twist.linear.x = 0.0
        # set the direction to rotate
        twist.angular.z = c_change_dir * rotatechange 
        # start rotation
        self.publisher.publish(twist)
        self.isSpinning = True

        # we will use the c_dir_diff variable to see if we can stop rotating
        c_dir_diff = c_change_dir
        #self.get_logger().info('c_change_dir: %f c_dir_diff: %f' % (c_change_dir, c_dir_diff))
        # if the rotation direction was 1.0, then we will want to stop when the c_dir_diff
        # becomes -1.0, and vice versa
        while(c_change_dir * c_dir_diff > 0):
            # allow the callback functions to run
            # if self.angularspeed == 0:
            #     break
            rclpy.spin_once(self)
            current_yaw = self.yaw
            # convert the current yaw to complex form
            c_yaw = complex(math.cos(current_yaw),math.sin(current_yaw))
            self.get_logger().info('Current Yaw: %f' % math.degrees(current_yaw))
            self.get_logger().info('Target Yaw: %f' %math.degrees(target_yaw))
            # get difference in angle between current and target
            c_change = c_target_yaw / c_yaw
            # get the sign to see if we can stop
            c_dir_diff = np.sign(c_change.imag)
            #self.get_logger().info('c_change_dir: %f c_dir_diff: %f' % (c_change_dir, c_dir_diff))

        # set the rotation speed to 0
        twist.angular.z = 0.0
        # stop the rotation
        self.publisher.publish(twist)
        self.isSpinning = False

    def listener_callback(self, msg):
        # print('callback')
        # create numpy array
        occdata = np.array(msg.data)
        # compute histogram to identify bins with -1, values between 0 and below 50, 
        # and values between 50 and 100. The binned_statistic function will also
        # return the bin numbers so we can use that easily to create the image 
        occ_counts, edges, binnum = scipy.stats.binned_statistic(occdata, np.nan, statistic='count', bins=occ_bins)
        # get width and height of map
        iwidth = msg.info.width
        iheight = msg.info.height
        # calculate total number of bins
        total_bins = iwidth * iheight
        # log the info
        #self.get_logger().info('Unmapped: %i Unoccupied: %i Occupied: %i Total: %i' % (occ_counts[0], occ_counts[1], occ_counts[2], total_bins))

        # find transform to obtain base_link coordinates in the map frame
        # lookup_transform(target_frame, source_frame, time)
        try:
            trans = self.tfBuffer.lookup_transform('map', 'base_link', rclpy.time.Time(),timeout=rclpy.duration.Duration(seconds=0.01))
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            print(e)
            self.stopbot()
            return
            
        cur_pos = trans.transform.translation
        cur_rot = trans.transform.rotation
        self.x,self.y = cur_pos.x,cur_pos.y
        # convert quaternion to Euler angles
        roll, pitch, yaw = euler_from_quaternion(cur_rot.x, cur_rot.y, cur_rot.z, cur_rot.w)
        self.roll,self.pitch,self.yaw = roll,pitch,yaw
        #self.get_logger().info('Rot-Yaw: R: %f D: %f' % (yaw, np.degrees(yaw)))

        # get map resolution
        map_res = msg.info.resolution
        # get map origin struct has fields of x, y, and z
        map_origin = msg.info.origin.position
        # get map grid positions for x, y position
        grid_x = round((cur_pos.x - map_origin.x) / map_res)
        grid_y = round(((cur_pos.y - map_origin.y) / map_res))
        # self.get_logger().info('Grid Y: %i Grid X: %i' % (grid_y, grid_x))

        # binnum go from 1 to 3 so we can use uint8
        # convert into 2D array using column order
        odata = np.uint8(binnum.reshape(msg.info.height,msg.info.width))
        # set current robot location to 0
        odata[grid_y][grid_x] = 0
        target = self.target
        solgrid = []
        if self.shouldScan and (not target or abs(target[1]-cur_pos.x) + abs(target[0]-cur_pos.y) < proximity_limit) and (not self.isSpinning):
            # if should scan AND (either no target or at target) and is not spinning 
            if self.path:
                if not any([odata[round((self.currentFrontier[0]-map_origin.y)/map_res)+x[0],round((self.currentFrontier[1]-map_origin.x)/map_res)+x[1]] == 1 for x in ((1,0),(0,1),(-1,0),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1))]):
                    # if none of the directions have a value of 1 in odata
                    # if no unexplored in all directions of current target cell
                    self.path = []
                else:
                    solgrid = [(round((x[0]-map_origin.y)/map_res),round((x[1]-map_origin.x)/map_res)) for x in self.path]
                    # solgrid = path in grid coordinates
                    
                    for x in range(1,len(solgrid)):
                        print(x)
                        print('8')
                        try:
                            if ableToTravel(odata,grid_x,grid_y,solgrid[-x][1],solgrid[-x][0],map_res):
                                self.path = self.path[-x:]
                                # aim at furthest reachable point
                        except:
                            continue
                
                    self.target = self.path.pop(0)
            else:
                # if no path 
                self.path = []
                self.stopbot()
                # if target:
                #     self.pastTargets.append(self.target)
                # should only search frontier if there is no current target or have reached the current target
                print("Frontier Positions")
                frontier_positions = findfronteirs(odata,(grid_y,grid_x))
                midpoint_positions = []
                for i in frontier_positions:
                    midpoint_positions.append(median(i))
                print(midpoint_positions)
                midpoint_positions = sortpos(midpoint_positions,grid_y,grid_x)
                if midpoint_positions:
                    self.doneMapping = False
                    target = [midpoint_positions[0][0]*map_res+map_origin.y,midpoint_positions[0][1]*map_res+map_origin.x]
                    self.currentFrontier = target
                    goal_grid = (round((target[0]-map_origin.y)/map_res),round((target[1]-map_origin.x)/map_res))
                    turtlbot_grid = round(precission/map_res)
                    solgrid = (astar(odata,(grid_y,grid_x),goal_grid,turtlbot_grid,self))
                    self.mapgrid = solgrid
                    
                    for x in range(1,len(solgrid)-1):
                        print(x)
                        try:
                            if ableToTravel(odata,grid_x,grid_y,solgrid[-x][1],solgrid[-x][0],map_res):
                                solgrid = solgrid[-x:]
                        except:
                            continue
                
                    print(solgrid)
                    
                    for i in solgrid:
                        self.path.append( (i[0] * map_res + map_origin.y,i[1] * map_res+map_origin.x) )
                    self.target = self.path.pop(0)
                    self.shouldScan = False
                    time.sleep(0.01)
                    print(solgrid)
                    print(self.target)
                    #self.startTime = time.time()
                else:
                    self.doneMapping = True
                    print("no frontier found")
        if testing:
            self.doneMapping = True
        # if self.mapgrid:
        #     for i in self.mapgrid:
        #         odata[i[0],i[1]] = 0
        if self.doneMapping and self.target:
            goal_grid = (round((self.target[0]-map_origin.y)/map_res),round((self.target[1]-map_origin.x)/map_res))
            odata[goal_grid[0],goal_grid[1]] = 0
            img = Image.fromarray(odata)
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
            rotated = img_transformed.rotate(np.degrees(yaw)-90, expand=True, fillcolor=map_bg_color)
            rotated.save('Map.png',cmap = 'gray',origin = 'lower')
    
            # show the image using grayscale map
            # plt.imshow(img, cmap='gray', origin='lower')
            # plt.imshow(img_transformed, cmap='gray', origin='lower')
            plt.imshow(rotated, cmap='gray', origin='lower')
            plt.draw_all()
            # pause to make sure the plot gets created
            plt.pause(0.00000000001)
            
    def movetotarget(self, target):
        if target:
            if (self.isCrashing):
                # Handles crash avoidance
                # print("Avoiding crash")
                self.stopbot()
                closestAngle = np.nanargmin(self.laser_range)
                # for i in range(len(self.laser_range)):
                #     print(i, self.laser_range[i])
                # print('closest Angle = {}'.format(closestAngle))
                leftFound = False
                rightFound = False
                for i in range(0,180):
                    anglePos = (closestAngle +i)%360
                    try:
                        if (not rightFound) and self.laser_range[anglePos] > 0.3:
                            posDisplace = i
                            rightFound = True
                        angleNeg = (closestAngle - i)%360
                        if (not leftFound) and self.laser_range[angleNeg]>0.3:
                            negDisplace = i
                            leftFound = True
                        if leftFound and rightFound:
                            break
                    except:
                        continue
                if not leftFound:
                    negDisplace = 180
                if not rightFound:
                    posDisplace = 180
                # print(posDisplace, negDisplace)
                posAngle = (closestAngle + posDisplace)%360
                negAngle = (closestAngle - negDisplace)%360
                if posDisplace + negDisplace > 30:
                    offset = 30
                else:
                    offset = 40
                if posAngle > 180:
                    posAngle = posAngle - 360
                if negAngle > 180:
                    negAngle = negAngle - 360
                
                if posAngle + negAngle > 0:
                    destinationAngle = posAngle + offset
                else:
                    destinationAngle = negAngle -offset
                # print("Pos Angle = {}, negAngle = {}".format(posAngle,negAngle))
                # print("destination angle: ",destinationAngle)
                while self.isCrashing:
                    rclpy.spin_once(self)
                    self.stop_distance = 0.4
                    twast = Twist()
                    twast.linear.x = 0.0
                    angular = 0.2
                    if posAngle + negAngle > 0:
                        angular = -angular
                    twast.angular.z = angular
                    self.publisher.publish(twast)
                self.stopbot()
                self.stop_distance = 0.25
                # self.rotatebot(destinationAngle)
                rclpy.spin_once(self)
                # self.isCrashing = False
                if not self.isCrashing:
                    twist = Twist()
                    twist.linear.x = 0.2
                    twist.angular.z = 0.0
                    self.publisher.publish(twist)
                    print("MOVING\n\n\n\n\n")
                    time.sleep(0.8)
                self.stopbot()          
            else:
                # Handles normal movement to target
                # print("target acquired")
                # print(target)
                angle = np.arctan((target[0]-self.y)/(target[1]-self.x))-self.yaw
                if (target[1]-self.x) < 0:
                    if (target[0]-self.y) > 0:
                        angle += np.pi
                    else:
                        angle -= np.pi
                
                print(np.degrees(angle))                
                if abs(np.degrees(angle)) > 10:
                    self.stopbot()
                    self.rotatebot(np.degrees(angle))                
                # print('Start moving')
                twist = Twist()
                twist.linear.x = 0.2
                twist.angular.z = 0.0
                self.publisher.publish(twist)
        rclpy.spin_once(self) #get the lidar data to prevent crash
            
    def Move(self):
        target = []
        rclpy.spin_once(self)
        print(target)
        while (True):
            # print('starting')
            print(target)
            rclpy.spin_once(self)
            try:
                target = self.target
                if not target or abs(target[1]-self.x) + abs(target[0]-self.y) < proximity_limit:
                    self.shouldScan = True
                        # print("Target reached")
                    self.stopbot()
                else:
                        # print('moving')
                    self.movetotarget(target)
            except Exception as e:
                print(e)
                print('a')


def main(args=None):
    rclpy.init(args=args)

    occupy = Occupy()

    # create matplotlib figure
    occupy.Move()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    occupy.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
