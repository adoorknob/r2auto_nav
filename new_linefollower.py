import rclpy
from rclpy.node import Node
import geometry_msgs.msg
import RPi.GPIO as gpio
import time
import requests
import json
import sys
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math
import cmath
import time
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
import numpy as np

gpio.setmode(gpio.BCM)

left_in = 13
right_in = 6

servo_pin = 12
# Set the pin as an output
gpio.setup(servo_pin, gpio.OUT)
# Initialise the servo to be controlled by pwm with 50 Hz frequency
p = gpio.PWM(servo_pin, 50)

gpio.setup(left_in,gpio.IN)
gpio.setup(right_in,gpio.IN)
# constants
rotatechange = 2.0
slow_rotatechange = 1.1
slow_rotate_delay = 1.5
speedchange = 0.20
LEFT = 1
http_url='http://192.168.90.87/openDoor'

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

class Mover(Node):
    def __init__(self):
        super().__init__('mover')
        # publish movement
        self.publisher_ = self.create_publisher(Twist,'cmd_vel',10)
        # publish to pc for astar
        self.publisher_astar = self.create_publisher(Twist,'start_astar',10)
        self.stop_thres = 0
        self.readKey()
        
# function to read keyboard input
    def readKey(self):
        try:
            a=0
            twist = Twist()

            while True:
                if gpio.input(right_in) == gpio.LOW and gpio.input(left_in) == gpio.LOW:

                    print('stop')
                    twist.angular.z = 0.0
                    twist.linear.x = 0.0
                    self.publisher_.publish(twist)
                    a += 1
                    # a is used to check t-junction for http call vs t-junction for bucket
                    print('a=',a)
                    if a==1: 

                        # first t-junction: makes call to door
                        url=http_url
                        data={"action": "openDoor", "parameters": {"robotId": 31}}

                        json_data=json.dumps(data)

                        headers={'Content-Type': 'application/json'}

                        while True:
                            print('calling')
                            response=requests.post(url, data=json_data, headers=headers)
                            self.door = int(response.json()['data']['message'][4])
                            print(f"door num: {self.door}")
                            time.sleep(1)
                            print("continue")
                            print('calling door')
                            time.sleep(1)
                            print('continue')

                            if self.door == LEFT: 
                                print('turning to left door')
                                twist.angular.z = slow_rotatechange
                                self.publisher_.publish(twist)
                                time.sleep(slow_rotate_delay)
                                break
                            else:
                                print('turning to right door')
                                twist.angular.z= -slow_rotatechange
                                self.publisher_.publish(twist)
                                time.sleep(slow_rotate_delay)
                                break

                    elif a==2:

                        p.start(7.5)
                        time.sleep(2)
                        print("I am throwing")
                        p.ChangeDutyCycle(2.5)
                        time.sleep(2)
                        p.ChangeDutyCycle(7.5)
                        time.sleep(2)
                        p.stop()

                        # go backwards
                        twist.linear.x = -0.08
                        twist.angular.z = 0.0
                        self.publisher_.publish(twist)
                        time.sleep(0.8)
                        # stopbot
                        twist.linear.x = 0.0
                        twist.angular.z = 0.0
                        self.publisher_.publish(twist)
                        time.sleep(0.5)
                        # 180 degree turn
                        twist.angular.z = slow_rotatechange
                        self.publisher_.publish(twist)
                        time.sleep(2.2*slow_rotate_delay)
                    elif a == 3:
                        if self.door == LEFT:
                            twist.angular.z = -slow_rotatechange
                        else:
                            twist.angular.z = slow_rotatechange
                        self.publisher_.publish(twist)
                        time.sleep(slow_rotate_delay)
                        t = geometry_msgs.msg.Twist()
                        print('calling astar')
                        self.publisher_astar.publish(t)
                        sys.exit() 

                # to turn move forward
                elif gpio.input(left_in) == gpio.HIGH and gpio.input(right_in) == gpio.HIGH:
                    print('forward')
                    twist.angular.z = 0.0
                    twist.linear.x = speedchange
                # to turn right
                elif gpio.input(right_in) == gpio.LOW and gpio.input(left_in) == gpio.HIGH:
                    twist.angular.z = -rotatechange
                    twist.linear.x = 0.0
                    print("turn right")
                # to turn left
                elif gpio.input(left_in) == gpio.LOW and gpio.input(right_in) == gpio.HIGH:
                    twist.angular.z = rotatechange
                    twist.linear.x = 0.0
                    print("turn left")
                      
                # start the movement
                self.publisher_.publish(twist)
                
                
        except Exception as e:
            print(e)
            gpio.cleanup()
            p.stop()

  # Ctrl-c detected
        finally:
            print('urmom')
            twist.angular.z = 0.0
            twist.linear.x = 0.0
            self.publisher_.publish(twist)
            gpio.cleanup()
         # stop m

def main(args=None):
    rclpy.init(args=args)

    test = Mover()

    rclpy.spin(test)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    scanner.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
