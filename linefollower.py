import rclpy
from rclpy.node import Node
import geometry_msgs.msg
import RPi.GPIO as gpio
import time
import requests
import json

gpio.setmode(gpio.BCM)

left_in = 13
right_in = 6


servo_pin = 17
Servo=27
# Set the pin as an output
gpio.setup(servo_pin, gpio.OUT)
gpio.setup(Servo, gpio.OUT)
# Initialise the servo to be controlled by pwm with 50 Hz frequency
p = gpio.PWM(servo_pin, 50)
P=gpio.PWM(Servo,50)
# Set servo to 90 degrees as it's starting position
p.start(7.5)
P.start(2.5)

gpio.setup(left_in,gpio.IN)
gpio.setup(right_in,gpio.IN)
# constants
rotatechange = 0.1
speedchange = 0.05


class Mover(Node):
    def init(self):
        super().init('mover')
        self.publisher_ = self.create_publisher(geometry_msgs.msg.Twist,'cmd_vel',10)

# function to read keyboard input
    def readKey(self):
        print('called')
        twist = geometry_msgs.msg.Twist()
        try:
            a=0
            while True:
                time.sleep(0.05)
                if gpio.input(right_out) == gpio.LOW and gpio.input(left_in) == gpio.LOW:
                    print('stop')
                    twist.angular.z = 0.0
                    twist.linear.x = 0.0
                    self.publisher_.publish(twist)
                    a+=1
                    print('a=',a)
                    if a==1: 
                        url='http://172.20.10.12/openDoor'
                        data={"action": "openDoor", "parameters": {"robotId": 37}}

                        json_data=json.dumps(data)

                        headers={'Content-Type': 'application/json'}

                        response=requests.post(url, data=json_data, headers=headers)
                        print(response.json()['data']['message'])
                        print(response.json()['data']['message'][4])
                        time.sleep(1)
                        print("continue")

                        twist.angular.z=0.3
                        self.publisher_.publish(twist)
                        time.sleep(5)
                    elif a==2:
                        print("I am throwing")
                        p.ChangeDutyCycle(2.5)
                        P.ChangeDutyCycle(7.5)
                        time.sleep(2)
                        p.ChangeDutyCycle(7.5)
                        P.ChangeDutyCycle(2.5)
                        time.sleep(2)
                        p.stop()
                        P.stop()
                        gpio.cleanup()
                        break

                # to turn move forward
                elif gpio.input(left_in) == gpio.HIGH and gpio.input(right_in) == gpio.HIGH:
                    print('forward')
                    twist.angular.z = 0.0
                    twist.linear.x = 0.1
                # to turn right
                elif gpio.input(right_in) == gpio.HIGH and gpio.input(left_in) == gpio.LOW:
                    twist.angular.z = -0.1
                    twist.linear.x = 0.0
                    print("turn right")
                # to turn left
                elif gpio.input(left_in) == gpio.HIGH and gpio.input(right_in) == gpio.LOW:
                    twist.angular.z = 0.1
                    twist.linear.x = 0.0
                    print("turn left")
                    '''
                elif gpio.input(left_out)==gpio.HIGH:
                    twist.angular.z = 0.2
                    twist.linear.x=0.1
                elif gpio.input(right_out)==gpio.HIGH:
                    twist.angular.z = -0.2
                    twist.linear.x=0.1
                    '''
                      
                # start the movement
                self.publisher_.publish(twist)
                
                
        except Exception as e:
            print(e)
            p.stop()
            gpio.cleanup()

  # Ctrl-c detected
        finally:
         # stop m
