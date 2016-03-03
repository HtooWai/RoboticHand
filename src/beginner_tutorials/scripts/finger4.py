#!/usr/bin/env python

## Simple talker demo that listens to std_msgs/Strings published 
## to the 'chatter' topic
## Edited by Htoo Wai to subscribe to input ints and 


import time
import sys
import signal
import rospy
from PyMata.pymata import PyMata
from beginner_tutorials.msg import Finger
from beginner_tutorials.msg import Pressure # Finger and Sensor messages

SERVO_PINS = [3, 5, 6, 9, 10, 11]   # for test usage
SENSOR = 0

# Indices for data passed to callback function
PIN_MODE = 0  # This is the PyMata Pin MODE = ANALOG = 2 and DIGITAL = 0x20:
PIN_NUMBER = 1
DATA_VALUE = 2

def cb_sensor(data):
    print("Analog Data: ",
          " Pin: ", data[PIN_NUMBER],
          " Pin Mode: ", data[PIN_MODE],
          " Data Value: ", data[DATA_VALUE])
    # data = data[2]
    # pub.publish(data)

arduino = PyMata("/dev/ttyACM0", verbose=True)
for pin in SERVO_PINS:
    arduino.servo_config(pin)
arduino.set_pin_mode(SENSOR, arduino.INPUT, arduino.ANALOG, cb_sensor)
# arduino = serial.Serial('/dev/ttyACM0', 9600);


def callback(data):
    servo_pose = data.finger_pose
    for pin in SERVO_PINS:
        arduino.analog_write(pin, servo_pose)
    rospy.loginfo(rospy.get_caller_id() + " I heard %d", servo_pose)

def listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'talker' node so that multiple talkers can
    # run simultaneously.
    rospy.init_node('HandController', anonymous=True)
    rospy.Subscriber("finger_pose", Finger, callback)
    pub = rospy.Publisher('finger_status', Pressure, queue_size=10)
    rate = rospy.Rate(50)
    
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
