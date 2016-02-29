#!/usr/bin/env python

## Simple talker demo that listens to std_msgs/Strings published 
## to the 'chatter' topic
## Edited by Htoo Wai to subscribe to input ints and 


import time, sys, signal
import rospy
import serial.tools.list_ports  # print(list(serial.tools.list_ports.comports()))
from PyMata.pymata import PyMata
from beginner_tutorials.msg import * # Finger and Sensor messages

# finger and each servo pin
THUMB_SERVO = 3  
INDEX_SERVO = 5
MIDDLE_SERVO = 6
RING_SERVO = 9
LITTLE_SERVO = 10
WRIST_SERVO = 11
SERVO_PINS = [3, 5, 6, 9, 10, 11]   # for test usage

# finger and each sensor pin
THUMB_SENSOR = 0
INDEX_SENSOR = 1
MIDDLE_SENSOR = 2
RING_SENSOR = 3
LITTLE_SENSOR = 4
WRIST_SENSOR = 5

# Indices for data passed to callback function
PIN_MODE = 0  # This is the PyMata Pin MODE = ANALOG = 2 and DIGITAL = 0x20:
PIN_NUMBER = 1
DATA_VALUE = 2


def signal_handler(sig, frame):
    print('You pressed Ctrl+C!!!!')
    if board is not None:
        board.reset()
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

class FSMHand():

    def __init__(self):
        # Initialize fingers and wrist pos
        self.t_pos = 0
        self.i_pos = 0
        self.m_pos = 0
        self.r_pos = 0
        self.l_pos = 0
        self.w_pos = 90

        # Initialize sensor values
        self.t_sen = 0
        self.i_sen = 0
        self.m_sen = 0
        self.r_sen = 0
        self.l_sen = 0

        # Initialize the arduino
        arduino = PyMata("/dev/ttyACM0", verbose=False)

        # Initialize the servos and sensors on arduino
        for pin in SERVO_PINS:
            arduino.servo_config(pin)
            sensor_pin = 0
            arduino.enable_analog_reporting(sensor_pin)
            sensor_pin += 1

        # Initialize the hand states
        self.curstate = 'open'
        self.states = {}
        self.transitionTable = {}


        def move_callback(self, data):
            servo_pose = data.finger_pose
            if self.curstate == 'open':
                arduino.analog_write(MIDDLE_SERVO, servo_pose)
                self.m_pos = servo_pose
            rospy.loginfo(rospy.get_caller_id() + " I heard %d", servo_pose)


def startFSM():
    hand = FSMHand()
    rospy.init_node('HandController', anonymous=True)
    rospy.Subscriber("finger_pose", Finger, hand.move_callback)
    pub = rospy.Publisher('finger_status', Sensor, queue_size=10)
    self.m_sen = board.analog_read(SENSOR)
    pub.publish(self.m_sen)
<<<<<<< Updated upstream
    if self.m_sen > 500 or self.m_pos == 180:
=======
    if self.m_sen > 500 or self.m_pos = 180:
>>>>>>> Stashed changes
        self.curstate = 'close'
    else:
        self.curstate = 'open'
    rate = rospy.Rate(50)
    rospy.spin()

if __name__ == '__main__':
    startFSM()



# def cb_sensor(data):
#     print("Analog Data: ",
#           " Pin: ", data[PIN_NUMBER],
#           " Pin Mode: ", data[PIN_MODE],
#           " Data Value: ", data[DATA_VALUE])
#     # data = data[2]
#     pub.publish(data)


