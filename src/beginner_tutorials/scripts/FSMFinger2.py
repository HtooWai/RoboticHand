#!/usr/bin/env python

## Simple FSM for the hand with 'open' and 'closed' states

import time, sys, signal
import rospy
import serial.tools.list_ports  # print(list(serial.tools.list_ports.comports()))
from PyMata.pymata import PyMata
from std_msgs.msg import String
from beginner_tutorials.msg import * # Finger and Sensor messages

# finger arrangement
THUMB = 0
INDEX = 1
MIDDLE = 2
RING = 3
LITTLE = 4
WRIST = 5

# finger and each servo pin
THUMB_SERVO = 3  
INDEX_SERVO = 5
MIDDLE_SERVO = 6
RING_SERVO = 9
LITTLE_SERVO = 10
WRIST_SERVO = 11
SERVO_PINS = [3, 5, 6, 9, 10, 11]   # for test usage
FINGER_PINS = [3, 5, 6, 9, 10]      # for ease of reading

# determined during calibration
LIMITS = [(35,124),(35,150),(35,150),(35,144),(35,150),(0,180)]

# finger and each sensor pin, just for ease of reading
THUMB_SENSOR = 0
INDEX_SENSOR = 1
MIDDLE_SENSOR = 2
RING_SENSOR = 3
LITTLE_SENSOR = 4
WRIST_SENSOR = 5
SENSOR_PINS = [0, 1, 2, 3, 4]

# Indices for data passed to callback function
PIN_MODE = 0  # This is the PyMata Pin MODE = ANALOG = 2 and DIGITAL = 0x20:
PIN_NUMBER = 1
DATA_VALUE = 2


class FSMHand():

    def __init__(self):
        # Initialize fingers and wrist pos
        self.finger_pos = [0, 0, 0, 0, 0, 90]

        # Initialize sensor values
        self.sensor_val = [0, 0, 0, 0, 0]

        # Initialize the arduino
        self.arduino = PyMata("/dev/ttyACM0", verbose=True)

        # Initialize the servos and sensors on arduino
        for servo_pin in SERVO_PINS:
            self.arduino.servo_config(servo_pin)
        for sensor_pin in SENSOR_PINS:
            self.arduino.enable_analog_reporting(sensor_pin)

        # Initialize the hand states
        self.curstate = 'open'
        self.states = {}
        self.transitionTable = {}


    #states are a dictionary of name/function pairints stored in a dictionary
    #i.e. {'open':self.Open}
    def AddFSMState(self,name,function):
            self.states[name] = function

    #each state gets its own transition table
    #a transition table is a list of states to switch to
    #given a "event"
    def AddFSMTransition(self,name,transitionDict):
            #yes we are making a dictionary the value bound to a dictionary key
            self.transitionTable[name] = transitionDict

    def Move_callback(self, data):
        servo_pose = data.finger_pose
        if self.curstate == 'open':
            for i, pin in enumerate(FINGER_PINS):
                self.arduino.analog_write(pin, servo_pose)
                self.finger_pos[i] = servo_pose
        rospy.loginfo(rospy.get_caller_id() + " I heard %d", servo_pose)


    def Event_handler(self):
        curstatefunction = self.states[self.curestate]
        curstatefunction()

    def RunFSM(self):
        pub = rospy.Publisher('finger_status', Pressure, queue_size=10)
        rate = rospy.Rate(50)

        while not rospy.is_shutdown():
            for i, sensor_pin in enumerate(SENSOR_PINS):
                self.sensor_val[i] = self.arduino.analog_read(sensor_pin)
            outdata = Pressure()
            outdata.sensor1 = self.sensor_val[0]
            pub.publish(outdata)

            if max(self.sensor_val) > 500 or max(self.finger_pos) == 150:
                self.curstate = 'close'
            else:
                self.curstate = 'open'
            print "Current State: ", self.curstate
            rate.sleep()

def startFSM():
    hand = FSMHand()

    rospy.init_node('HandController', anonymous=True)
    rospy.Subscriber("finger_pose", Finger, hand.Move_callback)
    rospy.Subscriber("hand_event", String, hand.Event_handler)
    hand.RunFSM()
    
if __name__ == '__main__':
    startFSM()



# Can be used if pub is declared in the callback
# def cb_sensor(data):
#     print("Analog Data: ",
#           " Pin: ", data[PIN_NUMBER],
#           " Pin Mode: ", data[PIN_MODE],
#           " Data Value: ", data[DATA_VALUE])
#     # data = data[2]
#     pub.publish(data)


