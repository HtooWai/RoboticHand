#! /usr/bin/env python
import rospy, math
import numpy as np
import sys, termios, tty, select, os
from geometry_msgs.msg import Twist
from beginner_tutorials.msg import Finger
 
class KeyTeleop(object):
  fng_bindings = {'q': 0, 'w': 90, 'e':180}
  cmd_bindings = {'q':np.array([1,1]),
                  'w':np.array([1,0]),
                  'e':np.array([1,-1]),
                  'a':np.array([0,1]),
                  'd':np.array([0,-1]),
                  'z':np.array([-1,-1]),
                  'x':np.array([-1,0]),
                  'c':np.array([-1,1])
                  }
  set_bindings = { 't':np.array([1,1]),
                  'b':np.array([-1,-1]),
                  'y':np.array([1,0]),
                  'n':np.array([-1,0]),
                  'u':np.array([0,1]),
                  'm':np.array([0,-1])
                }
  def init(self):
    # Save terminal settings
    self.settings = termios.tcgetattr(sys.stdin)
    # Initial values
    self.inc_ratio = 0.1
    self.speed = 0 #np.array([0.5, 1.0])
    self.command = 0 #np.array([0, 0])
    self.update_rate = 10   # Hz
    self.alive = True
    # Setup publisher
    self.pub_pose = rospy.Publisher('/finger_pose', Finger)
 
  def fini(self):
    # Restore terminal settings
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
 
  def run(self):
    try:
      self.init()
      self.print_usage()
      r = rospy.Rate(self.update_rate) # Hz
      while not rospy.is_shutdown():
        ch = self.get_key()
        self.process_key(ch)
        self.update()
        r.sleep()
    except rospy.exceptions.ROSInterruptException:
      pass
    finally:
      self.fini()
 
  def print_usage(self):
    msg = """
    Keyboard Teleop that Publish to /cmd_vel (geometry_msgs/Twist)
    Copyright (C) 2013
    Released under BSD License
    --------------------------------------------------
    H:       Print this menu
    Moving around:
      Q   W   E
      A   S   D
      Z   X   Z
    T/B :   increase/decrease max speeds 10%
    Y/N :   increase/decrease only linear speed 10%
    U/M :   increase/decrease only angular speed 10%
    anything else : stop
 
    G :   Quit
    --------------------------------------------------
    """
    new_msg = """
    Keyboard Teleop that Publish to /cmd_vel (geometry_msgs/Twist)
    Copyright (C) 2013
    Released under BSD License
    --------------------------------------------------
    H:       Print this menu
    Moving around:
      Q   W   E
      0   90  180

    G : Quit
    """
    self.loginfo(new_msg)
    self.show_status()
 
  # Used to print items to screen, while terminal is in funky mode
  def loginfo(self, str):
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
    print(str)
    tty.setraw(sys.stdin.fileno())
 
  # Used to print teleop status
  def show_status(self):
    new_msg = 'Status:\tlinear %.2f\tangular %.2f' % (self.speed,self.command)
    self.loginfo(new_msg)
 
  # For everything that can't be a binding, use if/elif instead
  def process_key(self, ch):
    if ch == 'h':
      self.print_usage()
    elif ch in self.fng_bindings.keys():
      self.command = self.fng_bindings[ch]
    elif ch in self.set_bindings.keys():
      self.speed = self.speed * (1 + self.set_bindings[ch]*self.inc_ratio)
      self.show_status()     
    elif ch == 'g':
      self.loginfo('Quitting')
      # Stop the robot
      finger = Finger()
      self.pub_pose.publish(finger)
      rospy.signal_shutdown('Shutdown')
    else:
      self.command = 0 # np.array([0, 0])
 
  def update(self):
    if rospy.is_shutdown():
      return
    finger = Finger()
    # cmd  = self.speed*self.command
    # twist.linear.x = cmd[0]
    # twist.angular.z = cmd[1]
    finger.finger_pose = self.command
    self.pub_pose.publish(finger)
 
  # Get input from the terminal
  def get_key(self):
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    return key.lower()
 
if __name__ == '__main__':
  rospy.init_node('keyboard_teleop')
  teleop = KeyTeleop()
  teleop.run()
