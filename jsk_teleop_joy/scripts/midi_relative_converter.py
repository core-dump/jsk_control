#!/usr/bin/env python
import pygame
import pygame.midi
import select
import sys
import yaml
import rospy
import roslib

try:
  from sensor_msgs.msg import Joy, JoyFeedbackArray
except:
  roslib.load_manifest('jsk_teleop_joy')
  from sensor_msgs.msg import Joy, JoyFeedbackArray

def joy_callback(data):
  joy_pub = rospy.Publisher("/joy_relative", Joy)
  joy = Joy()
  joy.header.stamp = rospy.Time.now()
  joy.axes = data.axes
  joy.buttons = joy.buttons
  joy_pub.publish(joy)

def main():
  rospy.init_node('midi_joy')
  rospy.Subscriber("/joy", Joy, joy_callback)
  rospy.spin()

if __name__ == '__main__':
  main()
