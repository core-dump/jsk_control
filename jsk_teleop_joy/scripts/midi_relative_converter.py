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

class AxesConverter:
  def __init__(self, axes):
    self.input_origin = list(axes)
    self.output_origin = [0.5] * len(axes)

  def convert(self, abs_axes):
    rel_axes = [0] * len(abs_axes)
    for i in range(len(abs_axes)):
      rel_axes[i] = abs_axes[i]
    return rel_axes

def joy_callback(data):
  joy_pub = rospy.Publisher("/joy_relative", Joy)
  joy = Joy()
  try:
    ac
  except NameError:
    ac = AxesConverter(data.axes)
  joy.header.stamp = rospy.Time.now()
  joy.axes = ac.convert(data.axes)
  joy.buttons = data.buttons
  joy_pub.publish(joy)

def main():
  global ac
  rospy.init_node('midi_joy')
  rospy.Subscriber("/joy", Joy, joy_callback)
  rospy.spin()

if __name__ == '__main__':
  main()
