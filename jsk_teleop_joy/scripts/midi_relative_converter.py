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
from std_msgs.msg import Int8

class AxesConverter:
  def __init__(self):
    self.input_origins = []
    self.output_origins = []
    self.prev_output_axes = [i for i in self.output_origins]

  def adjustAxesSize(self, axes_size):
    if len(self.input_origins) != axes_size:
      if len(self.input_origins) == 0:
        self.input_origins = [None] * axes_size
        self.output_origins = [0.5] * axes_size
        rospy.loginfo("[midi_relative_converter] adjust axes size.")
      else:
        rospy.logwarn("[midi_relative_converter] different axes size!!")

  def convert(self, abs_axes):
    self.adjustAxesSize(len(abs_axes))
    self.updateInputOrigins(abs_axes)
    rel_axes = self.__convertAxes(abs_axes, self.input_origins, self.output_origins)
    self.__updatePrevOutputAxes(rel_axes)
    return rel_axes

  def resetInputOrigins(self):
    for i in range(len(self.input_origins)):
      self.input_origins[i] = None

  def updateInputOrigins(self, abs_axes):
    for i in range(len(abs_axes)):
      if self.input_origins[i] is None and abs_axes[i] != 0 and abs_axes[i] != 1:
        self.input_origins[i] = abs_axes[i]

  def __updatePrevOutputAxes(self, rel_axes):
    self.prev_output_axes = list(rel_axes)

  def updateOutputOrigins(self):
    self.output_origins = list(self.prev_output_axes)

  def __convertAxes(self, abs_axes, in_origins, out_origins):
    rel_axes = [None] * len(abs_axes)
    for i in range(len(abs_axes)):
      rel_axes[i] = self.__convertAxis(abs_axes[i], in_origins[i], out_origins[i])
    return rel_axes

  def __convertAxis(self, abs_axis, in_origin, out_origin):
    if in_origin is None:
      rel_axis = out_origin
    elif abs_axis < in_origin:
      rel_axis = (((abs_axis - 0.0) /  (in_origin - 0.0)) * (out_origin - 0.0)) + 0.0
    elif abs_axis == in_origin:
      rel_axis = out_origin
    elif abs_axis > in_origin:
      rel_axis = (((abs_axis - in_origin) /  (1.0 - in_origin)) * (1.0 - out_origin)) + out_origin
    return rel_axis

class AxesConverterArray:
  def __init__(self, max_page=1):
    self.ac = [AxesConverter() for i in range(max_page)]
    self.crt_page = 0
    self.prev_input_axes = []

  def convert(self, abs_axes):
    return self.ac[self.crt_page].convert(abs_axes)

  def switch_page(self, page):
    if page >= 0 and page < len(self.ac):
      if page != self.crt_page:
        self.__updateReserveInputOrigins(self.prev_input_axes)
        self.__updateCurrentOutputOrigins()
        self.crt_page = page
      return page
    else:
      return -1

  def __updateReserveInputOrigins(self, abs_axes):
    for i in range(len(self.ac)):
      if i != self.crt_page:
        self.ac[i].adjustAxesSize(len(abs_axes))
        self.ac[i].resetInputOrigins()
        self.ac[i].updateInputOrigins(abs_axes)

  def __updateCurrentOutputOrigins(self):
    self.ac[self.crt_page].updateOutputOrigins()

def joy_callback(data):
  joy_pub = rospy.Publisher("/joy_relative/page_" + str(aca.crt_page), Joy)
  joy = Joy()
  joy.header.stamp = rospy.Time.now()
  joy.axes = aca.convert(data.axes)
  aca.prev_input_axes = list(data.axes)
  joy.buttons = data.buttons
  joy_pub.publish(joy)

def switch_page_cmd_cb(msg):
  aca.switch_page(msg.data)

def main():
  global aca
  rospy.init_node('midi_relative_converter')
  aca = AxesConverterArray(2)
  rospy.Subscriber("/joy", Joy, joy_callback)
  rospy.Subscriber("/midi_relative_converter/command/switch_page", Int8, switch_page_cmd_cb)
  rospy.spin()

if __name__ == '__main__':
  main()
