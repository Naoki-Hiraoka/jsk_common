#!/usr/bin/env python
# from __future__ import print_function

import sys
import time
import rospy
import rospkg
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class FilterUnreliableScan:

  def __init__(self):
    self.rotate_th    = float(rospy.get_param('~rotate_th', 0.5236)) # 30 deg/s
    self.fast_rotate  = False
    self.use_flywheel = False

    self.sub_use_flywheel = rospy.Subscriber("/use_flywheel", Int32, self.UseFlywheelCallback)
    self.sub_cmd_vel      = rospy.Subscriber("/cmd_vel", Twist, self.CmdVelCallback)
    # self.sub_lidar_scan   = rospy.Subscriber('~input_scan', LaserScan, self.ScanCallback)
    # self.pub_reliable_lidar_scan = rospy.Publisher('~output_scan', LaserScan, queue_size=1)
    self.sub_lidar_scan   = rospy.Subscriber("/multisense/lidar_scan", LaserScan, self.ScanCallback)
    self.pub_reliable_lidar_scan = rospy.Publisher("/multisense/reliable_lidar_scan", LaserScan, queue_size=1)

  def UseFlywheelCallback(self, data):
    if data == '1':
      self.use_flywheel = True
    else:
      self.use_flywheel = False

  def CmdVelCallback(self, data):
    if abs(data.angular.z) > self.rotate_th:
      self.fast_rotate = True
    else:
      self.fast_rotate = False

  def ScanCallback(self, data):
    if self.use_flywheel or self.fast_rotate:
      print ("Scan is unreliable!")
      time.sleep(2.0)
    else:
      self.pub_reliable_lidar_scan.publish(data)

def main(args):
  rospy.init_node('filter_unreliable_scan', anonymous=True)
  operator = FilterUnreliableScan()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)
