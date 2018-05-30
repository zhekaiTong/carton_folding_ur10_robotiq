#!/usr/bin/env python
import rospy
from ur_moveit_myplan.msg import qr_status
from std_msgs.msg import Int8

def callback(data):
  msg = qr_status()
  msg.header.stamp = rospy.Time.now()
  msg.status = data
  status_pub = rospy.Publisher('custom_qr_status', qr_status, queue_size=20)
  status_pub.publish(msg)

def listen_cam_status():
  rospy.init_node('cam_sta_pub', anonymous=True)
  
  status_sub = rospy.Subscriber('/visp_auto_tracker/status', Int8, callback)
  rospy.spin()

if __name__ == '__main__':
  try:
      listen_cam_status()
  except rospy.ROSInterruptException: pass
