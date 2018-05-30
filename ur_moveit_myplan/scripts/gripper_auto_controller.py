#!/usr/bin/env python

import rospy
from std_msgs.msg import Char, String
from robotiq_force_torque_sensor.msg import ft_sensor
last_Fz = 0
is_activate = 0
##def callback(data):
##    global is_activate
##    global last_Fz
##    pub = rospy.Publisher('/robot_gripper_auto_control', String, queue_size=10)
##    if is_activate == 0:
##      rospy.sleep(2)
##      pub.publish('r')
##      rospy.sleep(2)    
##      pub.publish('a')
##      last_Fz = data.Fz
##      is_activate = 1
##    if abs(data.Fz-last_Fz) >= 8:
###    pub.publish('o')
###    rospy.sleep(4)
##      pub.publish('c')
##      rospy.sleep(5)
##      pub.publish('o')
###    while not rospy.is_shutdown():
###        gripper_char = '100'
###        pub.publish(gripper_char)
###        rospy.sleep(3)

## control for 3-finger gripper
def callback(data):
    global is_activate
    global last_Fz
    pub = rospy.Publisher('/robot_gripper_auto_control', String, queue_size=10)
    if is_activate == 0:
      rospy.sleep(2)
      pub.publish('r')
      rospy.sleep(2)    
      pub.publish('a')
      last_Fz = data.Fz
      is_activate = 1
    if abs(data.Fz-last_Fz) >= 8:
#    pub.publish('o')
#    rospy.sleep(4)
      pub.publish('c')
      rospy.sleep(5)
      pub.publish('o')
#    while not rospy.is_shutdown():
#        gripper_char = '100'
#        pub.publish(gripper_char)
#        rospy.sleep(3)
def gripper_pub():
  rospy.init_node('gripper_auto_controller', anonymous=True)
  rq_sensor_sub = rospy.Subscriber("robotiq_force_torque_sensor", ft_sensor, callback, queue_size=1)
  rospy.spin()
if __name__ == '__main__':
    try:
        gripper_pub()
    except rospy.ROSInterruptException:
        pass
