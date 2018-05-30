#!/usr/bin/env python

import rospy
from robotiq_force_torque_sensor.msg import ft_sensor

def callback(data):
    #rospy.loginfo('Fx=[%f]', data.Fx, 'Fy=[%f]', data.Fy, 'Fz=[%f]', data.Fz)
    print "Fx=", data.Fx, "Fy=", data.Fy, "Fz=", data.Fz

def listener():

    rospy.init_node('rq_sensor_read', anonymous=True)

    rospy.Subscriber("robotiq_force_torque_sensor", ft_sensor, callback, queue_size=1)

    rospy.spin()

if __name__ == '__main__':
    listener()
