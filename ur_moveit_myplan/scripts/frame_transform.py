#!/usr/bin/env python  
import roslib
roslib.load_manifest('ur_moveit_myplan')
import numpy
import rospy
import tf
from math import pi

if __name__ == '__main__':
    rospy.init_node('base_cam_transform')
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(20.0)

## translation and rotation of cam expressed in base_link
    xaxis = (1, 0, 0)
    yaxis = (0, 1, 0)
    Rx = tf.transformations.rotation_matrix(-pi/2, xaxis)
    Ry = tf.transformations.rotation_matrix(pi, yaxis)
    #Rx[:3,3] = numpy.array([-1.0, -0.02, 0.0])
    euler = tf.transformations.euler_from_matrix(numpy.dot(Rx,Ry), 'sxyz')
    quaternion = tf.transformations.quaternion_from_euler(euler[0], euler[1], euler[2], axes='sxyz')

## translation and rotation of gripper expressed in ee_link
    xaxis_g = (1, 0, 0)
    Rx_g = tf.transformations.rotation_matrix(pi/4, xaxis_g)
    euler_g = tf.transformations.euler_from_matrix(Rx_g, 'sxyz')
    quaternion_g = tf.transformations.quaternion_from_euler(euler_g[0], euler_g[1], euler_g[2], axes='sxyz')

    while not rospy.is_shutdown():
        #br.sendTransform((-1.0, -0.02, 0.0),
        #br.sendTransform((-26*0.025, 23*0.025, 0.1),
        br.sendTransform((-37*0.025, 10*0.025, 0.1),
                         (quaternion[0], quaternion[1], quaternion[2], quaternion[3]),
                         rospy.Time.now(),
                         "usb_cam",
                         "base_link")

	br.sendTransform((0.16, 0.0, 0.0),
                         (quaternion_g[0], quaternion_g[1], quaternion_g[2], quaternion_g[3]),
                         rospy.Time.now(),
                         "gripper",
                         "ee_link")

        br.sendTransform((0.124+0.16, 0.0, -0.03),
                         (quaternion_g[0], quaternion_g[1], quaternion_g[2], quaternion_g[3]),
                         rospy.Time.now(),
                         "two_finger_tip",
                         "ee_link")
        rate.sleep()
