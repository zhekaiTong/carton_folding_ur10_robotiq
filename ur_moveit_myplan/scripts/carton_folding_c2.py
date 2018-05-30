#!/usr/bin/env python

from apriltags_ros.msg import AprilTagDetection, AprilTagDetectionArray
import numpy
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import time
import roslib; roslib.load_manifest('ur_driver')
import actionlib
import tf
import math
from control_msgs.msg import *
from trajectory_msgs.msg import *
from sensor_msgs.msg import JointState
from math import pi, sin, cos, atan2
from moveit_msgs.msg import RobotTrajectory
from moveit_msgs.msg import *
from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import PoseStamped, Pose
from math import sqrt, pi, acos, sin, cos
from std_msgs.msg import String, UInt16, Int8, Char
from robotiq_force_torque_sensor.msg import ft_sensor

listener = tf.TransformListener()
last_Fz = 0
is_force = 0
gripper_pub = 0
start_lips_r = 0
start_lips_l = 0
start_lips_t = 0
c_l = 0.162
c_w = 0.105
c_h = 0.085

## First initialize moveit_commander and rospy.
print "============ Starting tutorial setup"
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('carton_folding',
                  anonymous=True)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group = moveit_commander.MoveGroupCommander("manipulator")

print "============ Waiting for RVIZ..."
print "============ Starting tutorial "

print "============ Reference frame: %s" % group.get_planning_frame()

print "============ Reference frame: %s" % group.get_end_effector_link()

print "============ Robot Groups:"
print robot.get_group_names()

print "============ Printing robot state"
print robot.get_current_state()
print "============"
  
display_trajectory_publisher = rospy.Publisher(
                                    '/move_group/display_planned_path',
                                    moveit_msgs.msg.DisplayTrajectory,
                                    queue_size=20)

def scale_trajectory_speed(traj, scale):
       # Create a new trajectory object
       new_traj = RobotTrajectory()
      
       # Initialize the new trajectory to be the same as the planned trajectory
       new_traj.joint_trajectory = traj.joint_trajectory
      
       # Get the number of joints involved
       n_joints = len(traj.joint_trajectory.joint_names)
      
       # Get the number of points on the trajectory
       n_points = len(traj.joint_trajectory.points)
       
       # Store the trajectory points
       points = list(traj.joint_trajectory.points)
      
       # Cycle through all points and scale the time from start, speed and acceleration
       for i in range(n_points):
           point = JointTrajectoryPoint()
           point.time_from_start = traj.joint_trajectory.points[i].time_from_start / scale
           point.velocities = list(traj.joint_trajectory.points[i].velocities)
           point.accelerations = list(traj.joint_trajectory.points[i].accelerations)
           point.positions = traj.joint_trajectory.points[i].positions
                        
           for j in range(n_joints):
               point.velocities[j] = point.velocities[j] * scale
               point.accelerations[j] = point.accelerations[j] * scale * scale
           
           points[i] = point

       # Assign the modified points to the new trajectory
       new_traj.joint_trajectory.points = points

       # Return the new trajecotry
       return new_traj

def add_collision_object(xp, yp, zp, rx, ry, rz, rw, x_length, y_length, z_length, name):
  ## Add collision object
  obj_pose = geometry_msgs.msg.PoseStamped()
  obj_pose.header.frame_id = robot.get_planning_frame()
  obj_pose.pose.position.x = xp
  obj_pose.pose.position.y = yp
  obj_pose.pose.position.z = zp
  obj_pose.pose.orientation.x = rx
  obj_pose.pose.orientation.y = ry
  obj_pose.pose.orientation.z = rz
  obj_pose.pose.orientation.w = rw
  scene.add_box(name, obj_pose, (x_length, y_length, z_length)) # x_axis, y_axis, z_axis
  #print "add collision", obj_pose

def quat2eular(qx, qy, qz, qw):
  quaternion = (
      qx,
      qy,
      qz,
      qw)
  euler = tf.transformations.euler_from_quaternion(quaternion, axes='sxyz')
  return euler

def go_to_home():
  group.clear_pose_targets()
  group_variable_values = group.get_current_joint_values()
  print "============ Joint values: ", group_variable_values
  group_variable_values[0] = -pi*20/180#-pi*50/180#pi*20/180
  group_variable_values[1] = -pi*120/180#-pi*112/180#-pi*90/180
  group_variable_values[2] = -pi*116/180#-pi*130/180#-pi*162/180
  group_variable_values[3] = -(pi*2+group_variable_values[1]+group_variable_values[2])
  group_variable_values[4] = -pi*135/180+group_variable_values[0]#group_variable_values[0]-pi*45/180#-pi*135/180+group_variable_values[0]#-pi*1/2+group_variable_values[0]
  group_variable_values[5] = pi*45/180
  group.set_joint_value_target(group_variable_values)
  plan = group.plan()
  print "============ Waiting while RVIZ displays plan2..."
  rospy.sleep(2)
  scaled_traj2 = scale_trajectory_speed(plan, 0.2)
  group.execute(scaled_traj2)

def move_waypoints(px, py, pz, vel):
  waypoints = []
  waypoints.append(group.get_current_pose().pose)
  wpose = copy.deepcopy(group.get_current_pose().pose)
  wpose.position.x = px
  wpose.position.y = py
  wpose.position.z = pz
  waypoints.append(copy.deepcopy(wpose))
  (plan, fraction) = group.compute_cartesian_path(
                               waypoints,   # waypoints to follow
                               0.01,        # eef_step
                               0.0)         # jump_threshold
  print "============ Waiting while RVIZ displays plan3..."
  scaled_traj = scale_trajectory_speed(plan, vel)
  group.execute(scaled_traj)       

def move_joint(theta0, theta1, theta2, theta3, theta4, theta5, vel):
  group.clear_pose_targets()
  group_variable_values = group.get_current_joint_values()
  group_variable_values[0] += theta0
  group_variable_values[1] += theta1
  group_variable_values[2] += theta2
  group_variable_values[3] += theta3
  group_variable_values[4] += theta4
  group_variable_values[5] += theta5
  group.set_joint_value_target(group_variable_values)
  plan = group.plan()
  print "============ Waiting while RVIZ displays plan2..."
  scaled_traj2 = scale_trajectory_speed(plan, vel)
  group.execute(scaled_traj2)

def move_target(x, y, z, ox, oy, oz, ow, vel):
  pose_target = geometry_msgs.msg.Pose()
  pose_target.orientation.x = ox
  pose_target.orientation.y = oy
  pose_target.orientation.z = oz
  pose_target.orientation.w = ow
  pose_target.position.x = x
  pose_target.position.y = y
  pose_target.position.z = z
  group.set_pose_target(pose_target)
  plan = group.plan()
  scaled_traj = scale_trajectory_speed(plan, vel)
  print "============ Waiting while RVIZ displays plan1..."
  group.execute(scaled_traj)

#def move_tool(x, y, z, rx, ry, rz, vel):
#  curr_x = group.get_current_pose().pose.position.x
#  curr_y = group.get_current_pose().pose.position.y
#  curr_z = group.get_current_pose().pose.position.z
#  curr_x_ori = group.get_current_pose().pose.orientation.x
#  curr_y_ori = group.get_current_pose().pose.orientation.y
#  curr_z_ori = group.get_current_pose().pose.orientation.z
#  curr_w_ori = group.get_current_pose().pose.orientation.w
#  eff_curr_euler = quat2eular(curr_x_ori, curr_y_ori, curr_z_ori, curr_w_ori)
#  curr_rot_mat = tf.transformations.euler_matrix(eff_curr_euler[0], eff_curr_euler[1], eff_curr_euler[2], axes='sxyz')
  
##  (g_base_trans,g_base_rot) = listener.lookupTransform('/base_link', '/gripper', rospy.Time(0)) #express frame arg2 in frame arg1
##  g_base_e = quat2eular(g_base_rot[0], g_base_rot[1], g_base_rot[2], g_base_rot[3])
##  curr_rot_mat = tf.transformations.euler_matrix(g_base_e[0], g_base_e[1], g_base_e[2], axes='sxyz')
##  print g_base_trans
##  print g_base_e[0]*180/pi, g_base_e[1]*180/pi, g_base_e[2]*180/pi
##  #print "before rotate", curr_rot_mat
##
#  zaxis = (0, 0, 1)
#  yaxis = (0, 1, 0)
#  xaxis = (1, 0, 0)
#  Rx = tf.transformations.rotation_matrix(rx, xaxis)
#  Ry = tf.transformations.rotation_matrix(ry, yaxis)
#  Rz = tf.transformations.rotation_matrix(rz, zaxis)
#  desire_rot_mat = numpy.dot(curr_rot_mat, Rz)
#  desire_rot_mat = numpy.dot(desire_rot_mat, Ry)
#  desire_rot_mat = numpy.dot(desire_rot_mat, Rx)
##  #print "after rotate", desire_rot_mat
#  desire_euler = tf.transformations.euler_from_matrix(desire_rot_mat, 'sxyz')
#  desire_quaternion = tf.transformations.quaternion_from_euler(desire_euler[0], desire_euler[1], desire_euler[2], axes='sxyz')
##  print "rotate after:", desire_euler[0]*180/pi, desire_euler[1]*180/pi, desire_euler[2]*180/pi
##
#  move_tool_xyz = numpy.array([x, y, z, 1])
#  #desire_trans_mat = numpy.dot(curr_rot_mat, (move_tool_xyz))
#  curr_rot_mat[:3,3] = numpy.array([curr_x, curr_y, curr_z]) # m[:3, 3]: the first three rows in the fourth column
#  desire_trans_mat = numpy.dot(curr_rot_mat, move_tool_xyz)
##  print "after trans", desire_trans_mat

#  move_target(desire_trans_mat[0], desire_trans_mat[1], desire_trans_mat[2], desire_quaternion[0], desire_quaternion[1], desire_quaternion[2], desire_quaternion[3], vel)

def move_frame(x, y, z, rx, ry, rz, vel, tg_frame):
  (base_g_trans,base_g_rot) = listener.lookupTransform('/base_link', tg_frame, rospy.Time(0)) #express frame arg2 in frame arg1
  base_g_rot_mat = tf.transformations.quaternion_matrix(base_g_rot)
  zaxis = (0, 0, 1)
  yaxis = (0, 1, 0)
  xaxis = (1, 0, 0)
  Rx = tf.transformations.rotation_matrix(rx, xaxis)
  Ry = tf.transformations.rotation_matrix(ry, yaxis)
  Rz = tf.transformations.rotation_matrix(rz, zaxis)
  base_g_rot_mat_new = numpy.dot(base_g_rot_mat, Rx)
  base_g_rot_mat_new = numpy.dot(base_g_rot_mat_new, Ry)
  base_g_rot_mat_new = numpy.dot(base_g_rot_mat_new, Rz)
  #print "new gri mat", base_g_rot_mat_new
  move_frame_xyz = numpy.array([x, y, z, 1])
  base_g_rot_mat[:3,3] = numpy.array(base_g_trans)
  base_g_trans_new = numpy.dot(base_g_rot_mat, move_frame_xyz)
  base_g_rot_mat_new[:3,3] = numpy.array([base_g_trans_new[0], base_g_trans_new[1], base_g_trans_new[2]])

  (g_ee_trans,g_ee_rot) = listener.lookupTransform(tg_frame, '/ee_link', rospy.Time(0)) #express frame arg2 in frame arg1
  g_ee_rot_mat = tf.transformations.quaternion_matrix(g_ee_rot)
  g_ee_rot_mat[:3,3] = numpy.array(g_ee_trans)
  
  base_ee_homo_new = numpy.dot(base_g_rot_mat_new, g_ee_rot_mat)
  desire_ee_trans = base_ee_homo_new[:3,3]
  #base_ee_homo_new[:3,3] = numpy.array([0, 0, 0])
  desire_ee_euler = tf.transformations.euler_from_matrix(base_ee_homo_new, axes='sxyz')
  desire_ee_q = tf.transformations.quaternion_from_euler(desire_ee_euler[0], desire_ee_euler[1], desire_ee_euler[2], axes='sxyz')
  if rx != 0.0 or ry!= 0.0 or rz != 0.0:
    move_target(desire_ee_trans[0], desire_ee_trans[1], desire_ee_trans[2], desire_ee_q[0], desire_ee_q[1], desire_ee_q[2], desire_ee_q[3], vel)
  else:
    move_waypoints(desire_ee_trans[0], desire_ee_trans[1], desire_ee_trans[2], vel)

def gripper_callback(data):
  global is_force
  global last_Fz
  global start_lips_r
  global c_l, c_w, c_h
  gripper_pub = rospy.Publisher('/robot_gripper_auto_control', String, queue_size=10)
  if is_force == 0:
    rospy.sleep(2)
    gripper_pub.publish('r')
    rospy.sleep(2)    
    gripper_pub.publish('a')
    rospy.sleep(2)
    gripper_pub.publish('p')
    last_Fz = data.Fz
    is_force = 1
  if abs(data.Fz-last_Fz) >= 20 and start_lips_r == 0:
    gripper_pub.publish('c')
    rospy.sleep(2)
    #move_frame(0.1+0.1*1.414, 0, 0, 0, 0, 0, 0.5, 'gripper')
    move_frame(0.1*1.414, 0, 0, 0, 0, 0, 0.5, 'gripper')
    gripper_pub.publish('o')
    rospy.sleep(1)
    move_frame(-0.115, 0.01, 0, 0, 0, 0, 0.5, 'gripper')
    rospy.sleep(0.1)
    gripper_pub.publish('b')
    rospy.sleep(1)
    gripper_pub.publish('c')
    rospy.sleep(2)
    move_frame(0.07, 0, 0, 0, 0, 0, 0.5, 'gripper')
    l1 = (c_l+c_w)*cos(pi/4)-c_w
    l2 = (c_l+c_w)*cos(pi/4)-c_l
    mx = sqrt(l1*l1+l2*l2)*cos(atan2(l1,l2)-45)
    mz = sqrt(l1*l1+l2*l2)*sin(atan2(l1,l2)-45)
    rospy.sleep(.1)
    #move_frame(mx, 0, mz+0.04, 0, 0, 0, 0.5, 'gripper') #carton collapse to left
    move_frame(mx-0.01, 0, mz+0.005, 0, 0, 0, 0.2, 'gripper')
    #move_frame((c_l+c_w)*cos(pi/4)-c_w-0.01, (c_l+c_w)*cos(pi/4)-c_l-0.01, 0, 0, 0, 0, 0.2, 'gripper')
    rospy.sleep(.1)
    move_frame(-0.02, 0, 0.02, 0, 0, 0, 0.5, 'gripper')
    rospy.sleep(.1)
    move_frame(0, -0.21, 0, 0, 0, 0, 0.5, 'gripper')
    gripper_pub.publish('o')
    rospy.sleep(1)
    gripper_pub.publish('p')
    start_lips_r = 1
    
def cam_callback(cam_pose):
  if cam_pose.detections != []:
    gripper_pub = rospy.Publisher('/robot_gripper_auto_control', String, queue_size=10)
    global start_lips_r
    global start_lips_l
    global start_lips_t
    global c_l, c_w, c_h
    ## ONLY FOR TEST!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    global is_force
    if start_lips_r == 1:
	    #print "camera rx ry rz", eff_curr_euler[0]*180/pi, eff_curr_euler[1]*180/pi, eff_curr_euler[2]*180/pi 
	    (g_tag_trans,g_tag_rot) = listener.lookupTransform('/gripper', '/april_tag_frame_id_0', rospy.Time(0)) #express frame arg2 in frame arg1
            (c_tag_trans,c_tag_rot) = listener.lookupTransform('/usb_cam', '/april_tag_frame_id_0', rospy.Time(0))
            c_tag_euler = quat2eular(c_tag_rot[0], c_tag_rot[1], c_tag_rot[2], c_tag_rot[3])
            (b_g_trans,b_g_rot) = listener.lookupTransform('/base_link', '/april_tag_frame_id_0', rospy.Time(0))
            add_collision_object(b_g_trans[0], b_g_trans[1], b_g_trans[2], b_g_rot[0], b_g_rot[1], b_g_rot[2], b_g_rot[3], c_l, c_w, 0.085, "carton")
	    euler = quat2eular(g_tag_rot[0], g_tag_rot[1], g_tag_rot[2], g_tag_rot[3])
	    g_tag_homo = tf.transformations.euler_matrix(euler[0], euler[1], euler[2], axes='sxyz')
	    g_tag_homo[:3,3] = numpy.array([g_tag_trans[0], g_tag_trans[1], g_tag_trans[2]])
	    tag_r = numpy.array([-c_l/2, 0, -c_w/2, 1])
	    desire_trans_mat = numpy.dot(g_tag_homo, tag_r)
	    grip_rot_angle = pi/4+c_tag_euler[1]#-(pi/2+euler[1])
            rospy.sleep(0.1)
	    #move_frame(0.1, 0, 0.05, 0, 0, 0, 0.2, 'gripper')
	    #move_frame(desire_trans_mat[0]-0.14, 0, 2*desire_trans_mat[2], 0, 0, 0, 0.2, 'gripper')#euler[0], euler[1], euler[2], 0.2)
            move_frame(desire_trans_mat[0]-0.14, 0, 2*desire_trans_mat[2], 0, 0, 0, 0.5, 'gripper')#euler[0], euler[1], euler[2], 0.2)
            rospy.sleep(0.1)
            move_frame(0, 0, 0, 0, grip_rot_angle, 0, 0.5, 'gripper')
            #rospy.sleep(1)
            #move_frame(0, 0.20, 0, 0, 0, 0, 0.2, 'gripper')
            rospy.sleep(0.1)
            move_frame(0, 0, 0, -pi/2, 0, 0, 0.5, 'gripper')
            rospy.sleep(0.1)
            #move_frame(0, 0, 0.11, 0, 0, 0, 0.2, 'gripper')
            gripper_pub.publish('75')
            rospy.sleep(.2)
            move_frame(-0.02, 0, 0.2, 0, 0, 0, 0.5, 'gripper')
            rospy.sleep(0.1)
            move_frame(0, -0.05, 0, 0, 0, 0, 0.5, 'gripper')
            rospy.sleep(0.1)
            move_frame(0.02, -0.01, 0, 0, 0, 0, 0.5, 'gripper') ##################  0.03, 0
            rospy.sleep(0.1)
            for i in range(0,8):
              move_frame(0.006, 0, 0, 0, 0, 0, 0.5, 'two_finger_tip')
              rospy.sleep(0.1)
              move_frame(0, 0, 0, 0, -pi/20, 0, 0.5, 'two_finger_tip')
              rospy.sleep(0.1)
              move_frame(0, 0, 0.002, 0, 0, 0, 0.5, 'two_finger_tip')
              rospy.sleep(0.1)
            gripper_pub.publish('c')
            rospy.sleep(2)
            gripper_pub.publish('75')
            rospy.sleep(.1)
	    #euler = quat2eular(base_cam_rot[0], base_cam_rot[1], base_cam_rot[2], base_cam_rot[3])
	    #print "trans", base_cam_trans
	    print "rot", euler[0]*180/pi, euler[1]*180/pi, euler[2]*180/pi
            start_lips_r = 2

    if start_lips_r == 2 and start_lips_l == 0:
            (b_g_trans,b_g_rot) = listener.lookupTransform('/base_link', '/gripper', rospy.Time(0))
            (c_tag_trans,c_tag_rot) = listener.lookupTransform('/usb_cam', '/april_tag_frame_id_0', rospy.Time(0))
            c_tag_euler = quat2eular(c_tag_rot[0], c_tag_rot[1], c_tag_rot[2], c_tag_rot[3])
            (tag_t_trans,tag_t_rot) = listener.lookupTransform('/april_tag_frame_id_0', '/two_finger_tip', rospy.Time(0))
            b_g_euler = quat2eular(b_g_rot[0], b_g_rot[1], b_g_rot[2], b_g_rot[3])
            grip_rot_angle = b_g_euler[1]-pi/2
            move_frame(-(0.209-(0.044+tag_t_trans[1])), 0, 0, 0, 0, 0, 0.5, 'gripper')
            rospy.sleep(0.1)
            #move_frame(0, 0, 0, 0, grip_rot_angle, 0, 0.2, 'gripper')
            #rospy.sleep(0.1)
            move_joint(0, 0, 0, pi+grip_rot_angle, 0, 0, 0.5)
            rospy.sleep(0.1)
            move_joint(0, 0, 0, 0, pi, 0, 0.5)
            rospy.sleep(0.1)
            ## move to the top lip
            (g_tag_trans,g_tag_rot) = listener.lookupTransform('/gripper', '/april_tag_frame_id_0', rospy.Time(0))
            g_tag_euler = quat2eular(g_tag_rot[0], g_tag_rot[1], g_tag_rot[2], g_tag_rot[3])
            g_tag_homo = tf.transformations.euler_matrix(g_tag_euler[0], g_tag_euler[1], g_tag_euler[2], axes='sxyz')
            g_tag_homo[:3,3] = numpy.array(g_tag_trans)
            #print "check g_tag_homo", g_tag_homo
            tag_t = numpy.array([0, c_h+c_w+0.0215-0.044, 0, 1])
            desire_trans_mat = numpy.dot(g_tag_homo, tag_t)
            #print "check trans mat", desire_trans_mat
            move_frame(0, desire_trans_mat[1], desire_trans_mat[2], 0, 0, 0, 0.5, 'gripper')
            rospy.sleep(0.1)
            move_frame(0, -0.01, 0, 0, 0, 0, 0.2, 'gripper')
            rospy.sleep(0.1)
            #move_frame(0, 0, -0.02, 0, 0, 0, 0.2, 'gripper')
            #rospy.sleep(0.1)
            move_frame(0, 0, 0, pi/2-c_tag_euler[2], 0, 0, 0.5, 'gripper')
            rospy.sleep(0.1)
            move_frame(0.12, 0, 0, 0, 0, 0, 0.2, 'gripper')#########?????????????????
            rospy.sleep(0.1)
            gripper_pub.publish('c')
            rospy.sleep(1)
            move_frame(-0.12, 0, 0, 0, 0, 0, 0.2, 'gripper')
            rospy.sleep(0.1)
            move_joint(0, 0, 0, 0, 0, -pi, 0.5)
#            move_frame(0, 0, 0, -(pi-c_tag_euler[2])-pi*2, 0, 0, 0.5, 'gripper')
            print "check top lips rotate:", -(pi-c_tag_euler[2])-pi*2
            rospy.sleep(0.1)
            move_frame(0.12-0.01-0.01, 0, 0, 0, 0, 0, 0.2, 'gripper')
            rospy.sleep(0.1)
            (g_cam_trans,g_cam_rot) = listener.lookupTransform('/gripper', '/usb_cam', rospy.Time(0))
            g_cam_euler = quat2eular(g_cam_rot[0], g_cam_rot[1], g_cam_rot[2], g_cam_rot[3])
            g_cam_homo = tf.transformations.euler_matrix(g_cam_euler[0], g_cam_euler[1], g_cam_euler[2], axes='sxyz')
            g_cam_homo[:3,3] = numpy.array(g_cam_trans)
            fix_frame = numpy.array([3*0.025, 0, 14*0.025, 1])
            desire_trans_mat = numpy.dot(g_cam_homo, fix_frame)
            print "check trans mat", desire_trans_mat
            move_frame(0, desire_trans_mat[1]+c_l/2-0.018, 0, 0, 0, 0, 0.2, 'gripper')
            rospy.sleep(0.1)
            move_frame(0, 0, desire_trans_mat[2], 0, 0, 0, 0.2, 'gripper')
            rospy.sleep(0.1)
#            move_frame(0, desire_trans_mat[1]+c_l/2-0.018, 0, 0, 0, 0, 0.2, 'gripper')
#            rospy.sleep(0.1)
            #move_frame(0, 0, -0.01, 0, 0, 0, 0.2, 'gripper')
            gripper_pub.publish('75')
            #move_frame(-0.12, 0, 0, 0, 0, 0, 0.2, 'gripper')
            #rospy.sleep(0.1)

            # move away from carton top to fold left (now right) lip
            (g_tag_trans,g_tag_rot) = listener.lookupTransform('/gripper', '/april_tag_frame_id_0', rospy.Time(0))
            g_tag_euler = quat2eular(g_tag_rot[0], g_tag_rot[1], g_tag_rot[2], g_tag_rot[3])
            g_tag_homo = tf.transformations.euler_matrix(g_tag_euler[0], g_tag_euler[1], g_tag_euler[2], axes='sxyz')
            g_tag_homo[:3,3] = numpy.array(g_tag_trans)
            tag_l = numpy.array([-c_l/2, 0, -c_w/2, 1])
            desire_trans_mat = numpy.dot(g_tag_homo, tag_l)
            rospy.sleep(1)
            move_frame(0, desire_trans_mat[1]+0.16, 0, 0, 0, 0, 0.5, 'gripper')
            #move_frame(-0.12, 0, 0, 0, 0, 0, 0.2, 'gripper')
            #rospy.sleep(0.1)
            rospy.sleep(0.1)
            move_frame(0, 0, 0, 0, 0, -pi/4, 0.5, 'gripper')########## pi/2 -> pi/4
            rospy.sleep(0.1)
            move_frame(0, 0, 0, -pi/2, 0, 0, 0.5, 'gripper')
            rospy.sleep(0.1)
            (t_tag_trans,t_tag_rot) = listener.lookupTransform('/two_finger_tip', '/april_tag_frame_id_0', rospy.Time(0))
            t_tag_euler = quat2eular(t_tag_rot[0], t_tag_rot[1], t_tag_rot[2], t_tag_rot[3])
            t_tag_homo = tf.transformations.euler_matrix(t_tag_euler[0], t_tag_euler[1], t_tag_euler[2], axes='sxyz')
            t_tag_homo[:3,3] = numpy.array(t_tag_trans)
            tag_l = numpy.array([-c_l/2, c_h+0.038/2-0.044, -c_w/2, 1])
            desire_trans_mat = numpy.dot(t_tag_homo, tag_l)
            move_frame(desire_trans_mat[0], desire_trans_mat[1], desire_trans_mat[2], 0, 0, 0, 0.5, 'two_finger_tip')
            now_x = group.get_current_pose().pose.position.x
            now_y = group.get_current_pose().pose.position.y
            now_z = group.get_current_pose().pose.position.z
            move_waypoints(now_x-0.01, now_y, now_z-0.07, 0.2)
            rospy.sleep(0.1)
            move_frame(0.03, 0, 0, 0, 0, 0, 0.5, 'gripper')
            rospy.sleep(0.1)
            for i in range(0,2):
              move_frame(0, 0, 0, 0, -pi/20, 0, 0.5, 'two_finger_tip')
              rospy.sleep(0.1)
              move_frame(-0.005, 0, 0.002, 0, 0, 0, 0.5, 'two_finger_tip')
              rospy.sleep(0.1)
              move_frame(0.01, 0, 0, 0, 0, 0, 0.5, 'two_finger_tip')
              rospy.sleep(0.1)
            gripper_pub.publish('c')
            rospy.sleep(2)
            gripper_pub.publish('75')
            rospy.sleep(0.1)
            start_lips_l = 1

    if start_lips_r == 2 and start_lips_l == 1 and start_lips_t == 0:
            move_frame(-0.1, 0, 0, 0, 0, 0, 0.2, 'gripper')
            rospy.sleep(0.1)
            move_frame(0, 0, 0, 0, -3*pi/20, 0, 0.2, 'two_finger_tip')
            rospy.sleep(0.1)
            move_joint(0, 0, 0, 0, 0, -pi/2+pi*2, 0.2)
            rospy.sleep(1)
            for i in range(0,10):
              move_frame(0, 0, 0, 0, -pi/20, 0, 0.2, 'two_finger_tip')
              rospy.sleep(0.1)
            move_frame(-0.05, 0, 0, 0, 0, 0, 0.5, 'gripper')
            #move_frame(0, 0, -0.25, 0, 0, 0, 0.2, 'gripper')
            rospy.sleep(0.1)
            gripper_pub.publish('39') #############$$$$$$$$$$$$$$$$ 40
            rospy.sleep(0.1)
            (t_tag_trans,t_tag_rot) = listener.lookupTransform('/two_finger_tip', '/april_tag_frame_id_0', rospy.Time(0))
            t_tag_euler = quat2eular(t_tag_rot[0], t_tag_rot[1], t_tag_rot[2], t_tag_rot[3])
            t_tag_homo = tf.transformations.euler_matrix(t_tag_euler[0], t_tag_euler[1], t_tag_euler[2], axes='sxyz')
            t_tag_homo[:3,3] = numpy.array(t_tag_trans)
            tag_back = numpy.array([0, 0.03, -c_w, 1])
            desire_trans_mat = numpy.dot(t_tag_homo, tag_back)
            move_frame(desire_trans_mat[0], desire_trans_mat[1], desire_trans_mat[2], 0, 0, 0, 0.5, 'two_finger_tip')
            rospy.sleep(0.1)
            move_frame(0.015, -0.01, -0.025, 0, 0, 0, 0.5, 'two_finger_tip')
            rospy.sleep(0.1)

            ## start to close the top lip
            for i in range(0,9):
              move_frame(0, 0, 0, 0, pi/20, 0, 0.5, 'two_finger_tip')
              rospy.sleep(0.1)
              move_frame(0.005, 0, -0.001, 0, 0, 0, 0.5, 'two_finger_tip')
              rospy.sleep(0.1)
              if i == 7:
                move_frame(0.01, 0, 0.01, 0, 0, 0, 0.5, 'two_finger_tip')
                rospy.sleep(0.1)
#            move_frame(0, 0, 0, 0, pi/20, 0, 0.5, 'two_finger_tip')
#            rospy.sleep(0.1)
            #move_frame(0.01, 0, 0, 0, 0, 0, 0.5, 'two_finger_tip')
            move_frame(-0.01, 0, 0, 0, 0, 0, 0.5, 'two_finger_tip')
            rospy.sleep(0.1)
            move_frame(0, 0, 0.01, 0, 0, 0, 0.5, 'two_finger_tip')
            rospy.sleep(0.1)
            move_frame(0.02, 0, 0, 0, 0, 0, 0.5, 'two_finger_tip')
            rospy.sleep(1)

            ## insert into slot
            move_frame(-0.01, 0, 0, 0, 0, 0, 0.2, 'two_finger_tip')
            rospy.sleep(0.1)
            gripper_pub.publish('40') #############$$$$$$$$$$$$$$$$ 40
            rospy.sleep(0.1)            
            move_frame(0, 0, 0, 0, -2*pi/20, 0, 0.2, 'two_finger_tip')
            rospy.sleep(0.1)
            gripper_pub.publish('48') #############$$$$$$$$$$$$$$$$ 40
            rospy.sleep(0.1)
            move_frame(0, 0, -0.02, 0, 0, 0, 0.2, 'two_finger_tip')
            rospy.sleep(0.1)
            move_frame(0.01, 0, 0, 0, 0, 0, 0.2, 'two_finger_tip')
            rospy.sleep(0.1) 
#            gripper_pub.publish('43')
            start_lips_t = 1

def move_robot():
  add_collision_object(0, 0, -0.2, 0, 0, 0, 0, 2.0, 1.5, 0.25, "table")
  #add_collision_object(0, -0.3, 0, 0, 0, 0, 0, 2.0, 0.01, 1.5, "wall1")
  #move_frame(0.02, 0, 0, 0, 0, 0, 0.2, 'gripper')
  go_to_home()
  global gripper_pub
  gripper_pub = rospy.Publisher('/robot_gripper_auto_control', String, queue_size=10)
  rq_sensor_sub = rospy.Subscriber("robotiq_force_torque_sensor", ft_sensor, gripper_callback, queue_size=1)
  cam_pose_sub = rospy.Subscriber('/tag_detections', AprilTagDetectionArray, cam_callback, queue_size=1)

if __name__=='__main__':
  move_robot()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
moveit_commander.os._exit(0)
