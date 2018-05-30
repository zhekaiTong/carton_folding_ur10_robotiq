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
from control_msgs.msg import *
from trajectory_msgs.msg import *
from sensor_msgs.msg import JointState
from math import pi
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

def add_collision_object(x_length, y_length, z_length):
  ## Add collision object
  obj_pose = geometry_msgs.msg.PoseStamped()
  obj_pose.header.frame_id = robot.get_planning_frame()
  obj_pose.pose.position.x = 0
  obj_pose.pose.position.y = 0
  obj_pose.pose.position.z = -0.2
  scene.add_box("table", obj_pose, (x_length, y_length, z_length)) # x_axis, y_axis, z_axis
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
  group_variable_values[0] = pi*20/180
  group_variable_values[1] = -pi*105/180
  group_variable_values[2] = -pi*162/180
  group_variable_values[3] = -(pi*2+group_variable_values[1]+group_variable_values[2])
  group_variable_values[4] = -pi*1/2+group_variable_values[0]
  group_variable_values[5] = pi
  group.set_joint_value_target(group_variable_values)
  plan = group.plan()
  print "============ Waiting while RVIZ displays plan2..."
  rospy.sleep(2)
  scaled_traj2 = scale_trajectory_speed(plan, 0.2)
  group.execute(scaled_traj2)

def move_waypoints(x, y, z, vel):
  waypoints = []
  waypoints.append(group.get_current_pose().pose)
  wpose = copy.deepcopy(group.get_current_pose().pose)
  wpose.position.x += x
  wpose.position.y += y
  wpose.position.z += z
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

def move_tool(x, y, z, rx, ry, rz, vel):
  curr_x = group.get_current_pose().pose.position.x
  curr_y = group.get_current_pose().pose.position.y
  curr_z = group.get_current_pose().pose.position.z
  curr_x_ori = group.get_current_pose().pose.orientation.x
  curr_y_ori = group.get_current_pose().pose.orientation.y
  curr_z_ori = group.get_current_pose().pose.orientation.z
  curr_w_ori = group.get_current_pose().pose.orientation.w
  eff_curr_euler = quat2eular(curr_x_ori, curr_y_ori, curr_z_ori, curr_w_ori)
  curr_rot_mat = tf.transformations.euler_matrix(eff_curr_euler[0], eff_curr_euler[1], eff_curr_euler[2], axes='sxyz')
  
  zaxis = (0, 0, 1)
  yaxis = (0, 1, 0)
  xaxis = (1, 0, 0)
  Rx = tf.transformations.rotation_matrix(rx, xaxis)
  Ry = tf.transformations.rotation_matrix(ry, yaxis)
  Rz = tf.transformations.rotation_matrix(rz, zaxis)
  desire_rot_mat = numpy.dot(curr_rot_mat, Rz)
  desire_rot_mat = numpy.dot(desire_rot_mat, Ry)
  desire_rot_mat = numpy.dot(desire_rot_mat, Rx)
  desire_euler = tf.transformations.euler_from_matrix(desire_rot_mat, 'sxyz')
  desire_quaternion = tf.transformations.quaternion_from_euler(desire_euler[0], desire_euler[1], desire_euler[2], axes='sxyz')

  move_tool_xyz = numpy.array([x, y, z, 1])
  desire_trans_mat = numpy.dot(curr_rot_mat, (move_tool_xyz))
  curr_rot_mat[:3,3] = numpy.array([curr_x, curr_y, curr_z]) # m[:3, 3]: the first three rows in the fourth column
  desire_trans_mat = numpy.dot(curr_rot_mat, move_tool_xyz)

  move_target(desire_trans_mat[0], desire_trans_mat[1], desire_trans_mat[2], desire_quaternion[0], desire_quaternion[1], desire_quaternion[2], desire_quaternion[3], vel)

def gripper_callback(data):
  global is_force
  global last_Fz
  global start_lips_r
  gripper_pub = rospy.Publisher('/robot_gripper_auto_control', String, queue_size=10)
  if is_force == 0:
    rospy.sleep(2)
    gripper_pub.publish('r')
    rospy.sleep(2)    
    gripper_pub.publish('a')
    last_Fz = data.Fz
    is_force = 1
  if abs(data.Fz-last_Fz) >= 20 and start_lips_r == 0:
    gripper_pub.publish('c')
    rospy.sleep(3)
    move_tool(0.075, 0, 0, 0, 0, 0, 0.2)
    gripper_pub.publish('o')
    rospy.sleep(2)
    move_tool(-0.005, 0.01, 0, 0, 0, 0, 0.2)
    move_tool(0.085, 0, 0, 0, 0, 0, 0.2)
    rospy.sleep(1)
    move_tool(-0.15, 0, 0, 0, 0, 0, 0.2)
    
    move_waypoints(0, 0, 0.52, 0.2)
    group_variable_values = group.get_current_joint_values()
    move_joint(0, 0, 0, 0, -group_variable_values[0], 0, 0.2)
    move_joint(0, 0, 0, pi/2, 0, 0, 0.2)
    move_joint(0, 0, 0, 0, pi, 0, 0.2)
    start_lips_r = 1
    
def cam_callback(cam_pose):
  if cam_pose.detections != []:
    gripper_pub = rospy.Publisher('/robot_gripper_auto_control', String, queue_size=10)
    global start_lips_l
    ## ONLY FOR TEST!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    global is_force
    if is_force == 0:
      move_waypoints(0, 0, 0.52, 0.5)
      group_variable_values = group.get_current_joint_values()
      move_joint(0, 0, 0, 0, -group_variable_values[0], 0, 0.5)
      move_joint(0, 0, 0, pi/2, 0, 0, 0.5)
      move_joint(0, 0, 0, 0, pi, 0, 0.5)
      is_force = 1
    if is_force == 1:# and start_lips_l == 0:
	    #print "camera rx ry rz", eff_curr_euler[0]*180/pi, eff_curr_euler[1]*180/pi, eff_curr_euler[2]*180/pi 
	    (ee_tag_trans,ee_tag_rot) = listener.lookupTransform('/ee_link', '/april_tag_frame_id_0', rospy.Time(0)) #express frame arg2 in frame arg1
	    euler = quat2eular(ee_tag_rot[0], ee_tag_rot[1], ee_tag_rot[2], ee_tag_rot[3])
	    ee_tag_homo = tf.transformations.euler_matrix(euler[0], euler[1], euler[2], axes='sxyz')
	    ee_tag_homo[:3,3] = numpy.array([ee_tag_trans[0], ee_tag_trans[1], ee_tag_trans[2]])
	    tag_up = numpy.array([0, 0.25, 0, 1])
	    desire_trans_mat = numpy.dot(ee_tag_homo, tag_up)
	    grip_rot_angle = -(pi/2+euler[1])
	    move_tool(desire_trans_mat[0], desire_trans_mat[1], desire_trans_mat[2], grip_rot_angle, 0, 0, 0.5)#euler[0], euler[1], euler[2], 0.2)
	    #euler = quat2eular(base_cam_rot[0], base_cam_rot[1], base_cam_rot[2], base_cam_rot[3])
	    #print "trans", base_cam_trans
	    print "rot", euler[0]*180/pi, euler[1]*180/pi, euler[2]*180/pi
            #start_lips_l = 1

def move_robot():
  add_collision_object(2.0, 1.5, 0.25)
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
