import rospy
import moveit_commander

import math

import moveit_msgs.msg
import numpy as np

from scipy.spatial.transform import Rotation as Rot

from action_library.arm_interface import arm
from action_library.gripper_interface import gripper
from action_library.tf import tf_interface

"""
location names
==============

- sink
- orange_box_pregrasp
- orange_box_grasp
- green_box_pregrasp
- green_box_grasp
"""

object_name_to_marker_id = {
    'orange_box': 'aruco_frame_filtered',
    'green_box': 'aruco_frame_228',
}

object_grasp_offset = np.array([0., 0., 0.12])
object_pregrasp_offset = np.array([0., 0., 2*0.15])

def get_box_goal_pose_inside_cabinet(name):
  p, q = tf_interface.get(name)
  R = Rot.from_quat(q).as_matrix()


  yg = np.array([0., 0., 1.])
  zg = -1. * R[:,0]
  xg = np.cross(yg, zg)

  Rg = np.eye(3)
  Rg[:,0] = xg
  Rg[:,1] = yg
  Rg[:,2] = zg

  qg = Rot.from_matrix(Rg).as_quat()

  pg = np.array(p) + R[:,0] * 0.15 + R[:,2] * -0.03 -0.035 * R[:,1]

  return pg, qg

def reach_to_pre_door_close():
  p, q = tf_interface.get('cabinet_outside_door')
  R = Rot.from_quat(q).as_matrix()
  pg = np.array(p) + R[:,0] * -0.05 + R[:,2] * 0.05 - R[:,1] * 0.185
  arm.move_eff_to(pg, q)

def close_door():
  _, q = tf_interface.get('cabinet_top')
  R = Rot.from_quat(q).as_matrix()


  pose = arm.move_group.get_current_pose().pose
  pos = pose.position
  ori = pose.orientation
  peff = np.array([pos.x, pos.y, pos.z])
  qg = np.array([ori.x, ori.y, ori.z, ori.w])

  pg = peff + R[:,0] * 0.2

  reach_to(pg)

  pg += R[:,1] * 0.11 * -1.
  reach_to(pg)


def get_location(name, offset=[0.,0.,0.]):
  """Returns position for a given location name."""
  p, _ = tf_interface.get(name)
  return (np.array(p) + np.array(offset)).tolist()
  # if name == 'sink':
  #   sink_position = [0.009, 0.698, 0.028 + 2*0.15]
  #   return sink_position
  # if name.endswith('_grasp'):
  #   object_name = name.replace('_grasp','')
  #   marker_id = object_name_to_marker_id[object_name]
  #   p, _ = tf_interface.get(marker_id)
  #   return (np.array(p) + object_grasp_offset).tolist()
  # if name.endswith('_pregrasp'):
  #   object_name = name.replace('_pregrasp','')
  #   marker_id = object_name_to_marker_id[object_name]
  #   p, _ = tf_interface.get(marker_id)
  #   return (np.array(p) + object_pregrasp_offset).tolist()

def open_gripper():
  """Opens gripper."""
  rospy.sleep(1.0)
  gripper.action('o')
  print('opened gripper')
  rospy.sleep(1.0)

def close_gripper():
  """Closes gripper."""
  rospy.sleep(1.0)
  gripper.action('c')
  print('closed gripper')
  rospy.sleep(1.0)

# def reach_to(position):
#   """Reaches to a given end-effector position, while keeping the orientation fixed."""
#   # q = arm.get_current_eff_quaternion()
#   quat = arm.move_group.get_current_pose().pose.orientation
#   q = [quat.x, quat.y, quat.z, quat.w]
#   arm.move_eff_to(position, q)
#   print('moved arm to position')
#   rospy.sleep(1.0)

def reach_to(position, orientation=None):
  """Reaches to a given end-effector position, while keeping the orientation fixed."""
  # q = arm.get_current_eff_quaternion()

  if orientation is None:
    quat = arm.move_group.get_current_pose().pose.orientation
    q = [quat.x, quat.y, quat.z, quat.w]
  else:
    q = orientation
  arm.move_eff_to(position, q)
  print('moved arm to position')
  rospy.sleep(1.0)

def change_orientation(direction):
  """Changes the end-effector orientation, while keeping the position fixed."""
  if direction == 'forward':
    orientation = arm.reference_forward_orientation()
  elif direction == 'down':
    orientation = arm.reference_down_orientation()
  else:
    raise ValueError("did not recognize direction")

  # p = arm.get_current_eff_position()
  pos = arm.move_group.get_current_pose().pose.position
  p = [pos.x, pos.y, pos.z]
  arm.move_eff_to(p, orientation)
  print('moved arm to orientation')
  rospy.sleep(1.0)

def turn_wrist(angle):
  goal = list(arm.get_current_joint_state().position)
  goal[-1] += math.radians(angle)
  arm.move_to_joint_position(goal)
  rospy.sleep(1.0)


def pour():
  """Performs the pour action"""
  turn_wrist(-60.0)
  print('poured')


# def stir_cup():
#   """Stire in a small cup"""
#   # q = arm.get_current_eff_quaternion()
#   quat = arm.move_group.get_current_pose().pose.orientation
#   q = [quat.x, quat.y, quat.z, quat.w]
#   print(q)
#   current = arm.reference_down_orientation()
#   print("This is current")
#   print(current)
#   diff = (q - current) < 0.5
#   print(diff)
#   if diff.all() :
#     goal = list(arm.get_current_joint_state().position)
#     goal[-1] -= math.radians(180.0)
#     arm.move_to_joint_position(goal)
#     print('stired once')
#     rospy.sleep(1.0)
#     goal = list(arm.get_current_joint_state().position)
#     goal[-1] -= math.radians(-180.0)
#     arm.move_to_joint_position(goal)
#     print('stired twice')
#     rospy.sleep(1.0)
#   else:
#     raise Exception("The gripper is not in down position.")


def stir_cup():
  """Stire in a cup"""
  # q = arm.get_current_eff_quaternion()
  quat = arm.move_group.get_current_pose().pose.orientation
  q = [quat.x, quat.y, quat.z, quat.w]
  current = arm.reference_down_orientation()
  diff = (q - current) < 0.5
  if diff.all() :
    # Define circle parameters
    center_point = arm.move_group.get_current_pose().pose.position
    original_position = center_point
    radius = 0.02  # adjust as needed
    num_points = 6  # adjust as needed

    # Calculate trajectory points on the circle
    trajectory_points = []
    for i in range(num_points):
        theta = i * (2 * math.pi / num_points)
        x = center_point.x + radius * math.cos(theta)
        y = center_point.y + radius * math.sin(theta)
        z = center_point.z
        trajectory_points.append([x, y, z])

    # Move the robot through the trajectory points
    trajectory_points.append([original_position.x, original_position.y, original_position.z])

    arm.pass_eff_to(trajectory_points,q)  # move to the next point
    rospy.sleep(0.5)
    # for point in trajectory_points:
    #     arm.move_eff_to(point, q)  # move to the next point
    #     rospy.sleep(0.5)  # adjust sleep duration as needed

    print('Stirred in the cup')

  else:
    raise Exception("The gripper is not in down position.")


def reach_to_pregrasp_bowl():
  p, q = tf_interface.get('bowl')

  R = Rot.from_quat(q).as_matrix()

  yg = np.array([0., 0., 1.])
  zg = -1. * R[:3, 0]
  xg = np.cross(yg, zg)

  Rg = np.eye(3)
  Rg[:,0] = xg
  Rg[:,1] = yg
  Rg[:,2] = zg
  qg = Rot.from_matrix(Rg).as_quat()
  pg = np.asarray(p) + R[:,2] * 0.085 + R[:,0] * 0.26

  arm.move_eff_to(pg, qg)

  turn_wrist(90)

def reach_to_cabinet():
  p, q = tf_interface.get('cabinet_outside_door')

  yg = np.array([0., 0., 1.])
  zg = -1.0 * Rot.from_quat(q).as_matrix()[:3, 2]
  xg = np.cross(yg, zg)

  Rg = np.eye(3)
  Rg[:,0] = xg
  Rg[:,1] = yg
  Rg[:,2] = zg
  qg = Rot.from_matrix(Rg).as_quat()
  pg = np.asarray(p) - zg * 0.2 + 0.045 * xg - 0.05 * yg

  arm.move_eff_to(pg, qg)

def open_cabinate():

  # Open the door a bit
  pose = arm.move_group.get_current_pose().pose
  pos = pose.position
  ori = pose.orientation
  peff = [pos.x, pos.y, pos.z]
  qeff = [ori.x, ori.y, ori.z, ori.w]
  Reff_ = Rot.from_quat(qeff)
  Rg_= Rot.from_euler('Z', 30, degrees=True) * Reff_
  Reff = Reff_.as_matrix()
  qg = Rg_.as_quat()
  Rg = Rg_.as_matrix()
  pg = np.array(peff) - 0.075 * Rg[:3,2] - 0.05 * Reff[:3,0]
  arm.move_eff_to(pg, qg)

  # Open the gripper
  open_gripper()

  # Move arm away
  pose = arm.move_group.get_current_pose().pose
  pos = pose.position
  ori = pose.orientation
  peff = [pos.x, pos.y, pos.z]
  qeff = [ori.x, ori.y, ori.z, ori.w]
  Reff = Rot.from_quat(qeff).as_matrix()
  pg = np.array(peff) - Reff[:3, 2] * 0.05 + Reff[:3, 0] * 0.05
  arm.move_eff_to(pg, qeff)


  # Move to door side
  dr_out_p, dr_out_q = tf_interface.get('cabinet_outside_door')

  yg = np.array([0., 0., 1.])
  zg = -1. * Rot.from_quat(np.array(dr_out_q)).as_matrix()[:3, 0]
  xg = np.cross(yg, zg)

  Rg = np.eye(3)
  Rg[:,0] = xg
  Rg[:,1] = yg
  Rg[:,2] = zg
  qg = Rot.from_matrix(Rg).as_quat()

  pg_ = dr_out_p - 0.35*zg
  pg_[2] = pg[2]

  arm.move_eff_to(pg_, qg)

  # Close gripper
  close_gripper()

  #  Move arm towards door
  pose = arm.move_group.get_current_pose().pose
  pos = pose.position
  ori = pose.orientation
  peff = [pos.x, pos.y, pos.z]
  qeff = [ori.x, ori.y, ori.z, ori.w]
  Reff = Rot.from_quat(qeff).as_matrix()
  pg = np.array(peff) + Reff[:3, 2] * 0.12 + Reff[:3, 0] * 0.05
  arm.move_eff_to(pg, qeff)

  # Fully open door
  pose = arm.move_group.get_current_pose().pose
  pos = pose.position
  ori = pose.orientation
  peff = [pos.x, pos.y, pos.z]
  qeff = [ori.x, ori.y, ori.z, ori.w]
  Reff = Rot.from_quat(qeff).as_matrix()
  pg = np.array(peff) + Reff[:3, 2] * 0.2 - Reff[:3, 0] * 0.2
  arm.move_eff_to(pg, qeff)

  # Look into cabinet
  top_p, top_q = tf_interface.get('cabinet_top')

  yg = np.array([0., 0., 1.])
  zg = -1. * Rot.from_quat(np.array(top_q)).as_matrix()[:3, 1]
  xg = np.cross(yg, zg)

  Rg = np.eye(3)
  Rg[:,0] = xg
  Rg[:,1] = yg
  Rg[:,2] = zg
  qg = Rot.from_matrix(Rg).as_quat()


  RR = Rot.from_quat(np.array(top_q)).as_matrix()

  pg = np.array(top_p) + RR[:3,2] * -0.15 + RR[:3, 0] * -0.15 + 0.3 * RR[:3,1]

  arm.move_eff_to(pg, qg)

def stir_bowl():
  """Stire in a big bowl"""
  # q = arm.get_current_eff_quaternion()
  quat = arm.move_group.get_current_pose().pose.orientation
  q = [quat.x, quat.y, quat.z, quat.w]
  current = arm.reference_down_orientation()
  diff = (q - current) < 0.5
  if diff.all() :
    # Define circle parameters
    center_point = arm.move_group.get_current_pose().pose.position
    original_position = center_point
    radius = 0.1  # adjust as needed
    num_points = 6  # adjust as needed

    # Calculate trajectory points on the circle
    trajectory_points = []
    for i in range(num_points):
        theta = i * (2 * math.pi / num_points)
        x = center_point.x + radius * math.cos(theta)
        y = center_point.y + radius * math.sin(theta)
        z = center_point.z
        trajectory_points.append([x, y, z])

    # Move the robot through the trajectory points
    trajectory_points.append([original_position.x, original_position.y, original_position.z])

    arm.pass_eff_to(trajectory_points,q)  # move to the next point
    rospy.sleep(0.5)
    # for point in trajectory_points:
    #     arm.move_eff_to(point, q)  # move to the next point
    #     rospy.sleep(0.5)  # adjust sleep duration as needed

    print('Stirred in the bowl')

  else:
    raise Exception("The gripper is not in down position.")


def align_with(name):
  """Aligns the end-effector orientation with the target orientation."""
  # Get the current end-effector position
  current_position = arm.move_group.get_current_pose().pose.position

  # Move the robot to the current position with the target orientation

  rospy.sleep(1.0)


