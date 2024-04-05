import sys
import rospy
import moveit_commander
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
import tf.transformations as tfs

import numpy as np

import math

from optas import RobotModel

from scipy.spatial.transform import Rotation as Rot

class Arm:

  group_name = "manipulator"

  def __init__(self):
    moveit_commander.roscpp_initialize(sys.argv)
    self.move_group = moveit_commander.MoveGroupCommander(self.group_name)

  def get_current_joint_state(self):
    msg = rospy.wait_for_message('joint_states', JointState)
    return msg

  # def get_current_eff_position(self):
  #   # T = self.fk_transform(self.get_current_joint_state().position)
  #   # return T[:3,3].flatten().tolist()
  #   self.move_group.get_current_pose().pose

  # def get_current_eff_quaternion(self):
  #   T = self.fk_transform(self.get_current_joint_state().position)
  #   return Rot.from_matrix(T[:3,:3]).as_quat().flatten().tolist()

  def move_eff_to(self, position, orientation):

    waypoints = []
    goal_pose = Pose()
    goal_pose.position.x = position[0]
    goal_pose.position.y = position[1]
    goal_pose.position.z = position[2]

    ori_goal = orientation
    goal_pose.orientation.x = ori_goal[0]
    goal_pose.orientation.y = ori_goal[1]
    goal_pose.orientation.z = ori_goal[2]
    goal_pose.orientation.w = ori_goal[3]

    waypoints.append(goal_pose)

    plan, fraction = self.move_group.compute_cartesian_path(waypoints, 0.1, 0.0)
    print("Plan here----------")
    print(plan)
    self.move_group.execute(plan, wait=True)

  def pass_eff_to(self, positions, orientation):

    waypoints = []

    # Assuming positions is a list of lists, where each sublist is a position [x, y, z]
    for position in positions:
        goal_pose = Pose()
        goal_pose.position.x = position[0]
        goal_pose.position.y = position[1]
        goal_pose.position.z = position[2]

        # Assuming the same orientation for all waypoints
        goal_pose.orientation.x = orientation[0]
        goal_pose.orientation.y = orientation[1]
        goal_pose.orientation.z = orientation[2]
        goal_pose.orientation.w = orientation[3]

        waypoints.append(goal_pose)

    # Now waypoints is a list of Pose objects, one for each position
    plan, fraction = self.move_group.compute_cartesian_path(waypoints, 0.1, 0.0)

    # Optionally print the plan and fraction of the path that was successfully planned
    print("Plan here----------")
    print(plan)
    print("Fraction of path successfully planned:", fraction)
    self.move_group.execute(plan, wait=True)


  def move_to_joint_position(self, position):
    # move_group = moveit_commander.MoveGroupCommander(self.group_name)
    self.move_group.set_max_acceleration_scaling_factor(0.3)
    self.move_group.set_max_velocity_scaling_factor(0.3)
    joint_name = self.get_current_joint_state().name
    joint_target = {n: p for n, p in zip(joint_name, position)}
    self.move_group.set_joint_value_target(joint_target)
    self.move_group.go(wait=True)
    self.move_group.stop()
    # self.move_group.clear_pose_targets()

  def reference_forward_orientation(self):
    return tfs.quaternion_from_euler(-math.radians(90), 0, 0)

  def reference_down_orientation(self):
    # x = np.array([1., 0., 0.])
    # y = np.array([0., -1., 0.])
    # z = np.array([0., 0., -1.])
    # R = np.eye(3)
    # R[:,0] = x
    # R[:,1] = y
    # R[:,2] = z
    # return Rot.from_matrix(R).as_quat()
    return tfs.quaternion_from_euler(math.radians(180), 0, 0)

arm = Arm()