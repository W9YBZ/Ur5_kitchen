#!/usr/bin/python3
import rospy
rospy.init_node('test_action_lib', anonymous=True)

from action_library.action_library import *

def main():


  reach_to_pregrasp_bowl()


  close_gripper()

  p, _ = tf_interface.get('bowl')

  p += np.array([0., 0., 0.2])

  reach_to(p)



  quit()



  # rospy.sleep(10.0)

  open_gripper()

  reach_to_cabinet()

  # turn_wrist(-20)
  # quit()

  # p = get_location('red_box_1', offset=[0., -0.02, 0.4])
  # reach_to(p)

  # p = get_location('red_box_1', offset=[0., -0.02, 0.25])
  # reach_to(p)

  close_gripper()

  open_cabinate()

  open_gripper()

  pg, qg = get_box_goal_pose_inside_cabinet('green_box_1')

  reach_to(pg, qg)

  close_gripper()

  p = get_location('bowl', offset=[0., 0., 0.45])
  reach_to(p)





  # change_orientation('down')

  # p = get_location('bowl', offset=[0., 0., 0.25])
  # reach_to(p)

  open_gripper()


  quit()






  # p = get_location('red_box_1', offset=[0., -0.02, 0.35])
  # reach_to(p)

  # p = get_location('green_box_2', offset=[0., -0.06, 0.375])
  # reach_to(p)

  # p = get_location('green_box_2', offset=[0., -0.06, 0.3])
  # reach_to(p)

  # change_orientation('down')

  # stir_cup()

  # open_gripper()



  reach_to_pre_door_close()

  change_orientation('down')

  close_gripper()

  close_door()


  # reach_to(get_location('orange_box_pregrasp'))
  # reach_to(get_location('orange_box_grasp'))
  # close_gripper()
  # reach_to(get_location('sink'))
  # open_gripper()
  # reach_to(get_location('green_box_pregrasp'))
  # reach_to(get_location('green_box_grasp'))
  # close_gripper()
  # reach_to(get_location('sink'))
  # open_gripper()
  # stir_bowl()


if __name__ == '__main__':
  main()