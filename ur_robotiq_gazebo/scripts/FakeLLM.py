import sys
import rospy
import action_lib  # Import your action_lib module where all the functions are defined

def main():
    try:
        # # # Initialize ROS node
        # # rospy.init_node('robotic_arm_control', anonymous=True)
        
        # # Initialize MoveIt Commander and other necessary setups from action_lib.py
        # action_lib.initialize_moveit_commander()
        
        # # Begin sequence of arm and gripper manipulations
        # rospy.loginfo("Moving the arm to the up position...")
        # action_lib.reach_to("mug_pregrasp")
        
        # rospy.loginfo("Moving the arm to the ready position 0")
        # action_lib.move_to_grasp("mug")
        # action_lib.close_gripper()


        # action_lib.reach_to("mug_liftup")
        
        # action_lib.reach_to("sink")
        
        action_lib.open_gripper()

        # # Insert additional action_lib function calls as needed
        ########################################
        action_lib.reach_to("tap")
        action_lib.move_to_grasp("tap")
        # action_lib.close_gripper()
        # action_lib.switch_on_tap()
        # action_lib.open_gripper()



        
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return

if __name__ == '__main__':
    main()