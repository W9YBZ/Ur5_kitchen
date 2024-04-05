#!/usr/bin/env python

import rospy
from gazebo_msgs.msg import ModelStates

def model_states_callback(msg):
    # This function is called every time a new message is received on /gazebo/model_states
    print("Current positions of models in Gazebo:")
    for i, name in enumerate(msg.name):
        position = msg.pose[i].position
        print(f"  Model: {name}")
        print(f"    Position - x: {position.x}, y: {position.y}, z: {position.z}")
    print("-" * 40)

def listener():
    rospy.init_node('gazebo_model_states_listener', anonymous=True)

    # Subscribe to the /gazebo/model_states topic
    rospy.Subscriber("/gazebo/model_states", ModelStates, model_states_callback)

    # Keep the node alive
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
