#!/usr/bin/env python3
import rospy

class Node:

    def __init__(self):
        rospy.init_node('action_library_node')

    def spin(Self):
        rospy.spin()

def main():
    Node().spin()

if __name__ == '__main__':
    main()