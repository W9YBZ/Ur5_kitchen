import rospy

from robotiq_2f_gripper_control.msg import Robotiq2FGripper_robot_output

class GripperInterface:

  def __init__(self):
    self.pub = rospy.Publisher(
      'Robotiq2FGripperRobotOutput',
      Robotiq2FGripper_robot_output,
      queue_size=10,
    )

  def action(self, char):

    command = Robotiq2FGripper_robot_output()
    command.rACT = 1
    command.rGTO = 1

    if char == 'c':
        command.rSP  = 255
        command.rFR  = 1
        command.rPR = 255

    if char == 'o':
        command.rSP  = 255
        command.rFR  = 10
        command.rPR = 0

    self.pub.publish(command)

    print('i am here')

gripper = GripperInterface()