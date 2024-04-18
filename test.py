import rospy
from franka_gripper.msg import MoveGoal, MoveAction
import actionlib

#def control_gripper(width):
 #   client = actionlib.SimpleActionClient('/franka_gripper/move', MoveAction)
  #  client.wait_for_server()

   # goal = MoveGoal(width=width, speed=0.1)  # Hier können Sie die Geschwindigkeit anpassen
    #client.send_goal(goal)
  #  client.wait_for_result()

   # return client.get_result()

#if __name__ == '__main__':
 #   try:
  #      rospy.init_node('panda_gripper_control')

        # Öffnen des Greifers (Breite = 0.08)
   #     result = control_gripper(0.08)
    #    rospy.loginfo("Gripper width set to: {}".format(result))

     #   rospy.sleep(1)  # Warten, um den Effekt zu sehen

        # Schließen des Greifers (Breite = 0.0)
      #  result = control_gripper(0.0)
       # rospy.loginfo("Gripper width set to: {}".format(result))

        #rospy.sleep(1)  # Warten, um den Effekt zu sehen

#    except rospy.ROSInterruptException:
 #       pass#

import rospy
import actionlib
from franka_gripper.msg import GraspAction, GraspGoal

#def control_panda_gripper(width, speed, force):
    # Initialize the action client for gripper control
 #   client = actionlib.SimpleActionClient('/franka_gripper/grasp', GraspAction)
  #  client.wait_for_server()

    # Set the gripper width, speed, and force
   # goal = GraspGoal(width=width, speed=speed, force=force)
    #client.send_goal(goal)
#    client.wait_for_result()

#if __name__ == '__main__':
 #   try:
  #      rospy.init_node('panda_gripper_control')

        # Open the gripper (Width = 0.08, Speed = 0.1, Force = 10)
   #     control_panda_gripper(0.08, 0.1, 10)
    #    rospy.loginfo("Gripper opened.")

     #   rospy.sleep(1)  # Wait to see the effect

        # Close the gripper (Width = 0.0, Speed = 0.1, Force = 10)
      #  control_panda_gripper(0.0, 0.1, 10)
       # rospy.loginfo("Gripper closed.")

        #rospy.sleep(1)  # Wait to see the effect

#    except rospy.ROSInterruptException:
 #       pass


#! /usr/bin/env python3
import franka_gripper.msg
import rospy
import sys

# Brings in the SimpleActionClient
import actionlib


# Brings in the messages used by the grasp action, including the
# goal message and the result message.

def grasp_client():
    # Creates the SimpleActionClient, passing the type of the action
    # (GraspAction) to the constructor.
    client = actionlib.SimpleActionClient('/franka_gripper/grasp', franka_gripper.msg.GraspAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    # Creates a goal to send to the action server.
    goal = franka_gripper.msg.GraspGoal()
    goal.width = 0.022
    goal.epsilon.inner = 0.005
    goal.epsilon.outer = 0.005
    goal.speed = 0.1
    goal.force = 5

    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()  # A GraspResult


if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('grasp_client_py')
        result = grasp_client()
        print("Success: ",result.success)
        print("Error message: ", result.error)
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)

