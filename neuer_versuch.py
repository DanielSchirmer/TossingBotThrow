#!/usr/bin/env python

import rospy
import actionlib
from franka_gripper.msg import GraspAction, GraspGoal

def control_gripper_force_and_width(width, force, width_tolerance):
    rospy.init_node('gripper_force_and_width_controller')  # Initialisiere den ROS-Knoten
    client = actionlib.SimpleActionClient('/franka_gripper/grasp', GraspAction)  # Erstelle den Action-Client
    rospy.loginfo("Waiting for franka gripper grasp action server...")
    client.wait_for_server()  # Warte auf den Action-Server
    rospy.loginfo("franka gripper grasp action server found.")

    # Erstelle und sende das Ziel an den Action-Server
    goal = GraspGoal()
    goal.width = width
    goal.force = force
    goal.speed = 0.1  # Setze die Geschwindigkeit auf das Maximum
    goal.epsilon.inner = width_tolerance  # Setze die innere Breitentoleranz
    goal.epsilon.outer = width_tolerance  # Setze die äußere Breitentoleranz

    rospy.loginfo("Sending goal to franka gripper grasp action server...")
    client.send_goal(goal)  # Sende das Ziel an den Action-Server

    # Warte auf das Ergebnis
    client.wait_for_result()
    result = client.get_result()

    print(result)

    # Logge das Ergebnis
    if result:
        rospy.loginfo("Result: Success - %s", result.success)
    else:
        rospy.logwarn("No result received.")

if __name__ == "__main__":
    # Definiere die Breite, die Kraft und die Breitentoleranz in der main
    gripper_width = 0.06  # Beispielwert für die Breite des Greifers
    gripper_force = 5  # Beispielwert für die Kraft des Greifers
    gripper_width_tolerance = 0.001  # Beispielwert für die Breitentoleranz
    # Rufe die Funktion auf und übergebe die definierten Parameter
    control_gripper_force_and_width(gripper_width, gripper_force, gripper_width_tolerance)
