import rospy
from gazebo_msgs.srv import SpawnModel, DeleteModel
from geometry_msgs.msg import Pose
from std_msgs.msg import String

from move_group_python_interface_tutorial import MoveGroupPythonInterfaceTutorial
import rospy
import actionlib
from franka_gripper.msg import GraspAction, GraspGoal

import actionlib
from franka_gripper.msg import MoveGoal, MoveAction, GraspAction, GraspGoal

import asyncio

# Gripper mit Force
async def control_gripper_force_and_width(width, force, width_tolerance):
    # ist in der main definiert
    #rospy.init_node('gripper_force_and_width_controller')  # Initialisiere den ROS-Knoten

    # Erstelle und sende das Ziel an den Action-Server
    goal = GraspGoal()

    if((force == 0) and (width_tolerance == 0.0)):
        client = actionlib.SimpleActionClient('/franka_gripper/move', MoveAction)
        client.wait_for_server()

        goal = MoveGoal(width=width, speed=0.1)
        client.send_goal(goal)
        client.wait_for_result()

        return client.get_result()


    else:
        client = actionlib.SimpleActionClient('/franka_gripper/grasp', GraspAction)  # Erstelle den Action-Client
        rospy.loginfo("Waiting for franka gripper grasp action server...")
        client.wait_for_server()  # Warte auf den Action-Server
        rospy.loginfo("franka gripper grasp action server found.")
        goal.width = width
        goal.speed = 0.1  # Setze die Geschwindigkeit auf das Maximum
        goal.force = force
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



async def main():
    try:
        
        rospy.init_node('gripper_force_and_width_controller')  # Initialisiere den ROS-Knoten

        # Erstellen eines MoveGrouo-Objektes von Tutorial
        robot = MoveGroupPythonInterfaceTutorial()


        # Joint State mit den Winkeln
        robot.go_to_joint_state(0, -10, 0, -150, 0, 130, 45)

        # Öffnen des Greifers neu
        control_gripper_force_and_width(0.075, 0, 0)


        # Alte Winkel
        #robot.go_to_joint_state(0, 10, 0, -160, 0, 170, 45)


        # Neue Winkel über Objekt
        robot.go_to_joint_state(0, 19, 0, -153, 0, 171, 45)


        # Schließen des Greifers mit Force
        control_gripper_force_and_width(0.06, 10, 0.001)

        
        # Bewegen des Roboters nach oben
        robot.go_to_joint_state(0, -10, 0, -150, 0, 130, 45)


        # Ausholen
        robot.go_to_joint_state(0, -100, 0, -175, 0, 90, 45)


        # Starten der Funktionen gleichzeitig
        loop = asyncio.get_event_loop()
        task1 = asyncio.create_task(robot.go_to_joint_state(0, 44, 0, -4, 0, 135, 45))
        task2 = loop.run_in_executor(None, control_gripper_force_and_width, 0.08, 0, 0)  # Beispielwerte für width, force und width_tolerance

        # Warten auf das Ende der Bewegungsfunktion
        await asyncio.gather(task1, task2)
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    asyncio.run(main())