import rospy
import asyncio
from move_group_python_interface_tutorial import MoveGroupPythonInterfaceTutorial
import actionlib
from franka_gripper.msg import GraspAction, GraspGoal, MoveAction, MoveGoal

# Funktion zum Öffnen des Greifers
def control_gripper_force_and_width(width, force, width_tolerance):
    # Erstelle und sende das Ziel an den Action-Server
    goal = GraspGoal()

    if force == 0 and width_tolerance == 0.0:
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

async def main():
    try:
        # Initialisiere den ROS-Knoten
        rospy.init_node('gripper_force_and_width_controller')

        # Erstellen eines MoveGroup-Objektes
        robot = MoveGroupPythonInterfaceTutorial()

        # Starten der Funktionen gleichzeitig
        loop = asyncio.get_event_loop()
        task1 = asyncio.create_task(robot.go_to_joint_state(0, 0, 0, -4, 0, 135, 45))
        task2 = loop.run_in_executor(None, control_gripper_force_and_width, 0.08, 0, 0)  # Beispielwerte für width, force und width_tolerance

        # Warten auf das Ende der Bewegungsfunktion
        await asyncio.gather(task1, task2)
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    asyncio.run(main())
