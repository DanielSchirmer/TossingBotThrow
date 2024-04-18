import rospy
from move_group_python_interface_tutorial import MoveGroupPythonInterfaceTutorial
import actionlib
from franka_gripper.msg import GraspAction, GraspGoal, MoveAction, MoveGoal
import asyncio

# Greifersteuerung mit Force und Breite
async def control_gripper_force_and_width(width, force, width_tolerance):
    # Erstellen und Senden des Ziels an den Action-Server
    goal = GraspGoal()

    if force == 0 and width_tolerance == 0.0:
        client = actionlib.SimpleActionClient('/franka_gripper/move', MoveAction)
        client.wait_for_server()

        goal = MoveGoal(width=width, speed=0.1)
        client.send_goal(goal)
        client.wait_for_result()

        return client.get_result()
    else:
        client = actionlib.SimpleActionClient('/franka_gripper/grasp', GraspAction)
        rospy.loginfo("Waiting for franka gripper grasp action server...")
        client.wait_for_server()
        rospy.loginfo("franka gripper grasp action server found.")
        goal.width = width
        goal.speed = 0.1
        goal.force = force
        goal.epsilon.inner = width_tolerance
        goal.epsilon.outer = width_tolerance

    rospy.loginfo("Sending goal to franka gripper grasp action server...")
    client.send_goal(goal)
    client.wait_for_result()
    result = client.get_result()

    print(result)

    if result:
        rospy.loginfo("Result: Success - %s", result.success)
    else:
        rospy.logwarn("No result received.")

async def main():
    try:
        rospy.init_node('gripper_force_and_width_controller')

        # Erstellen eines MoveGrouo-Objektes von Tutorial
        robot = MoveGroupPythonInterfaceTutorial()

        # Starten der Bewegungen des Roboters und des Greifers gleichzeitig
        tasks = [
            robot.go_to_joint_state(0, -10, 0, -150, 0, 130, 45),
            control_gripper_force_and_width(0.075, 0, 0),
            robot.go_to_joint_state(0, 19, 0, -153, 0, 171, 45),
            control_gripper_force_and_width(0.06, 10, 0.001),
            robot.go_to_joint_state(0, -10, 0, -150, 0, 130, 45),
            robot.go_to_joint_state(0, -100, 0, -175, 0, 90, 45)
        ]

        # Führen Sie die ersten Aufgaben aus
        await asyncio.gather(*tasks)

        # Führen Sie die letzten beiden Aufgaben parallel aus
        loop = asyncio.get_running_loop()
        task1 = loop.run_in_executor(None, robot.go_to_joint_state, 0, 44, 0, -4, 0, 135, 45)
        task2 = loop.run_in_executor(None, control_gripper_force_and_width, 0.08, 0, 0)  # Beispielwerte für width, force und width_tolerance

        await asyncio.gather(task1, task2)

    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    asyncio.run(main())
