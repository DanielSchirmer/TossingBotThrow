import franka_gripper.msg
import rospy
import sys

# Brings in the SimpleActionClient
import actionlib

from move_group_python_interface_tutorial import MoveGroupPythonInterfaceTutorial


# Brings in the messages used by the action, including the
# goal message and the result message.

def move_gripper():
    
    # Creates the SimpleActionClient, passing the type of the action
    client = actionlib.SimpleActionClient('/franka_gripper/move', franka_gripper.msg.MoveAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    # Creates a goal to send to the action server.
    goal = franka_gripper.msg.MoveGoal(width=0.06, speed=0.1)
    #goal.width = 0.022
    #goal.speed = 1.0
    
    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()  # A move result


if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('junaid_gripper')
        result = move_gripper()

        print("Success: ",result.success)
        print("Error message: ", result.error)
    except rospy.ROSInterruptException:
        print("program interrupted before completion")