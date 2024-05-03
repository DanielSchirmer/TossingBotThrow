import rospy
from threading import Thread
from time import sleep
import actionlib
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from franka_gripper.msg import GraspAction, GraspGoal, MoveGoal, MoveAction
from move_group_python_interface_tutorial import MoveGroupPythonInterfaceTutorial

# Methode zum Spawnen eines Objekts
def spawnObject(model_name, pose, reference_frame="world"):
    rospy.wait_for_service('/gazebo/set_model_state')
    try:
        set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        state = ModelState()
        state.model_name = model_name
        state.pose.position.x = pose[0]
        state.pose.position.y = pose[1]
        state.pose.position.z = pose[2]
        state.reference_frame = reference_frame
        set_model_state(state)
        rospy.loginfo("Spawned object '{}' successfully.".format(model_name))
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: {}".format(e))

# Methode zur Steuerung des Greifers mit Kraft und Breite
def controlGripper(width, force, width_tolerance):
    # Erstelle den Action-Client
    if force == 0 and width_tolerance == 0.0:
        client = actionlib.SimpleActionClient('/franka_gripper/move', MoveAction)
        client.wait_for_server()
        goal = MoveGoal(width=width, speed=1.0)
    else:
        client = actionlib.SimpleActionClient('/franka_gripper/grasp', GraspAction)
        rospy.loginfo("Waiting for franka gripper grasp action server...")
        client.wait_for_server()
        rospy.loginfo("franka gripper grasp action server found.")
        goal = GraspGoal()
        goal.width = width
        goal.speed = 0.5
        goal.force = force
        goal.epsilon.inner = width_tolerance
        goal.epsilon.outer = width_tolerance

    # Sende das Ziel an den Action-Server
    rospy.loginfo("Sending goal to franka gripper grasp action server...")
    client.send_goal(goal)

    # Warte auf das Ergebnis
    client.wait_for_result()
    result = client.get_result()
    rospy.loginfo("Result: Success - %s", result.success if result else "No result received")

# Methode für die Wurfbewegung
def throwMovement(robot):
    robot.go_to_joint_state(-166, 50, 0, -4, 0, 170, -20)

# Methode für das Loslassen
def throwOpen():
    # Warten bis Loslassen
    sleep(1.4)
    controlGripper(0.08, 0, 0)

# Methode zur gleichzeitigen Ausführung
def throw(robot):
    thread1 = Thread(target=throwMovement, args=(robot,))
    thread2 = Thread(target=throwOpen)

    thread1.start()
    thread2.start()
    
    thread1.join()
    thread2.join()

# Hauptfunktion
def main():
    try:
        # Initialisiere den ROS-Knoten für den Greifer
        rospy.init_node('throwController')

        # Erstellen eines MoveGroup-Objekts für den Roboter
        robot = MoveGroupPythonInterfaceTutorial()

        # Bewegungen des Roboters und Steuerung des Greifers
        robot.go_to_joint_state(0, -10, 0, -150, 0, 130, 45)
        controlGripper(0.08, 0, 0)
        robot.go_to_joint_state(0, 19, 0, -153, 0, 171, 45)
        controlGripper(0.06, 10, 0.001)

        # Wurf
        throw(robot)

        # Warten bis spawn
        input()

        # Spawne Objekt
        model_name = "cube"
        
        
        pose = [0.4, 0, 0.4190880]
        spawnObject(model_name, pose)

    except rospy.ROSInterruptException:
        pass

# Hauptprogramm starten
if __name__ == "__main__":
    main()
