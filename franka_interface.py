import rospy
from gazebo_msgs.srv import SpawnModel, DeleteModel
from geometry_msgs.msg import Pose
from std_msgs.msg import String

from move_group_python_interface_tutorial import MoveGroupPythonInterfaceTutorial

import actionlib
from franka_gripper.msg import MoveGoal, MoveAction, GraspAction, GraspGoal

# Gripper ohne Force
def control_gripper(width):
    client = actionlib.SimpleActionClient('/franka_gripper/move', MoveAction)
    client.wait_for_server()

    goal = MoveGoal(width=width, speed=0.1)
    client.send_goal(goal)
    client.wait_for_result()

    return client.get_result()


# Gripper mit Force
def control_gripper_force_and_width(width, force, width_tolerance):
    client = actionlib.SimpleActionClient('/franka_gripper/grasp', GraspAction)
    client.wait_for_server()

    goal = GraspGoal()
    goal.width = width
    goal.force = force
    goal.speed = 0.1
    goal.epsilon.inner = width_tolerance
    goal.epsilon.outer = width_tolerance

    client.send_goal(goal)
    client.wait_for_result()
    result = client.get_result()

    print(result)

    if result:
        rospy.loginfo("Result: Success - %s", result.success)
    else:
        rospy.logwarn("No result received.")

if __name__ == '__main__':
    # Initialisierung des ROS-Knotens
    rospy.init_node('panda_gripper_controller')

    try:
        # Erstellen eines MoveGrouo-Objektes von Tutorial
        robot = MoveGroupPythonInterfaceTutorial()

        # Joint State mit den Winkeln
        robot.go_to_joint_state(0, -10, 0, -150, 0, 130, 45)

        # Öffnen des Greifers neu
        control_gripper_force_and_width(0.08, 0, 0)

        # Neue Winkel über Objekt
        robot.go_to_joint_state(0, 19, 0, -153, 0, 171, 45)

        # Schließen des Greifers mit Force
        control_gripper_force_and_width(0.06, 20, 0.001)

        # Bewegen des Roboters nach oben
        robot.go_to_joint_state(0, -10, 0, -150, 0, 130, 45)

        # Öffnen des Greifers neu
        control_gripper_force_and_width(0.08, 0, 0)

    except rospy.ROSInterruptException:
        pass

    #def spawn_object():
    # Initialize ROS node
#    rospy.init_node('spawn_object_node')

    
    # Wait for gazebo service to become available
#    rospy.wait_for_service('/gazebo/spawn_sdf_model')
#    rospy.wait_for_service('/gazebo/delete_model')

    # Service proxy for spawning and deleting models in Gazebo
#    spawn_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
#    delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)

    # Load object SDF file
#    with open('/home/student/panda_ws/src/panda-gazebo/panda_gazebo/resources/models/cube/model.sdf', 'r') as f:
#        object_sdf = f.read()

    # Define pose for the object
#    object_pose = Pose()
#    object_pose.position.x = 0.5
#    object_pose.position.y = 0
#    object_pose.position.z = 0

    # Spawn object in Gazebo
#    object_name = "cube"
#    spawn_model(object_name, object_sdf, "", object_pose, "world")


#def attach_object_to_robot(object_name, link_name):
    # Initialize ROS node
#    rospy.init_node('attach_object_node')

    # Wait for gazebo service to become available
#    rospy.wait_for_service('/gazebo/attach_model')

    # Service proxy for attaching models in Gazebo
#    attach_model = rospy.ServiceProxy('/gazebo/attach_model', AttachModel)

    # Attach object to the robot
#    try:
#        attach_model(object_name, link_name)
#        rospy.loginfo("Object {} attached to link {}".format(object_name, link_name))
#    except rospy.ServiceException as e:
#        rospy.logerr("Failed to attach object to link: {}".format(e))
