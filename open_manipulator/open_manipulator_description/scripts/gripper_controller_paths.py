#!/usr/bin/env python3

#Use 'python' for ROS Kinetic and Melodic.
#Use 'python3' for ROS Noetic
import rospy, math
from nav_msgs.msg import Odometry, Path #Message type to subscribe to /odom and to publish in /path topics
from open_manipulator_msgs.msg import KinematicsPose 
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from math import sin, cos, pi
from geometry_msgs.msg import PoseStamped #Message type to publish the robots' path

#kbhit function implemented on Linux
import sys, select, os #Handling command line arguments
if os.name == 'nt':
  import msvcrt
else:
  import tty, termios

#Define global variables (position and orientation, using Euler anlges)
x=0; y=0; z=0; yaw=0
gripper_path_msg = Path()

#Create the title of the global frame. Global frame is required by Rviz to define the SAME frame for any robot
global_frame = "end_effector_link"

def getKey(): #Function to use keyboard events on Linux
    if os.name == 'nt':
      return msvcrt.getch()

    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def poseCallback(msg): #Callback function to get the drone posture
        global x, y, z, yaw
        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z
        #Operations to convert from quaternion to Euler angles
        quater = msg.pose.orientation
        quater_list = [quater.x, quater.y, quater.z, quater.w]
        (roll, pitch, yaw) = euler_from_quaternion(quater_list)
        
        gripper_pose = PoseStamped() #Create the PoseStamped instance
        gripper_pose.pose.position = msg.pose.position
        gripper_path_msg.poses.append(gripper_pose) #Add the gripper position to the list of points

def main_function():
	rospy.init_node("gripper_controller_paths", anonymous=True) #Initialize the node. anonymous=True for multiple nodes
	global rate
	rate = rospy.Rate(50) #Node frequency (Hz)
	counter = 0

	rospy.Subscriber('/gripper/kinematics_pose',KinematicsPose, poseCallback) #To subscribe to the topic
	
	#To publish the paths
	gripper_path_pub = rospy.Publisher('/gripper_path', Path, queue_size=10) 
	
	global gripper_path_msg
	#Important: Assignation of the SAME reference frame for ALL robots
	gripper_path_msg.header.frame_id = global_frame
	
	print("Press 'q' to finish the node\n")
	print("To clean path, press 'c' key\n")
	rospy.logwarn("To start the movement, the simulation must be running\n\n") #Warning message
	
	
	while(1):
		
		#Publish the "path" messages
		gripper_path_pub.publish(gripper_path_msg)
		
		rate.sleep() #spinOnce() function does not exist in python
		key = getKey()
		if(key == 'c'): #Clear paths lists
			del gripper_path_msg.poses[:]
			rospy.logwarn("Clear path\n")
			key = 0; #Reset the key
		elif(key == 'q'):
			break

	print("\n Node finished\n")


if __name__ == '__main__':
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    try:
    	main_function()  #Execute the function
	
    except rospy.ROSInterruptException:
        pass

