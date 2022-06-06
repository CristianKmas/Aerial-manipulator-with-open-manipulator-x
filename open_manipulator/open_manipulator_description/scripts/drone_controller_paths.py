#!/usr/bin/env python3

import rospy, math
from nav_msgs.msg import Path #Message type to subscribe to /odom and to publish in /path topics
from geometry_msgs.msg import PointStamped, QuaternionStamped
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
x=0; y=0; z=0; roll=0; pitch=0; yaw=0;

drone_path_msg = Path()

#Create the title of the global frame. Global frame is required by Rviz to define the SAME frame for any robot
global_frame = "body_FLU"

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

def poseCallback(msg.point, msg.quaternion): #Callback function to get the drone posture
        global x, y, z
        x = msg.point.x
        y = msg.point.y
        z = msg.point.z
        
        quater = msg.quaternion
        quater_list = [quater.x, quater.y, quater.z, quater.w]
        (roll, pitch, yaw) = euler_from_quaternion(quater_list)
        
        drone_pose = PoseStamped() #Create the PoseStamped instance
        drone_pose.pose.position = msg.point
        drone_path_msg.poses.append(drone_pose) #Add the drone position to the list of points
        

def main_function():
	rospy.init_node("drone_controller_paths", anonymous=True) #Initialize the node. anonymous=True for multiple nodes
	global rate
	rate = rospy.Rate(50) #Node frequency (Hz)
	counter = 0
	
	rospy.Subscriber('/dji_sdk/local_position',PointStamped, poseCallback) #To subscribe to the topic
	rospy.Subscriber('/dji_sdk/attitude',QuaternionStamped, poseCallback) #To subscribe to the topic	
	#To publish the paths
	drone_path_pub = rospy.Publisher('/drone_path', Path, queue_size=10)
	
	global drone_path_msg
	#Important: Assignation of the SAME reference frame for ALL robots
	drone_path_msg.header.frame_id = tr_path_msg.header.frame_id = global_frame

	enable_motors() #Execute the functions
	takeoff()
	
	print("Press 'q' to finish the node\n")
	print("To clean paths, press 'c' key\n")
	rospy.logwarn("To start the movement, the simulation must be running\n\n") #Warning message
	
	global t, t0 #t and t0 are global to be used in velocity_controller()
	t0 = rospy.Time.now().to_sec()
	
	while(1):
	
		#Publish the "path" messages
		drone_path_pub.publish(drone_path_msg)
		
		rate.sleep() #spinOnce() function does not exist in python
		key = getKey()
		if(key == 'c'): #Clear paths lists
			del drone_path_msg.poses[:]
			rospy.logwarn("Clear paths\n")
			key = 0; #Reset the key
		elif(key == 'q'):
			break

	land() #Execute land function
	print("\n Node finished\n")


if __name__ == '__main__':
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    try:
    	main_function()  #Execute the function
	
    except rospy.ROSInterruptException:
        pass

    #if os.name != 'nt':
    #   termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
