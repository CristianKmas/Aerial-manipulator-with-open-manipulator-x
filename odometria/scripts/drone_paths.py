#!/usr/bin/env python3

#Use 'python' for ROS Kinetic and Melodic.
#Use 'python3' for ROS Noetic
import rospy, math, tf
from nav_msgs.msg import Odometry, Path #Message type to subscribe to /odom and to publish in /path topics
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from math import sin, cos, pi
from geometry_msgs.msg import PoseStamped, QuaternionStamped, PointStamped, Point, Quaternion, Pose #Message type to publish the robots' path

#kbhit function implemented on Linux
import sys, select, os #Handling command line arguments
if os.name == 'nt':
  import msvcrt
else:
  import tty, termios

#Define global variables (position and orientation, using Euler anlges)
x=0; y=0; z=0; 
drone_path_msg = Path()

odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
odom_broadcaster = tf.TransformBroadcaster()

#Create the title of the global frame. Global frame is required by Rviz to define the SAME frame for any robot
global_frame = "world"

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
        global x, y, z
        x = msg.point.x
        y = msg.point.y
        z = msg.point.z
                
        drone_pose = PoseStamped() #Create the PoseStamped instance
        drone_pose.pose.position = msg.point
        drone_path_msg.poses.append(drone_pose) #Add the drone position to the list of points

        
def attiCallback(msg):   
	global quater_list     
        #Operations to convert from quaternion to Euler angles
	quater = msg.quaternion
	quater_list = [quater.x, quater.y, quater.z, quater.w]
	(roll, pitch, yaw) = euler_from_quaternion(quater_list)
	
	current_time = rospy.Time.now()
	last_time = rospy.Time.now()
	odom_broadcaster.sendTransform(
	(x, y, z),
	quater_list,
	current_time,
	"body_FLU",
	"world"
	)

	# next, we'll publish the odometry message over ROS
	odom = Odometry()
	odom.header.stamp = current_time
	odom.header.frame_id = "odom"
	# set the position
	odom.pose.pose = Pose(Point(x, y, z), Quaternion(*quater_list))

	# publish the message
	odom_pub.publish(odom)
	last_time = current_time

def main_function():
	rospy.init_node("drone_paths", anonymous=True) #Initialize the node. anonymous=True for multiple nodes
	global rate
	rate = rospy.Rate(50) #Node frequency (Hz)
	counter = 0

	rospy.Subscriber('/dji_sdk/local_position',PointStamped, poseCallback) #To subscribe to the topic
	rospy.Subscriber('/dji_sdk/attitude',QuaternionStamped, attiCallback)
	#To publish the paths
	drone_path_pub = rospy.Publisher('/drone_path', Path, queue_size=10) 

	global drone_path_msg
	#Important: Assignation of the SAME reference frame for ALL robots
	drone_path_msg.header.frame_id = global_frame
	
	print("Press 'q' to finish the node\n")
	print("To clean path, press 'c' key\n")
	rospy.logwarn("To start the movement, the simulation must be running\n\n") #Warning message
	# first, we'll publish the transform over tf

	
	while(1):
		
		#Publish the "path" messages
		drone_path_pub.publish(drone_path_msg)
		
		rate.sleep() #spinOnce() function does not exist in python
		key = getKey()
		if(key == 'c'): #Clear paths lists
			del drone_path_msg.poses[:]
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

