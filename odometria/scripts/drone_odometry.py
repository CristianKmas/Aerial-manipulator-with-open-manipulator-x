#!/usr/bin/env python3

import math
import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PointStamped, Pose, QuaternionStamped, Quaternion, Point

x = 0.0
y = 0.0
z = 0.0
quater_list = [0, 0, 0, 0]
    # compute odometry in a typical way given the velocities of the robot
def localCallback(msg):
	global x, y, z
	x = msg.point.x
	y = msg.point.y
	z = msg.point.z

def attiCallback(msg):
	global quarter_list

	quater = msg.quaternion
	quater_list = [quater.x, quater.y, quater.z, quater.w]
	(roll, pitch, yaw) = euler_from_quaternion(quater_list)
	


rospy.init_node('drone_odometry')
odom_pub = rospy.Publisher('odom', Odometry, queue_size=50)
odom_broadcaster = tf.TransformBroadcaster()
rospy.Subscriber('/dji_sdk/local_position', PointStamped, 10, localCallback)
rospy.Subscriber('/dji_sdk/attitude', QuaternionStamped, 10, attiCallback)


current_time = rospy.Time.now()
last_time = rospy.Time.now()

r = rospy.Rate(50)
while not rospy.is_shutdown():
    current_time = rospy.Time.now()



# first, we'll publish the transform over tf
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
r.sleep()
