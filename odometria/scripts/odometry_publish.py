#!/usr/bin/env python3
import math
from math import sin, cos, pi

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

rospy.init_node('odometry_publish')

odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
odom_broadcaster = tf.TransformBroadcaster()

x = 0.0
y = 0.0
z = 0.0
th = 0.0

vx = 0.1
vy = -0.1
vz = 0.1
vth = 0.1

current_time = rospy.Time.now()
last_time = rospy.Time.now()

r = rospy.Rate(1.0)
while not rospy.is_shutdown():
    current_time = rospy.Time.now()

    # compute odometry in a typical way given the velocities of the robot
    dt = (current_time - last_time).to_sec()
    delta_x = (vx * cos(th) - vy * sin(th) + vz*sin(th)) * dt
    delta_y = (vx * sin(th) + vy * cos(th) + vz*sin(th)) * dt
    delta_z = (vx * sin(th) + vy * cos(th) + vz*sin(th)) * dt
    delta_th = vth * dt

    x += delta_x
    y += delta_y
    z += delta_z
    th += delta_th

    # since all odometry is 6DOF we'll need a quaternion created from yaw
    odom_quat = tf.transformations.quaternion_from_euler(0, 0, th)

    # first, we'll publish the transform over tf
    odom_broadcaster.sendTransform(
        (x, y, z),
        odom_quat,
        current_time,
        "body_FLU",
        "world"
    )

    # next, we'll publish the odometry message over ROS
    odom = Odometry()
    odom.header.stamp = current_time
    odom.header.frame_id = "world"

    # set the position
    odom.pose.pose = Pose(Point(x, y, z), Quaternion(*odom_quat))

    # set the velocity
    odom.child_frame_id = "body_FLU"
    odom.twist.twist = Twist(Vector3(vx, vy, vz), Vector3(0, 0, vth))

    # publish the message
    odom_pub.publish(odom)

    last_time = current_time
    r.sleep()
