#!/usr/bin/env python3
#
# MOBILE ROBOTS - FI-UNAM, 2024-2
# THE PLATFORM ROS 
#
# Instructions:
# Write a program to move the robot forwards until the laser
# detects an obstacle in front of it.
# Required publishers and subscribers are already declared and initialized.
#

import rospy
from sensor_msgs.msg   import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray

NAME = "EMS"

def callback_scan(msg):
    global obstacle_detected
    #
    # TODO:
    # Do something to detect if there is an obstacle in front of the robot.
    # Set the 'obstacle_detected' variable with True or False, accordingly.
    #
    n = int((msg.angle_max - msg.angle_min)/msg.angle_increment/2)
    obstacle_detected = msg.ranges [n] < 1.5 #modified
    return

#se agrega otra funcion
def arm_move(msg):
    rospy.loginfo("Datos recibidos izq: %s", msg.data)
    return 
#prueba de brazo derecho
def arm_move_der(msg):
    rospy.loginfo("Datos recibidos der: %s", msg.data)
    return
def move_hand(msg):
    rospy.loginfo("Datos de cabeza: %s", msg.data)
    return

def main():
    print("ROS BASICS - " + NAME)
    rospy.init_node("ros_basics")
    rospy.Subscriber("/hardware/scan", LaserScan, callback_scan)
    pub_cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

    #Aggre new subscriber for pose
    rospy.Subscriber("/hardware/left_arm/goal_pose", Float64MultiArray, arm_move)
    pub_larm_pose = rospy.Publisher("/hardware/left_arm/goal_pose", Float64MultiArray, queue_size=10)
    loop = rospy.Rate(10)
    
    #Se agrega otro suscriptor para la pose del brazo der
    rospy.Subscriber("/hardware/right_arm/goal_pose", Float64MultiArray, arm_move_der)
    pub_larm_pose_der = rospy.Publisher("/hardware/right_arm/goal_pose", Float64MultiArray, queue_size=10)
    loop_der = rospy.Rate(10)

    #Aggre new subscriber for head goal_pose
    rospy.Subscriber("/hardware/head/goal_pose", Float64MultiArray, move_hand)
    pub_head_pose = rospy.Publisher("/hardware/head/goal_pose", Float64MultiArray, queue_size=10)
    loop_head = rospy.Rate(10)


    global obstacle_detected
    obstacle_detected = False
    while not rospy.is_shutdown():
        #
        # TODO:
        # Declare a Twist message and assign the appropiate speeds:
        # Move forward if there is no obstacle in front of the robot, and stop otherwise.
        # Use the 'obstacle_detected' variable to check if there is an obstacle. 
        # Publish the Twist message using the already declared publisher 'pub_cmd_vel'.
        msg_cmd_vel = Twist()
        #Aggre new data in msg the pose
        msg_la_pose = Float64MultiArray()
        msg_la_pose.data = [0.0,0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        pub_larm_pose.publish(msg_la_pose) #Publish the menssage

        #se agrega el de der
        msg_la_pose_der = Float64MultiArray()
        msg_la_pose_der.data = [0.0,0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        pub_larm_pose_der.publish(msg_la_pose_der)

        #Aggre new data for head
        msg_head_pose = Float64MultiArray()
        msg_head_pose.data = [0.0, 0.0]
        pub_head_pose.publish(msg_head_pose)

        msg_cmd_vel.linear.x = 0 if obstacle_detected else 0.3
        pub_cmd_vel.publish(msg_cmd_vel)
        
        if msg_cmd_vel.linear.x == 0:
            #msg_la_pose.data = [1.0416, -0.0001, 0.5003, -0.5000, 1.0999, -0.2880, 0.0001]
            msg_la_pose.data = [1.8, -0.005, 0.8, -0.9000, 2.0999, -0.2880, 0.0007]
            pub_larm_pose.publish(msg_la_pose)

            #Derecho
            msg_la_pose_der.data = [2.3, -0.69, 1.2, -1, 2.8, -2, 0.0063]
            pub_larm_pose_der.publish(msg_la_pose_der)

            #head
            msg_head_pose.data = [2.5, 1.0]
            pub_head_pose.publish(msg_head_pose)
        loop.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    
