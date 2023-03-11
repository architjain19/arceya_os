#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from cmd_vel_controller.msg import Diff_Drive_Vel


robot_namespace = "/diff_drive"
cmd_vel_topic = "/cmd_vel"

wheel_sep_length = 0.0      # wheel seperation length/2
wheel_radius = 0.0          # wheel diameter/2

diff_drive_vel_pub = None


def get_params():
    
    global wheel_sep_length, wheel_radius
    
    wheel_sep_length = rospy.get_param('/dd_controller/wheel_sep_length', default=0.220)
    wheel_radius = rospy.get_param('/dd_controller/wheel_radius', default=0.0639)
    
    rospy.loginfo("[DD_IK_Controller]: Received launch parameters")
    rospy.loginfo(f"[DD_IK_Controller]: Setting these parameters: [wheel_sep_length]: {wheel_sep_length}\t[wheel_radius]: {wheel_radius}")
    

def dd_vel_publisher(wheel_front_left, wheel_front_right, wheel_rear_left, wheel_rear_right):
    dd_msg = Diff_Drive_Vel()
    
    dd_msg.front_left_vel = wheel_front_left
    dd_msg.front_right_vel = wheel_front_right
    dd_msg.rear_left_vel = wheel_rear_left
    dd_msg.rear_right_vel = wheel_rear_right
    
    diff_drive_vel_pub.publish(dd_msg)

    rospy.loginfo("[DD_IK_Controller]: Diff_Drive Individual velocities published")


def dd_inverse_kinematics(wheel_radius, wheel_sep_length, dd_vel_x, dd_vel_y, dd_vel_z):

    """
    Description:    Inverse kinematics calculation to calculate individual wheel velocities of a four wheel diiferential drive.
    """

    wheel_front_left = (1/wheel_radius)*(dd_vel_x + dd_vel_z * wheel_sep_length)
    wheel_front_right = (1/wheel_radius)*(dd_vel_x - dd_vel_z * wheel_sep_length)
    wheel_rear_left = (1/wheel_radius)*(dd_vel_x + dd_vel_z * wheel_sep_length)
    wheel_rear_right = (1/wheel_radius)*(dd_vel_x - dd_vel_z * wheel_sep_length)

    rospy.loginfo(f"[DD_IK_Controller]: Received a new Twist message: [linear_x]: {dd_vel_x}\t[linear_y]: {dd_vel_y}\t[angular_z]: {dd_vel_z}")
    rospy.loginfo(f"[DD_IK_Controller]: Calculated individual wheel velocities: [Front Left]: {wheel_front_left}\t[Front Right]: {wheel_front_right}\t[Rear Left]: {wheel_rear_left}\t[Rear Right]: {wheel_rear_right}")

    dd_vel_publisher(wheel_front_left, wheel_front_right, wheel_rear_left, wheel_rear_right)

def cmd_vel_callback(data):

    """
    Description:    Callback function for subscriber of topic named `/diff_drive/cmd_vel`.
                    Also performing inverse kinematics on cmd_vel twist messages for differential drive.
    """

    dd_vel_x = data.linear.x
    dd_vel_y = data.linear.y
    dd_vel_z = data.angular.z

    # rospy.loginfo(f"[DD_IK_Controller]: Received a new Twist message: linear_x: {dd_vel_x}\tlinear_y: {dd_vel_y}\tangular_z: {dd_vel_z}")

    dd_inverse_kinematics(wheel_radius, wheel_sep_length, dd_vel_x, dd_vel_y, dd_vel_z)

    rospy.loginfo("[DD_IK_Controller]: ------------------------------------------")


def main():

    """
    Description:    This node subscribes to `/diff_drive/cmd_vel` topic and 
                    return inidvidual wheel velocities via inverse kinematics calculations.
    Suggestions:    `anonymous=True` while running same node multiple times at once
    """

    rospy.init_node('diff_drive_ik_controller')
    rospy.loginfo("[DD_IK_Controller]: Differential_drive_controller node has started")

    get_params()

    global diff_drive_vel_pub
    diff_drive_vel_pub = rospy.Publisher("/dd_controller/diff_drive_velocities", Diff_Drive_Vel, queue_size=10)

    sub_topic_name = robot_namespace + cmd_vel_topic
    rospy.Subscriber(sub_topic_name, Twist, cmd_vel_callback)


    try:
        rospy.spin()
    except KeyboardInterrupt:
        try:
            pass
        except Exception as e:
            pass
        print("[DD_IK_Controller] Shutting down the node")