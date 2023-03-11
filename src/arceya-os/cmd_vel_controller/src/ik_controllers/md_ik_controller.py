#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

robot_namespace = "/mecanum_drive"
cmd_vel_topic = "/cmd_vel"


wheel_sep_width = 0.0       # wheel seperation width/2
wheel_sep_length = 0.0      # wheel seperation length/2
wheel_radius = 0.0          # wheel diameter/2


def get_params():
        
    """
    Description:    Function to get required ros params specified while launching this node.
    """

    global wheel_sep_width, wheel_sep_length, wheel_radius
    
    wheel_sep_width = rospy.get_param('/md_controller/wheel_sep_width', default=0.271)
    wheel_sep_length = rospy.get_param('/md_controller/wheel_sep_length', default=0.220)
    wheel_radius = rospy.get_param('/md_controller/wheel_radius', default=0.0639)
    
    rospy.loginfo("[MD_IK_Controller]: Received launch parameters")
    rospy.loginfo(f"[MD_IK_Controller]: Setting these parameters: [wheel_sep_width]: {wheel_sep_width}\t[wheel_sep_length]: {wheel_sep_length}\t[wheel_radius]: {wheel_radius}")
    

def md_inverse_kinematics(wheel_radius, wheel_sep_width, wheel_sep_length, md_vel_x, md_vel_y, md_vel_z):

    """
    Description:    Inverse kinematics calculation to calculate individual wheel velocities of a four wheel mecanum drive.
    """

    wheel_front_left = (1/wheel_radius)*(md_vel_x - md_vel_y - (wheel_sep_width + wheel_sep_length) * md_vel_z)
    wheel_front_right = (1/wheel_radius)*(md_vel_x + md_vel_y + (wheel_sep_width + wheel_sep_length) * md_vel_z)
    wheel_rear_left = (1/wheel_radius)*(md_vel_x + md_vel_y - (wheel_sep_width + wheel_sep_length) * md_vel_z)
    wheel_rear_right = (1/wheel_radius)*(md_vel_x - md_vel_y + (wheel_sep_width + wheel_sep_length) * md_vel_z)

    wheel_front_right = -1*wheel_front_right
    wheel_rear_right = -1*wheel_rear_right

    rospy.loginfo(f"[MD_IK_Controller]: Received a new Twist message: [linear_x]: {md_vel_x}\t[linear_y]: {md_vel_y}\t[angular_z]: {md_vel_z}")
    rospy.loginfo(f"[MD_IK_Controller]: Calculated individual wheel velocities: [Front Left]: {wheel_front_left}\t[Front Right]: {wheel_front_right}\t[Rear Left]: {wheel_rear_left}\t[Rear Right]: {wheel_rear_right}")


def cmd_vel_callback(data):

    """
    Description:    Callback function for subscriber of topic named `/mecanum_drive/cmd_vel`.
                    Also performing inverse kinematics on cmd_vel twist messages for mecanum drive.
    """

    md_vel_x = data.linear.x
    md_vel_y = data.linear.y
    md_vel_z = data.angular.z

    # rospy.loginfo(f"[MD_IK_Controller]: Received a new Twist message: linear_x: {md_vel_x}\tlinear_y: {md_vel_y}\tangular_z: {md_vel_z}")

    md_inverse_kinematics(wheel_radius, wheel_sep_width, wheel_sep_length, md_vel_x, md_vel_y, md_vel_z)

    rospy.loginfo("[MD_IK_Controller]: ------------------------------------------")


def main():

    """
    Description:    This node subscribes to `/mecanum_drive/cmd_vel` topic and 
                    return inidvidual wheel velocities via inverse kinematics calculations.
    Suggestions:    `anonymous=True` while running same node multiple times at once
    """

    rospy.init_node('mecanum_drive_ik_controller')
    rospy.loginfo("[MD_IK_Controller]: Mecanum_drive_controller node has started")

    get_params()

    sub_topic_name = robot_namespace + cmd_vel_topic
    rospy.Subscriber(sub_topic_name, Twist, cmd_vel_callback)


    try:
        rospy.spin()
    except KeyboardInterrupt:
        try:
            pass
        except Exception as e:
            pass
        print("[MD_IK_Controller] Shutting down the node")