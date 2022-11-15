#!/usr/bin/env python

# rospy is the pure Python client library for ROS. Enables interface with ROS topics, Services, and parameters
import rospy
# geometry_msgs provides messages for common geometric primitives
# We are importing the PoseStamped to handle the message type of the car pose
from geometry_msgs.msg import PoseStamped
# Markers allow programatic addition of shapes to the 3D view
from visualization_msgs.msg import Marker

# Rotation is a tool to allow us to get radians from quaternions
from scipy.spatial.transform import Rotation as R

import numpy as np

LOOK_AHEAD_DISTANCE = 1


# We usually pass a function pointer to the call back function
def callback(data): 
    rospy.loginfo(rospy.get_caller_id() + 'Car\'s pose: %s', data.pose)
    # Unpack the x and y position coordinates from the pose
    x = data.pose.position.x
    y = data.pose.position.y

    o_x = data.pose.orientation.x
    o_y = data.pose.orientation.y
    o_z = data.pose.orientation.z
    o_w = data.pose.orientation.w

    r = R.from_quat([o_x, o_y, o_z, o_w])
    yaw = r.as_euler('zyx', degrees=False)[0]

    look_ahead_x = x + LOOK_AHEAD_DISTANCE * np.cos(yaw)
    look_ahead_y = y + LOOK_AHEAD_DISTANCE * np.sin(yaw)

    l_a_p_marker = Marker()
    # Configure our waypoint's properties
    l_a_p_marker.header.frame_id = "/map"
    l_a_p_marker.type = l_a_p_marker.SPHERE
    l_a_p_marker.action = l_a_p_marker.ADD
    l_a_p_marker.scale.x = 0.05
    l_a_p_marker.scale.y = 0.05
    l_a_p_marker.scale.z = 0.05
    l_a_p_marker.color.a = 1.0
    l_a_p_marker.color.r = 0.4
    l_a_p_marker.color.g = 0.4
    l_a_p_marker.color.b = 0.4
    l_a_p_marker.pose.orientation.w = 1.0
    l_a_p_marker.id = 1000
    # Read the raw coordinate infromation into the waypoint's x and y position
    l_a_p_marker.pose.position.x = look_ahead_x
    l_a_p_marker.pose.position.y = look_ahead_y
    # Zero the z position as we are dealing with 2D Space
    l_a_p_marker.pose.position.z = 0

    pub_controls.publish(l_a_p_marker)

if __name__ == '__main__':
    # Initialize the baseline controller node
    rospy.init_node('lookahead_visualizer')

    # Collect the control topic to publish the steering instructions to
    control_topic = '/look_ahead_visual'
    # Congifure the publisher nodes interaface
    pub_controls = rospy.Publisher(control_topic, Marker, queue_size=1)

    # Set the control publish frequency
    rospy.sleep(1.0)

    # Begin the lookahead visualizer
    rospy.Subscriber('/car/car_pose', PoseStamped, callback)

    rospy.sleep(1.0)
    
    
    # use rospy.spin and let lidar callback handle the control
    rospy.spin()