#!/usr/bin/env python

# Pure client library for ROS
import rospy
# NumPy adds large array support and a collection of mathematical functions
import numpy as np
# ackermann_msgs provides the steering and velocity drive messages
from ackermann_msgs.msg import AckermannDrive, AckermannDriveStamped
# LaserScan messages transmit LIDAR data
from sensor_msgs.msg import LaserScan
# geometry_msgs provide message types for common geometric primitves
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import PoseWithCovariance
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Quaternion
# tf.transformations is a library for performing calculatiosn on 4x4 matrices
from tf.transformations import quaternion_from_euler
# Simple and easy to use PID Controller
from simple_pid import PID

# Initalize PID controller and the corresponding multiples
pid = PID(2.5, 0.2, 0.1, setpoint=0.0)

# Define the left and right of the AV in degrees and the LIDAR range
LIDAR_LEFT = 450
LIDAR_RIGHT = 270
LIDAR_RANGE = 30

# Set the look ahead distance
LOOK_AHEAD_DISTANCE = 0.5

# Set the waypoint coordinates input file
REFERENCE_PATH_INPUT_FILE = "/home/hanwen/catkin_ws/src/mushr_ros_intro/src/reference_path_ideal.txt"

# Store the reference path made up of pose
reference_path = PoseArray()

# Store the car's current pose
current_pose = Pose()

# Determine the lookahaed position
# Calculate the ehading vector
# Determine the closest point on the trajectory from car position
# Calculate the trajectory vector
# Calculate the Cross-Track Error
# Use the Stanley Formulation to determine steering angle
# Imporve the look-ahead distance based on the speed of the vehicle
# Visualize reference path in Rviz
# Visualize the lookahead position
# Visualize waypoint markers

def reference_path_loader():
    """
    Builds waypoints from pose from an input file and loads them into a reference path
    """

    # Read in raw coordinate information from the reference path input file and load to a global reference array
    with open(REFERENCE_PATH_INPUT_FILE, 'r') as f:
        for line in f:
            # Process the raw coordinate(x,y) data
            line = line.strip()
            raw_coordinates = line.split(',')

            # Convert the coordinates to pose
            x = float(raw_coordinates[0])
            y = float(raw_coordinates[1])
            point = Point(x=x, y=y)
            pose = Pose(position=point)
            # Store pose in the reference path
            reference_path.poses.append(pose)

def current_pose_callback(data):
    """
    Update the global current car pose variable with the car's current pose.

    :param data: The node that is subscribed to the car's current pose
    """
    # Store the car's current pose
    current_pose = data.pose

def car_pose_listener():
    """
    This node is a subscriber.
    It listens for the car's curent pose.
    """

    # Initialize the node
    rospy.init_node('car_position_listener')

    # Declares our node subscribes to the car pose topic
    # When a new message is received callback is invoked with message as the first argument
    rospy.Subscriber('/car/car_pose', PoseStamped, current_pose_callback)

    # spin() simply keeps python from exiting until this node is stopped. Should I replace spin() with some sort of frequency controlled subscribe?
    rospy.spin()

def nearest_waypoint():
    """
    Collect the car's current pose.
    Use it to calculate the nearest waypoint.
    Return the nearest waypoint.
    """

    current_pose
def send_init_pose(pub_init_pose, init_pose):
    """
    Publishes the initial pose of the car.
    
    :param pub_init_pose: Publisher object containing information on the publish topic, message type, and queue
    :param init_pose: 3 value list containg the x, y, z euler coordinates of the car's starting position
    """

    # Unpack the initial x,y,z coordinates
    x, y, theta = float(init_pose[0]), float(init_pose[1]), float(init_pose[2])
    # Return a quaternion from Euler angles
    q = Quaternion(*quaternion_from_euler(0, 0, theta))
    # Define the x,y coordinates of the point in the pose
    point = Point(x=x, y=y)
    # Define the initial pose
    pose = PoseWithCovariance(pose=Pose(position=point, orientation=q))
    # Publish the initial pose
    pub_init_pose.publish(PoseWithCovarianceStamped(pose=pose))

def lidar_callback(data, args):
    """
    Makes steering and power decisions about our car.
    Publishes those decisions.

    :param data: The data type of the publishing node
    :param args: The node that publishes
    """

    # Localize the publisher node
    pub_controls = args
    # Calculate the distance from the car to the wall at its left
    left_dist = np.nanmean(np.array(data.ranges[LIDAR_LEFT-LIDAR_RANGE:LIDAR_LEFT+LIDAR_RANGE]))
    # Calculate the distance from the car to the wall on its right
    right_dist = np.nanmean(np.array(data.ranges[LIDAR_RIGHT-LIDAR_RANGE:LIDAR_RIGHT+LIDAR_RANGE]))
    # Calculate the control value
    control_value = - left_dist + right_dist
    # Log the control value
    rospy.loginfo('Control Value: %s', control_value)

    # If the control value is not a number set it to 0
    if np.isnan(control_value): control_value = 0
    # Caculate the steering angle using the PID
    steering_angle = pid(control_value)
    # Log the steering angle
    rospy.loginfo('Steering Angle: %s', steering_angle)

    # Create the drive message containing the corrective steering angle and speed
    drive = AckermannDrive(steering_angle=steering_angle, speed=1.0)
    # Publish the steering angle and speed
    pub_controls.publish(AckermannDriveStamped(drive=drive))

if __name__ == "__main__":
    # Initialize the baseline controller node
    rospy.init_node('baseline_pid')

    # Collect the control topic to publish the steering instructions to
    control_topic = rospy.get_param("~control_topic", "/car/mux/ackermann_cmd_mux/input/navigation")
    # Congifure the publisher nodes interaface
    pub_controls = rospy.Publisher(control_topic, AckermannDriveStamped, queue_size=1)

    # Collect the intial pose topic
    init_pose_topic = rospy.get_param("~init_pose_topic", "/initialpose")
    # Publish the inital pose
    pub_init_pose = rospy.Publisher(init_pose_topic, PoseWithCovarianceStamped, queue_size=1)

    # Begin the LIDAR scan and subsequently trigger the steering and drive controls
    sub_lidar_scan = rospy.Subscriber("/car/scan", LaserScan, lidar_callback, (pub_controls))

    # Set the control publish frequency
    rospy.sleep(1.0)

    # initialize the car's initial pose
    init_pose = [0., 0., np.pi]
    send_init_pose(pub_init_pose, init_pose)
    
    # use rospy.spin and let lidar callback handle the control
    rospy.spin()