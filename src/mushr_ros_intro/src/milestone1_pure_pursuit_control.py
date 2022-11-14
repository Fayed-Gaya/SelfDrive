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
# Rotation is a tool to allow us to get radians from quaternions
from scipy.spatial.transform import Rotation as R

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


def reference_path_loader():
    """
    Builds waypoints from pose from an input file and loads them into a reference path
    """
    # Store raw coordinates locally in a list
    raw_reference_path = []
    # Read in raw coordinate information from the reference path input file and load to a global reference array
    with open(REFERENCE_PATH_INPUT_FILE, 'r') as f:
        for line in f:
            # Process the raw coordinate(x,y) data
            line = line.strip()
            raw_coordinates = line.split(',')
            raw_waypoint = [float(raw_coordinates[0]), float(raw_coordinates[1])]
            # Append the raw coordinates to the raw reference path
            raw_reference_path.append(raw_waypoint)
    
    # Assign our list into our global numpy array
    reference_path = np.array(raw_reference_path)

    return reference_path

def nearest_waypoint_current_pose(reference_path, current_pose):
    """
    Calculate the nearest waypoint to the current pose of the car along the reference path.
    """

    # Convert the current pose into a numpy array
    # current_pose_array = np.array([float(current_pose.position.x), float(current_pose.position.y)])
    # current_pose_array = np.expand_dims(current_pose_array, axis=1).T


    print(np.shape(current_pose))
    print(np.shape(reference_path))

    # Calculate an array of all of the distances between the current pose and the poses of the waypoints that make up the reference path
    distance_array = np.linalg.norm(current_pose - reference_path, axis=1)

    print('DIST ARRAY %s' % np.shape(distance_array))
    # Calculate the minimim distance way point as an array
    min_val_index = np.argmin(distance_array)
    print('MIN INDEX %s' % min_val_index)
    print('MIN VALUE %s' % distance_array[min_val_index])
    # Unpack the x and y coordinates of the nearest waypoint
    nearest_x_coordinate = reference_path[min_val_index][0]
    nearest_y_coordinate = reference_path[min_val_index][1]

    return np.array(nearest_x_coordinate, nearest_y_coordinate)

def look_ahead_pt_calculator(current_pose_orientation, current_pos, l_d):
    """
    Calculate the lookahead point.
    """
    x = current_pose_orientation.x
    y = current_pose_orientation.y
    z = current_pose_orientation.z
    w = current_pose_orientation.w

    r = R.from_quat([x, y, z, w])
    yaw = r.as_euler('zyx', degrees=False)[0]

    look_ahead_x = current_pos[0] + l_d * np.cos(yaw)
    look_ahead_y = current_pos[1] + l_d * np.cos(yaw)

    look_ahead_pt = np.array([look_ahead_x, look_ahead_y])

    return look_ahead_pt

def current_pose_callback(data, args):
    """
    Calculate the current position of the car.
    Calculate the cross-track error.
    Calculate the heading error.

    : param data: Node
    : param args: Tuple containing publisher controls and the reference path.
    """

    # Unpack the node API
    pub_controls = args[0]
    # Unpack the reference path
    reference_path = args[1]

    # Store the car's current pose
    current_pose = np.array([data.pose.position.x, data.pose.position.y])

    # Calculate the nearest waypoint tp the current position
    np_actual = nearest_waypoint_current_pose(reference_path, current_pose)

    # Calculate the lookahead point
    look_ahead_pt = look_ahead_pt_calculator(data.pose.orientation, current_pose, LOOK_AHEAD_DISTANCE)

    print(look_ahead_pt)

    # Calculate the nearest waypoint to the the 

    # Calculate the L2 Norm, cross-track error
    cross_track_error = np.linalg.norm(np_actual - current_pose)
    

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

    

    
    # Calculate the distance from the car to the wall at its left
    left_dist = np.nanmean(np.array(data.ranges[LIDAR_LEFT-LIDAR_RANGE:LIDAR_LEFT+LIDAR_RANGE]))
    # Calculate the distance from the car to the wall on its right
    right_dist = np.nanmean(np.array(data.ranges[LIDAR_RIGHT-LIDAR_RANGE:LIDAR_RIGHT+LIDAR_RANGE]))
    # Calculate the control value
    control_value = - left_dist + right_dist
    # Log the control value
    # rospy.loginfo('Control Value: %s', control_value)

    # If the control value is not a number set it to 0
    if np.isnan(control_value): control_value = 0
    # Caculate the steering angle using the PID
    steering_angle = pid(control_value)
    # Log the steering angle
    # rospy.loginfo('Steering Angle: %s', steering_angle)

    # Create the drive message containing the corrective steering angle and speed
    drive = AckermannDrive(steering_angle=steering_angle, speed=1.0)
    # Publish the steering angle and speed
    pub_controls.publish(AckermannDriveStamped(drive=drive))

if __name__ == "__main__":
    # Load the reference path
    reference_path = reference_path_loader()



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

    # initialize the car's initial pose
    init_pose = [0., 0., np.pi]
    send_init_pose(pub_init_pose, init_pose)

    # Begin the LIDAR scan and subsequently trigger the steering and drive controls
    # sub_lidar_scan = rospy.Subscriber("/car/scan", LaserScan, lidar_callback, (pub_controls, reference_path))

    # Begin Pure Pursuit
    rospy.Subscriber('/car/car_pose', PoseStamped, current_pose_callback, (pub_controls, reference_path))
    # Set the control publish frequency
    rospy.sleep(1.0)
    
    # use rospy.spin and let lidar callback handle the control
    rospy.spin()