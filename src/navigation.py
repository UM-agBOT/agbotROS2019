#!/usr/bin/env python

"""
UM-agBOT Weed and Feed agBOT Challenge 2018-2019
    Description:
	The ROS node for UM-agBOT navigation

    History:
	2019-05-08 by Franklin ogidi and George Dyck iii
	- Created.
"""

import tf
import time
import rospy
import numpy
import tf.transformations

from std_srvs.srv import Empty
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, Bool
from math import radians, sin, cos, pi
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

ROWDISTANCE= 0.762 # [m] distance between rows of corn

def calibrate_imu():
    """
    Calibrates the imu using ros service provided by Phidgets Inc.
    """
    rospy.wait_for_service('imu/calibrate')

    try:
	req = rospy.ServiceProxy('imu/calibrate', Empty)
	res = req()
	rospy.loginfo("IMU calibration done")
    except rospy.ServiceException, e:
	rospy.loginfo("Service call failed: %s"%e)

def imu_calibrate(msg):
    """
    Calibrates the imu, if it is not calibrated
    """
    if not msg.data:
	rospy.loginfo("Calbrating imu")
	calibrate_imu()

class Navigation:

    def __init__(self):
	self.odom_msg = Odometry()
	self.velocity_msg = Twist()
	self.steering_pose = Pose()

	self.yaw = 0.0
	self.yaw_ref = 0.0
	self.actual_speed = 0.0

	# angular velocity
	self.ang_vel_z = 0.0

	# publishing cue
	self.send_odom = False
	self.send_velocity = False
	self.send_steer_pose = False

	# configurable parameters
	self.num_of_passes = rospy.get_param("~number_of_passes", 2)
	self.target_speed = rospy.get_param("~target_speed", 10/3.6)
	self.target_distance = rospy.get_param("~target_distance", 154/3.281)
	self.min_steer_threshold = rospy.get_param("~min_steering_angle_threshold", 0.1222)
	self.max_steer_threshold = rospy.get_param("~max_steering_angle_threshold", 0.3491)

	# do not change
	self.imu_count = True

    def start(self, msg):
	"""
	Callback function that determines if it's okay to start dead reckoning 
	or not
	"""
	if msg.data:
	    # Wait for the imu_data to become available
	    rospy.loginfo("Waiting for imu/data topic...")
	    rospy.wait_for_message("imu/data", Imu)
	    self.dead_reck()

    def imu_data(self, msg):
	"""
	Callback function that extracts angular velocity and linear 
	acceleration from filtered imu msg
	"""
	if self.imu_count:
	    quaternion_ref = (msg.orientation.x,
			      msg.orientation.y,
			      msg.orientation.z,
			      msg.orientation.w)

	    euler_ref = tf.transformations.euler_from_quaternion(quaternion_ref)

	    self.yaw_ref = euler_ref[2]
	    #rospy.loginfo(self.yaw_ref*(180/pi))
	    self.imu_count = False

	# orientation
	quaternion = (	msg.orientation.x,
					msg.orientation.y,
					msg.orientation.z,
					msg.orientation.w)

	euler = tf.transformations.euler_from_quaternion(quaternion)

	self.yaw = -1 * (euler[2] - self.yaw_ref) # -1 is used because we've flipped the axis

	#if self.yaw > abs(0.3491):
	 #   rospy.loginfo("yaw from orientation transformation: "+str(self.yaw))

	# angular velocity
	self.ang_vel_z = msg.angular_velocity.z

    def vehicle_speed(self, msg):
	"""
	Callback function that gets the agBOT's speed, 
	determined from the vehicle's CAN bus
	"""
	self.actual_speed = (msg.data / 3.6)
	#rospy.loginfo("vehicle speed: "+str(self.actual_speed)+" m/s")

    def publish_odometry(self, x, y, th, vx, vy, vth, current_time):
	"""
	Used to publish odometry information while the agBOT is in motion

    	Parameters:
		x: positon of the agBOT in the x direction
		y: positon of the agBOT in the y direction
		th: rotation of the agBOT around the z direction
		vx: velocity of the agBOT in the x direction
		vy: velocity of the agBOT in the y direction
		vth: angular velocity of the agBOT about the z direction
		current_time: cuurent_time as assigned during the function call
	"""

	odom = Odometry()
	odom_broadcaster = tf.TransformBroadcaster()

	# create quaternion from yaw and transform from 'odom' frame to 'base_link' frame
	odom_quat = tf.transformations.quaternion_from_euler(0, 0, th)
	odom_broadcaster.sendTransform(
					(x, y, 0.0),
					odom_quat,
					current_time,
					"base_link",
					"odom")

	# fill odometry message variables
	odom.header.stamp = current_time
	odom.header.frame_id = "odom"
	odom.child_frame_id = "base_link"
	odom.pose.pose = Pose(Point(x, y, 0.0), Quaternion(*odom_quat))
	odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, vth))

	# publish the message
	self.odom_msg = odom
	self.send_odom = True

    def publish_steering_angle(self, wheel_angle):
	"""
	Used to publish steering angle 
	Parameters:
	    wheel angle: angle of the wheels w.r.t steering column
	"""
	steer_pos = Pose()

	steer_quat = tf.transformations.quaternion_from_euler(0, 0, wheel_angle)
	steer_pos = Pose(Point(0., 0., 0.), Quaternion(*steer_quat))
	#steer_pos.header.stamp = rospy.Time.now()
	#steer_pos.header.frame_id = "steering"

	self.steering_pose = steer_pos
	self.send_steer_pose = True

    def publish_velocity(self, target_speed):
	"""
	Used to publish the desired speed of the agBOT
	Parameters:
	    target_speed
	"""
	self.velocity_msg.linear.x = target_speed
	self.send_velocity = True

    def dead_reck(self):
	"""
	Estimate the position of the agBOT w.r.t a fixed frame using available sensor data.
	"""

	# Wait for the vehicle_speed topic to become available
	rospy.loginfo("Waiting for vehicle_speed topic...")
	rospy.wait_for_message("vehicle_speed", Float64)

	# publish desired speed
	self.publish_velocity(self.target_speed)		

	#initialize fixed global reference frame		
	x = 0.0
	y = 0.0
	th = 0.0

	x_deviation = 0
	turns = [1, -1, 1, -1]

	for i in range(self.num_of_passes):
		x, y, th = self.forward(self.target_distance + x_deviation, x, y, th)

		turn_speed = 5 / 3.6

		x_t, y_t, th_t = self.turn(i, turns[i], turn_speed, x, y, th)

		x += x_t
		y += x_t
		th += th_t

		x_deviation = self.target_distance - x

	# Stop agBOT
	self.publish_velocity(0)

    def forward(self, target_distance, x, y, th):
	"""
	Used to keep the agBOT in a straight line while moving forward

	Parameters:
		target_distance: The target distance for the agBOT to move torwards
		x: x position w.r.t fixed frame when function is called
		y: y position w.r.t fixed frame when function is called
		th: heading w.r.t fixed frame when function is called

	Returns:
		x: x position w.r.t fixed frame after target_distance is reached
		y: y position w.r.t fixed frame after target_distance is reached
		th: heading w.r.t fixed frame after target_distance is reached
	"""

	vy = 0
	vth = self.ang_vel_z
	vx = self.actual_speed

	ideal_distance = x
	current_distance = x

	last_time = rospy.Time.now()
	current_time = rospy.Time.now()

	rospy.loginfo("moving forward")
	while(current_distance < target_distance):
		# calculate lateral deviation
		th = self.yaw
		#rospy.loginfo(th*(180/pi))
		#correct steering, if necessary
		if (abs(th) > self.max_steer_threshold):
			self.publish_steering_angle(th)
			rospy.loginfo("Trying to correct by "+str(-th*(180/pi))+" degrees")

		# Distance calculus
		current_time = rospy.Time.now()
		current_speed = self.actual_speed
		dt = (current_time - last_time).to_sec()































		vy = 0
		vx = current_speed
		vth = self.ang_vel_z

		delta_x = (vx * cos(th) - vy * sin(th)) * dt
		delta_y = (vx * sin(th) + vy * cos(th)) * dt

		x += delta_x
		y += delta_y

		current_distance = x
		ideal_distance += (vx * dt)
		#rospy.loginfo("Current deviation in x "+str(ideal_distance - current_distance))

		# publish odometry
		self.publish_odometry(x, y, th, vx, vy, vth, current_time)

		last_time = current_time

	rospy.loginfo("forward movement done")
	return x, y, th

    def turn(self, turn_count, direction, turn_speed, x, y, th):
	""" 
	Used to perform a turn on the agBOT

	Parameters:
		turn_count: keeps track of number of turns
		direction: specifies 'left' or 'right' turn
		turn_speed: speed of agBOT during turn
		x: x position w.r.t fixed frame
		y: y position w.r.t fixed frame
		th: heading w.r.t fixed frame
	Returns:
		x: x position w.r.t fixed frame after target_distance is reached
		y: y position w.r.t fixed frame after target_distance is reached
		th: heading w.r.t fixed frame after target_distance is reached
	"""
	
	# move agBOT at desired speed
	self.publish_velocity(0.0)

	# publish steering angle to movement node
	self.publish_steering_angle(self.max_steer_threshold * -direction)
	time.sleep(5)
	self.publish_velocity(turn_speed)

	# initialize variable
	x_init = x

	# Set the current time for distance calculus
	current_time = rospy.Time.now()
	last_time = current_time

	if (direction == 1):
		rospy.loginfo("Turning Right")
		while(x >= x_init):
			th = self.yaw
			thresh = 0.01745

			if (th <= (-0.7854 + thresh) or th >= (-0.7854 - thresh)):
				self.publish_velocity(0)
				self.publish_steering_angle(self.max_steer_threshold * direction)
				time.sleep(5)
				self.publish_velocity(turn_speed)

			elif (th <= (3.054 + thresh) or th >= (3.054 - thresh)):
				self.publish_velocity(0)				
				self.publish_steering_angle(0)
				time.sleep(5)
				self.publish_velocity(turn_speed)
				if turn_count == 1:
					y_dev = 2 * ROWDISTANCE - y # sees how far off the agBOT is from target row, NOT GOOD FOR ALL CASES, just the first turn
					x_add, y_add = self.in_row_adjust(y_dev, -direction, 45.72, 64.01) 
					x += x_add
					y += y_add

			current_time = rospy.Time.now()
			current_speed = self.actual_speed
			dt = (current_time - last_time).to_sec()
				
			vy = 0.0
			vx = current_speed
			vth = self.ang_vel_z

			# Distance calculus
			vx_global = current_speed * cos(th)
			vy_global = current_speed * sin(th)

			delta_x = vx_global * dt 
			delta_y = vy_global * dt

			x += delta_x
			y += delta_y

			# publish the odometry msg
			self.publish_odometry(x, y, th, vx, vy, vth, current_time)

			last_time = current_time

	elif (direction == -1):
		rospy.loginfo("Turning Left")
		while(x <= x_init): 
			th = self.yaw
			thresh = 0.01745
				
			if (th <= (-0.7854 + thresh) or th >= (-0.7854 - thresh)):
				self.publish_steering_angle(self.max_steer_threshold * direction)
			
			elif (th <= (3.054 + thresh) or th >= (3.054 - thresh)):
				self.publish_velocity(0)				
				self.publish_steering_angle(0)
				time.sleep(5)
				self.publish_velocity(turn_speed)
				if turn_count == 2:
					y_dev = 5 * 0.762 - y # sees how far off the agBOT is from target row, NOT GOOD FOR ALL CASES, just the second turn
					x_add, y_add = self.in_row_adjust(y_dev, -direction, -18.29, 0.0) 
					x +=x_add
					y +=y_add
				
			current_time = rospy.Time.now()
			current_speed = self.actual_speed
			dt = (current_time - last_time).to_sec()
				
			vy = 0.0
			vx = current_speed
			vth = self.ang_vel_z

			# Distance calculus
			vx_global = current_speed * cos(th)
			vy_global = current_speed * sin(th)

			delta_x = vx_global * dt 
			delta_y = vy_global * dt

			x += delta_x
			y += delta_y

			# publish the odometry msg
			self.publish_odometry(x, y, th, vx, vy, vth, current_time)

			last_time = current_time

	return x, y, th
    
    def in_row_adjust(self, y_dev, direction, bottom, top):
        """
        This while loop will adjust the y position of the ATV until it is centered in the row 
        NOTE: if turning wrong way veering more off course, then swap the wheel angle directions
        """
        if (y_dev > 0):
            wheel_angle = self.min_steer_threshold * direction
        else:
            wheel_angle = -self.min_steer_threshold * direction
		
        # publish steering angle to movement node
	self.publish_steering_angle(wheel_angle)
		
        current_time = rospy.Time.now()
        last_time = current_time
        threshold = 0.1
	rospy.loginfo("doing an adjustment")
        while(y_dev > threshold or y_dev < -threshold and (bottom < x < top)): # THE BOTTOM AND TOP REFER TO THE ROW LENGTH
            	#rospy.loginfo("Doing an adjustment")
            
		# Get the angle to calculate 
		th = self.yaw

		current_time = rospy.Time.now()
		current_speed = self.actual_speed
		dt = (current_time - last_time).to_sec()

		vy = 0.0
		vx = current_speed
		vth = self.ang_vel_z

		# Distance calculus
		vx_global = current_speed * cos(th)
		vy_global = current_speed * sin(th)

		delta_x = vx_global * dt 
		delta_y = vy_global * dt

		x += delta_x
		y += delta_y
		y_dev += delta_y
           
		# publish the odometry msg
	        self.publish_odometry(x, y, th, vx, vy, vth, current_time)
			
	    	last_time = current_time
         
        self.publish_steering_angle(0)

        return x, y

def stop_agbot():
    """
    Inform movement node to stop the agBOT
    """
    nav = Navigation()
    nav.publish_velocity(0)
    time.sleep(5)

if __name__ == "__main__":
    try:
        rospy.init_node("navigation")
	rospy.Subscriber("imu/is_calibrated", Bool, imu_calibrate)

        nav = Navigation()

	rospy.Subscriber("startup", Bool, nav.start)
	rospy.Subscriber("imu/data", Imu, nav.imu_data)
	rospy.Subscriber("vehicle_speed", Float64, nav.vehicle_speed)


	odom_pub = rospy.Publisher("odom", Odometry, queue_size=10)
	vel_pub = rospy.Publisher("target_speed", Twist, queue_size=10)
	steering_pub = rospy.Publisher("steering_pose", Pose, queue_size=10)

	rate = rospy.Rate(10)

	while not rospy.is_shutdown():
		if nav.send_velocity:
			vel_pub.publish(nav.velocity_msg)
			nav.send_velocity = False
		if nav.send_odom:
			odom_pub.publish(nav.odom_msg)
			nav.send_odom = False
		if nav.send_steer_pose:
			steering_pub.publish(nav.steering_pose)
			nav.send_steer_pose = False
		rate.sleep()

		rospy.on_shutdown(stop_agbot)
    except rospy.ROSInterruptException:
        pass

