#!/usr/bin/env python
# -*- coding: utf-8 -*-

# ****************************************************************************
# UM-agBOT Biosystems Capstone Group, 2018-2019
#
# Description:
#   The ROS node in charge of activating spray nozzles based on input from
#   YOLO object detection model.
#
# History:
#   2019-03-15 by Franklin ogidi
#   - Created.
#   2019-04-10 by Franklin ogidi
#   - Fixed nozzle localization issue
#   2019-04-11 by Franklin Ogidi
#   - Added y-axis of bounding box for spray time delay
#   2019-04-22 by Franklin Ogidi
#   - Added threading
#   2019-07-16 by Franklin Ogidi
#   - Changed code structure to include Sprayer class
#   - Modified code to publish nozzle state as a Bool so that it can be
#     sent to a web server for use on an app
#   2020-03-07 by Franklin Ogidi
#   - Updated for compatibility with the latest JCA Controller API
#	http://docs.jcaelectronics.ca/jcaroscontroller/ref-falcon/
# *****************************************************************************

import time
import rospy
import threading

from darknet_ros_msgs.msg import BoundingBoxes
from std_msgs.msg import Float64, Int64, Bool, String
from jca_controller_ros_nodes_interface.msg import Output

class Sprayer:
	
    def __init__(self):
	self.image_width = 640 	#px
	self.image_height = 480 #px

	self.speed = 0.0
	self.foxtail_counter = 0
	self.cocklebur_counter = 0
	self.healthy_corn_counter = 0
	self.diseased_corn_counter = 0
	self.giant_ragweed_counter = 0

	# Nozzle positions
	self.nozzle_1_position = 0
	self.nozzle_2_position = 0
	self.nozzle_3_position = 0
	self.nozzle_4_position = 0
	self.nozzle_5_position = 0
	self.nozzle_6_position = 0
	
	# falcon publisher
	self.falcon_pub = rospy.Publisher('/falcon1/command/output', Output, queue_size=10, latch=True)

	# count publishers
	self.diseased_corn_count = rospy.Publisher("dieseased_corn_count", Int64, queue_size=1, latch=True)
	self.giant_ragweed_count = rospy.Publisher("giant_ragweed_count", Int64, queue_size=1, latch=True)
	self.healthy_corn_count = rospy.Publisher("healthy_corn_count", Int64, queue_size=1, latch=True)
	self.cocklebur_count = rospy.Publisher("cocklebur_count", Int64, queue_size=1, latch=True)
	self.foxtail_count = rospy.Publisher("foxtail_count", Int64, queue_size=1, latch=True)

    def turn_on_pump(self):
	"""
	Turns on the pump once the script is started. Pump stays on until node is shutdown.
	"""

	msg = Output()
	msg.name = 'hso_1'
	msg.value = 100
	self.falcon_pub.publish(msg)
	
    def get_vehicle_speed(self, msg):
	"""
	Gets vehicle speed (in km/hr) from capstone_ros package. Converts units to m/s.
	Reference: https://github.com/rahmant3/capstoneROS2018_public
	"""

	#rospy.loginfo(msg)
	self.speed = 0.277778 * msg.data #m/s
		
    def set_boom_parameters(self):
	#ATV Boom parameters
	nozzle6_range = 50 			# 50 cm
	nozzle5_range = 60 			# 60 cm
	nozzle4_range = nozzle5_range + 50 	# 110 cm
	nozzle3_range = nozzle4_range + 50 	# 160 cm
	nozzle2_range = nozzle3_range + 50 	# 210 cm
	nozzle1_range = nozzle3_range + 60 	# 220 cm
	boom_length = nozzle1_range 		# 220 cm

	#normalize boom parameters by image width
	self.nozzle_1_position = float(nozzle1_range) / boom_length * self.image_width	# 640 px
	self.nozzle_2_position = float(nozzle2_range) / boom_length * self.image_width	# 611 px
	self.nozzle_3_position = float(nozzle3_range) / boom_length * self.image_width	# 465 px
	self.nozzle_4_position = float(nozzle4_range) / boom_length * self.image_width  # 320 px
	self.nozzle_5_position = float(nozzle5_range) / boom_length * self.image_width	# 175 px
	self.nozzle_6_position = float(nozzle6_range) / boom_length * self.image_width	# 145 px
    
    def process_bounding_box(self, msg):
	"""
	Gets array of bounding boxes from darknet_ros package. Determines nozzle to activate.
	Reference: https://github.com/leggedrobotics/darknet_ros
	"""
	#rospy.loginfo(msg)

	bounding_box = msg.bounding_boxes

	for i in range(len(bounding_box)):
  	    obj = bounding_box[i].Class
	    #prob = bounding_box[i].probability

	    if self.speed == 0:
		time_to_nozzle = 0
		print("Detected "+obj+"; will not spray unless vehicle is moving")
	    else:
		xmin = bounding_box[i].xmin
		ymin = bounding_box[i].ymin
		xmax = bounding_box[i].xmax
		ymax = bounding_box[i].ymax
				
		#find center of bounding box
		xmid = xmin + ((xmax - xmin) * 0.5)
		ymid = ymin + ((ymax - ymin) * 0.5)

		distance = 4 + ((float(ymid) / self.image_height) * 1.60) #meters
		time_to_nozzle = float(distance) / self.speed

		if obj == "corn (healthy)":
		    self.healthy_corn_counter += 1
		    output_pin = self.select_corn_nozzle(xmid)
		    self.healthy_corn_count.publish(self.healthy_corn_counter)
		elif obj == "corn (diseased)":
		    self.diseased_corn_counter += 1
		    output_pin = self.select_corn_nozzle(xmid)
		    self.diseased_corn_count.publish(self.diseased_corn_counter)
		elif obj == "giant ragweed":
		    self.giant_ragweed_counter += 1
		    output_pin = self.select_weed_nozzle(xmid)
		    self.giant_ragweed_count.publish(self.giant_ragweed_counter)
		elif obj == "foxtail":
		    self.foxtail_counter += 1
		    output_pin = self.select_weed_nozzle(xmid)
		    self.foxtail_count.publish(self.foxtail_counter)
		elif obj == "cocklebur":
		    self.cocklebur_counter += 1	
		    output_pin = self.select_weed_nozzle(xmid)
		    self.cocklebur_count.publish(self.cocklebur_counter)
		else:
		    rospy.loginfo(obj+" is not a valid object!")

		threading.Timer(time_to_nozzle, self.activate_nozzle, args=[output_pin]).start()
				
    def select_corn_nozzle(self, xmid):
	if xmid >= self.nozzle_3_position:
	    output_pin = "hso_8" # nozzle 2
	elif xmid <= self.nozzle_5_position:
	    output_pin = "hso_6" # nozzle 5
	else:
	    rospy.loginfo("Output pin could not be determined")
	    output_pin = ""
			
	return output_pin
	
    def select_weed_nozzle(self, xmid):
	if xmid <= self.nozzle_6_position:
  	    output_pin = "hso_3" # nozzle 6
	elif xmid <= self.nozzle_4_position and xmid > self.nozzle_6_position:
	    output_pin = "hso_10" # nozzle 4
	elif xmid <= self.nozzle_3_position and xmid > self.nozzle_4_position:
	    output_pin = "hso_7" # nozzle 3
	elif xmid <= self.nozzle_1_position and xmid > self.nozzle_4_position:
	    output_pin = "hso_5" # nozzle 1
	else:
	    rospy.loginfo("Output pin could not be determined")
	    output_pin = ""
			
	return output_pin
	
    def activate_nozzle(self, output_pin):
	"""
	Activates a nozzle for a specified duration of time.
	"""
	if output_pin != "":
	    falcon_msg = Output()
	    falcon_msg.name = output_pin
	    falcon_msg.value = float(100)
	    self.falcon_pub.publish(falcon_msg)

	    spray_duration = 0.25 #s

	    falcon_msg.value = float(0)
	    time.sleep(spray_duration)
	    self.falcon_pub.publish(falcon_msg)

	else:
	    rospy.loginfo("No nozzle was activated because output pin could not be determined")
	
    def turn_off_pump(self):
	"""
	Turns off pump.
	"""

	msg = Output()
	msg.name = 'hso_1'
	msg.value = 0
	self.falcon_pub.publish(msg)

if __name__ == "__main__":
    try:
        rospy.init_node("sprayer")
		
	sprayer_manager = Sprayer()
	sprayer_manager.turn_on_pump()
	sprayer_manager.set_boom_parameters()
	
	rospy.Subscriber("vehicle_speed", Float64, sprayer_manager.get_vehicle_speed)
	rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, sprayer_manager.process_bounding_box)
		
        rospy.on_shutdown(sprayer_manager.turn_off_pump)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

