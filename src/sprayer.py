#!/usr/bin/env python
# -*- coding: utf-8 -*-

# ****************************************************************************
# UM-agBOT Biosystems Capstone Group, 2018-2019
#
# Description:
#   The ROS node in charge of activating spray nozzles based on input from
#   object detection model.
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
# *****************************************************************************

import time
import rospy
import threading
from std_msgs.msg import Int64
from std_msgs.msg import Float64
from falcon_ros_node.msg import falcon_output
from darknet_ros_msgs.msg import BoundingBoxes

def turn_on_pump():
    """
    Turns on the pump once the script is on. Pump stays on until node is shutdown.
    """
	
    msg = falcon_output()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "pump is on"
    msg.value = float(100)
    pub = rospy.Publisher('/falcon/outputs/hso_1', falcon_output, queue_size=1, latch = True)
    pub.publish(msg)

def get_vehicle_speed(msg):
    """
    Gets vehicle speed (in km/hr) from capstone_ros package. Converts units to m/s.
    """
	
    #rospy.loginfo(msg)
	
    global speed
    speed = 0.277778 * msg.data #m/s
    
def process_bounding_box(msg):
    """
    Gets array of bounding boxes from darknet_ros package. Determines nozzle to activate.
    """
    #rospy.loginfo(msg)

    global speed
    global weed_counter
    global corn_counter
    output_pin = ""
    image_width = 640 	#px
    image_height = 480 	#px
    
    #ATV Boom parameters
    nozzle6_range = 50 			# 50 cm
    nozzle5_range = 60 			# 60 cm
    nozzle4_range = nozzle5_range + 50 	# 110 cm
    nozzle3_range = nozzle4_range + 50 	# 160 cm
    nozzle2_range = nozzle3_range + 50 	# 210 cm
    nozzle1_range = nozzle3_range + 60 	# 220 cm
    boom_length = nozzle1_range 	# 220 cm

    #normalize boom parameters by image width
    s1 = float(nozzle1_range) / boom_length * image_width	# 640 px
    s2 = float(nozzle2_range) / boom_length * image_width	# 611 px
    s3 = float(nozzle3_range) / boom_length * image_width	# 465 px
    s4 = float(nozzle4_range) / boom_length * image_width   	# 320 px
    s5 = float(nozzle5_range) / boom_length * image_width	# 175 px
    s6 = float(nozzle6_range) / boom_length * image_width	# 145 px
    
    bounding_box = msg.bounding_boxes
    
    for i in range(len(bounding_box)):
        obj = bounding_box[i].Class
        xmin = bounding_box[i].xmin
        ymin = bounding_box[i].ymin
        xmax = bounding_box[i].xmax
        ymax = bounding_box[i].ymax
        #prob = bounding_box[i].probability

        #find center of bounding box
        xmid = xmin + ((xmax - xmin) * 0.5)
        ymid = ymin + ((ymax - ymin) * 0.5)
        
        if obj == "corn (healthy)" or obj == "corn (diseased)":
  	    corn_counter += 1
	    corn_count.publish(corn_counter)
            if xmid >= s3:
                output_pin = "hso_8" #solenoid 2
            elif xmid <= s5:
                output_pin = "hso_6" #solenoid 5
        else:
	    weed_counter += 1
	    weed_count.publish(weed_counter)
            if xmid <= s6:
                output_pin = "hso_3" #solenoid 6
            elif xmid <= s4 and xmid > s6:
                output_pin = "hso_10" #solenoid 4
            elif xmid <= s3 and xmid > s4:
                output_pin = "hso_7" #solenoid 3
            elif xmid <= s1 and xmid > s4:
                output_pin = "hso_5" #solenoid 1
	distance = 4 + ((float(ymid) / image_height) * 1.60) #meters

        if speed == 0:
            time_to_nozzle = 0
        else:
            time_to_nozzle = float(distance) / speed
	
        threading.Timer(time_to_nozzle, activate_nozzle, args=[output_pin]).start()
	
def activate_nozzle(output_pin):
    """
    Activates a nozzle for a specified duration of time.
    """

    pub = rospy.Publisher("/falcon/outputs/"+output_pin, falcon_output, queue_size=10, latch = True)
	
    falcon_msg = falcon_output()
    falcon_msg.header.stamp = rospy.Time.now()
    falcon_msg.header.frame_id = "sprayed "+output_pin
    falcon_msg.value = float(100)
    pub.publish(falcon_msg)
	
    spray_duration = 0.25 #s
	
    falcon_msg.header.stamp = rospy.Time.now()
    falcon_msg.header.frame_id = output_pin+" off"
    falcon_msg.value = float(0)
    time.sleep(spray_duration)
    pub.publish(falcon_msg)
	
def turn_off_pump():
    """
    Turns off pump.
    """
    pub = rospy.Publisher("/falcon/outputs/hso_1", falcon_output, queue_size=1, latch = True)
    msg = falcon_output()
    msg.header.frame_id = "pump is off"
    msg.value = float(0)
    pub.publish(msg)

if __name__ == "__main__":
    try:
        rospy.init_node("sprayer")
        weed_counter = 0
	corn_counter = 0
	turn_on_pump()
        weed_count = rospy.Publisher("weed_count", Int64, queue_size=1, latch=True)
	corn_count = rospy.Publisher("corn_count", Int64, queue_size=1, latch=True)
	rospy.Subscriber("vehicle_speed", Float64, get_vehicle_speed)
	rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, process_bounding_box)
        rospy.on_shutdown(turn_off_pump)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

