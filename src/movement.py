#!/usr/bin/env python

import time
import rospy
import subprocess
import tf.transformations
import numpy
from std_msgs.msg import String
from std_msgs.msg import Float64
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist
from falcon_ros_node.msg import falcon_output
from falcon_ros_node.msg import falcon_input

####Steering Variables###
TURNTOL=0.05 # [V] the turning tolerance
HOMESTATE_STEER=1.391 # [V] the position where the ATV will idealy drive 'straight'

###Throttle Variables###
HOMESTATE=2.85 #the position of the actuator where it is only ideling and not moving the ATV
VOLTSperMS=0.0482 # conversion from m/s to volts on potentiometer (its linear)
SPEEDTOL=0.001
currentSpeed=0

def turn_to_voltage(turnreading):
    #linear y=mx+b #y is voltage #x is position in degrees (0 when going straight)
    valueturn=turnreading * 0.0608 + 1.391 # data was gathered and processed in excel to get linear model
    return valueturn

def adjust_steering(turnMsg):
    quaternion=(
	turnMsg.orientation.x,
	turnMsg.orientation.y,
	turnMsg.orientation.z,
	turnMsg.orientation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    targetTurn=euler[2] # produces yaw (angle about z in rads) from the ROS message
    targetTurnVolt = turn_to_voltage(numpy.rad2deg(targetTurn))
    #print(numpy.rad2deg(targetTurn))
    turnAdjData= targetTurnVolt - hum_read("hbinvolt3") # compairs throttle target voltage to current voltage
    #print(turnAdjData)
    msg1=falcon_output()
    msg2=falcon_output()
    if (targetTurnVolt>0 and targetTurnVolt<2.62):
        while (turnAdjData>TURNTOL or turnAdjData<(-1*TURNTOL)):
	    if (turnAdjData>TURNTOL):
	        #print("test right")
	        msg1.value=float(0)
	        msg2.value=float(100)
	        steeringAdj1.publish(msg1) #will turn right
    	        steeringAdj2.publish(msg2)
	    elif (turnAdjData<(-1*TURNTOL)):
	        print("test left")
	        msg1.value=float(100)
	        steeringAdj1.publish(msg1) # will turn left
	        msg2.value=float(0)
	        teeringAdj2.publish(msg2)
	    turnAdjData=targetTurnVolt-hum_read("hbinvolt3")
	    #print(turnAdjData)
        msg1.value=float(0) # will stop turning process
        msg2.value=float(0)
        steeringAdj1.publish(msg1)
        steeringAdj2.publish(msg2)

def hum_read(value3):
    command3="cat /dev/"+value3
    humReadResult=float(subprocess.check_output(command3, stderr=subprocess.STDOUT , shell=True))
    return humReadResult

def hum_command(pin1, value1, pin2, value2):
    command1="echo "+value1+" > /dev/"+pin1
    command2="echo "+value2+" > /dev/"+pin2
    subprocess.call(command1, shell=True)
    subprocess.call(command2, shell=True)

def brake(result): # applies brake for 3.5 seconds at maximum pressure, then releases
    if (result.data):
        pin1="hboutduty2"
        pin2="hboutduty3"
        #print("Braking")
        hum_command(pin1, '0', pin2, '0')
        hum_command(pin1, '1000', pin2, '0') # moves braking actuator in
        time.sleep(2)
        hum_command(pin1, '0', pin2, '0') # resets pins
        hum_command(pin1, '0', pin2, '1000') # moves braking actuator back
        time.sleep(2)
        hum_command(pin1, '0', pin2, '0') # resets pins

def speed_to_Voltage(speedreading):
#	WORK TO BE DONE HERE#### see following for instructions
#    	1) turnon ATV findout what actuator position gets the agBOT only ideling, use this as HOMESTATE
#    	2) find the actuator position to get the ATV running at 5m/s this will be RUNNING SPEED (unless we want to move slower)
# 	the same idea as the turn_to_voltage

    value = -1 * speedreading * VOLTSperMS + HOMESTATE
    return value

def update_speed(canSpeedReading):
    currentSpeed = canSpeedReading.data
    if (currentSpeed>(12)): #kills the machine if it exceeds 12km/h or 2
	kill_switch(1)

def adjust_speed(targetSpeedMSG):
    targetSpeed = targetSpeedMSG.linear.x
    speedAdjustData=speed_to_Voltage(targetSpeed)-hum_read("hbinvolt2") # calculates the difference between throttle position and position of throttle to achieve the desired speed
    while(speedAdjustData>SPEEDTOL or speedAdjustData<(-1*SPEEDTOL)):
        if (speedAdjustData>SPEEDTOL):
            throttleAdj.publish(1) # speeds up ATV
	    #print("speeding up")
        elif (speedAdjustData<(-1*SPEEDTOL)):
	    throttleAdj.publish(-1) # slows down ATV
	    #print("slowing down")
	time.sleep(0.01)
	throttleAdj.publish(0)
	speedAdjustData=speed_to_Voltage(targetSpeed)-hum_read("hbinvolt2") #updates the value for the while loop
	#print(hum_read("hbinvolt2"))
    #print("checkpoint2")

def move_throttle_homestate(): #makes sure actuator is in correct position
    while (hum_read("hbinvolt2")>(0.01 + HOMESTATE) or hum_read("hbinvolt2")<(HOMESTATE-0.01)):
	if (hum_read("hbinvolt2")> HOMESTATE):
	    #print("IN")
            throttleAdj.publish(-1) #will speed the ATV to the ideling homestate
        elif (hum_read("hbinvolt2")<HOMESTATE):
	    #print("OUT")
            throttleAdj.publish(1) # will slow the ATV to the ideling homestate
    #print("throttle at home-state")
    startup.publish(True)

def move_steering_homestate():
    currentSteeringPos=hum_read("hbinvolt3")
    while (currentSteeringPos>(HOMESTATE_STEER+TURNTOL) or currentSteeringPos<(HOMESTATE_STEER-TURNTOL)):
	msg1=falcon_output()
	msg2=falcon_output()
	if (currentSteeringPos > (HOMESTATE_STEER+TURNTOL)):
	    msg1.value=float(100)
    	    msg2.value=float(0)
	    steeringAdj1.publish(msg1) # moves the steering right
	    steeringAdj2.publish(msg2)
	    time.sleep(0.05)
	elif (currentSteeringPos<(HOMESTATE_STEER-TURNTOL)):
	    msg1.value=float(0)
	    msg2.value=float(100)
	    steeringAdj1.publish(msg1)
	    steeringAdj2.publish(msg2)
            time.sleep(0.05)
	currentSteeringPos=hum_read("hbinvolt3")
    print("steering at home-state")
    msg1=falcon_output()
    msg2=falcon_output()
    msg1.value=float(0)
    msg2.value=float(0)
    steeringAdj1.publish(msg1)
    steeringAdj2.publish(msg2)

def kill_switch(kill):
    if (kill):
        pin1="hboutduty2"
	pin2="hboutduty3"
	#hum_command(pin1, '0', pin2, '0')
	#hum_command(pin1, '1000', pin2, '0') # locks up brake incase emergency
	move_throttle_homestate()
	#move_steering_homestate()
def shutdown_sequence():
	#move_steering_homestate()
	#time.sleep(5)
	move_throttle_homestate()
	time.sleep(5)

if __name__ == '__main__':
    try:
	rospy.init_node("movement")
	startup=rospy.Publisher("startup", Bool, queue_size=10, latch=True) # this will go to Eric's node to let it know to start up the navigation
	startup.publish(False)
	throttleAdj=rospy.Publisher("throttle_adjust", Float64, queue_size=10) # this goes to the Jetson throttle node
	steeringAdj1=rospy.Publisher("/falcon/outputs/hso_2", falcon_output, queue_size=10,  latch=True) # used to adjust steering angle
        steeringAdj2=rospy.Publisher("/falcon/outputs/hso_4", falcon_output, queue_size=10, latch=True) # used to adjust steering angle
	rospy.Subscriber("vehicle_speed", Float64, update_speed) # Tamkin Node, used to monitor speed to insure it doesn't move too fast
	rospy.Subscriber("target_speed", Twist, adjust_speed) # Navigation Node, used to control speed of ATV
	rospy.Subscriber("brake", Bool, brake) # Naviation Node, used to control braking capabilities
	rospy.Subscriber("steering_pose", Pose, adjust_steering) # Navigation Node, used to control steering
	rospy.Subscriber("kill_switch", Bool, kill_switch) # local message used to engage brake and homestate the throttle
	move_steering_homestate()
	move_throttle_homestate()
	rospy.on_shutdown(shutdown_sequence)
	rospy.spin()
    except rospy.ROSInterruptException:
        pass
