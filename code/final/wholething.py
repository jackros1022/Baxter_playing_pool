## Authors: 1.Shiva Karthik Reddy Koyya 2. VishnuNandan 
## The project is designed to make "BAXTER- A robot by rethink Robotics" play the game of pool.
## The entire program is divided into 5 major steps.

## Hadware Requirments 1. 2 Balls  we used Red and White for any other colour the code has to be modified; 2. Cue stick as end effector.

from __future__ import division
from math import sin,cos, tan, atan2, asin, acos,pow,radians,degrees
import cv2
from sensor_msgs.msg import Image
import rospy
import baxter_interface
from baxter_interface import Limb
from cv_bridge import CvBridge
import time
import numpy as np
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import Header

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)
import argparse
import struct
import math
import sys
rospy.init_node("Detekcia")
import time
## step one detects both the red and white ball's center and finds the desired orientation of the hand for Short 
def Step1():

	# starting Baxters's Head 
	camera = baxter_interface.CameraController('head_camera')
	limb_r = Limb('right')
	camera.resolution = (1280, 800)
	camera.exposure = 100
	camera.gain = 15
	camera.fps = 20
	camera.open()

	bridge = CvBridge()

	lol_1 = 0
	cam = 0
	objectFound = 0



	def euler2quat(roll,pitch,yaw):
		q0=cos(roll/2)*cos(pitch/2)*cos(yaw/2)+sin(roll/2)*sin(pitch/2)*sin(yaw/2)
		q1=sin(roll/2)*cos(pitch/2)*cos(yaw/2)-cos(roll/2)*sin(pitch/2)*sin(yaw/2)	
		q2=cos(roll/2)*sin(pitch/2)*cos(yaw/2)+sin(roll/2)*cos(pitch/2)*sin(yaw/2)
		q3=cos(roll/2)*cos(pitch/2)*sin(yaw/2)-sin(roll/2)*sin(pitch/2)*cos(yaw/2)
		return q0,q1,q2,q3 
	

	def changeEG(exposure, gain):
		if camera.exposure != exposure:
			camera.exposure = exposure
		if camera.gain != gain:
			camera.gain = gain


	def createPhoto():
		try:
			msg=rospy.wait_for_message('/cameras/head_camera/image', Image)
		except rospy.exceptions:
			print "error"    
		img = bridge.imgmsg_to_cv2(msg, "bgr8")
		frame = img
		cam = frame[500:700, 500:800]
		exposure = cv2.getTrackbarPos('Exposure', 'cam')
		gain = cv2.getTrackbarPos('Gain', 'cam')
		changeEG(exposure, gain)
		return cam






	blueLower = (0,120, 40)
	blueUpper = (255, 255, 255)
	greenLower = (10,10,50)
	greenUpper = (30,255,255)
	x=0
	y=0
	radius=0
	def Tracking(frame):


	  for x in range(0, 10):
	    blurred = cv2.GaussianBlur(frame, (11, 11), 0)
	    hsv=cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
	 

	    mask = cv2.inRange(hsv, blueLower, blueUpper)
	    mask = cv2.erode(mask, None, iterations=2)
	    mask = cv2.dilate(mask, None, iterations=2)
	    # find contours in the mask and initialize the current
	    # (x, y) center of the ball
	    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
		cv2.CHAIN_APPROX_SIMPLE)[-2]
	    center = None
	    
	    
	    # only proceed if at least one contour was found
	    if len(cnts) > 0:
		    
		# find the largest contour in the mask, then use
		# it to compute the minimum enclosing circle and
		# centroid
		    c = max(cnts, key=cv2.contourArea)
		    ((x, y), radius) = cv2.minEnclosingCircle(c)

		    

		    M = cv2.moments(c)

		    center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

	   
	    return x,y,radius,center	


	def Tracking1(frame):


	  for x in range(0, 10): 
	    blurred = cv2.GaussianBlur(frame, (11, 11), 0)
	    hsv=cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
	 

	    mask = cv2.inRange(hsv, greenLower, greenUpper)
	    mask = cv2.erode(mask, None, iterations=2)
	    mask = cv2.dilate(mask, None, iterations=2)
	    # find contours in the mask and initialize the current
	    # (x, y) center of the ball
	    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
		cv2.CHAIN_APPROX_SIMPLE)[-2]
	    center = None
	   
	    # only proceed if at least one contour was found
	    if len(cnts) > 0:
		    
		# find the largest contour in the mask, then use
		# it to compute the minimum enclosing circle and
		# centroid
		    c = max(cnts, key=cv2.contourArea)
		    ((x, y), radius) = cv2.minEnclosingCircle(c)
		   
	   
		    M = cv2.moments(c)

		    center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

	   
	  return x,y,radius,center	
		


	def drawObject(x, y, radius , center, frame):

		cv2.circle(frame, (int(x), int(y)), int(radius),
		        (0, 255, 255), 2)
		cv2.circle(frame, center, 5, (0, 0, 255), -1)
		return frame

	def drawlinme(p1,p2,frame):

		cv2.line(frame, p1, p2,( 255, 255, 0 ), 1, 8, 0) 
		return frame





	#limb_r.move_to_neutral()
	for x in range(1, 40):	
		cam = createPhoto()
		#cv2.imshow('cam', cam)
		#cv2.waitKey(1)
	
		a, b , radius ,center=Tracking(cam)
		#cv2.waitKey(1)
		drawObject(a, b, radius,center,cam)
		x, y , radius ,center1=Tracking1(cam)
		#cv2.waitKey(1)
		drawObject(x, y, radius,center,cam)

		drawlinme(center,center1,cam)
	
		g=b-y
		h=a-x
		val=h/g
		valr=(degrees(val)/2)-10
		r=178+valr
		ra=radians(r)
		#inital orient (178,-93,5)
	
		x,y,z,w=euler2quat(ra,-1.62316,0.0872665)
		


	
	return x,y,z,w,radians(valr),cam

# move the baxter's hand to chack the Que and come to a home position.  In this the chacking action can be removed without any effect on the main program. 


def Step2():
	limb1 = baxter_interface.Limb('left')
	limb = baxter_interface.Limb('right')
	limb.set_joint_position_speed(1.0)

	

	wave_1 = {'right_s0': 0.7144515511413575, 'right_s1': -0.6699661083435059, 'right_w0': -1.740301201867676, 'right_w1': -1.570796325, 'right_w2': 0.018791264630126956, 'right_e0': 0.06250971703491211, 'right_e1': 0.704097180834961}
	wave_2={'left_w0': 3.03383049977417, 'left_w1': -1.5079031127685547, 'left_w2': -0.21207284368286133, 'left_e0': 0.15301458341674806, 'left_e1': 0.6707330987365723, 'left_s0': -0.9295923563964844, 'left_s1': -0.7221214550720215}

	wave_3={'right_s0': 0.7857816576965333, 'right_s1': -0.1955825502319336, 'right_w0': -2.869694555657959, 'right_w1': 1.609912835046387, 'right_w2': -3.0591411827453614, 'right_e0': -1.9665633678222658, 'right_e1': -0.04985437554931641}

	wave_4={'right_s0': 0.7857816576965333, 'right_s1': -0.1955825502319336, 'right_w0': -2.869694555657959, 'right_w1': 1.609912835046387, 'right_w2': 3.0591411827453614, 'right_e0': -1.9665633678222658, 'right_e1': -0.04985437554931641}


	limb.move_to_joint_positions(wave_1)
	limb1.move_to_joint_positions(wave_2)
	t_end = time.time() + 5
	
	limb.move_to_joint_positions(wave_3)
	limb.move_to_joint_positions(wave_4)

	#wave_8={'right_s0': 0.6396699878173828, 'right_s1': -0.49202433715209964, 'right_w0': -1.7138400333068848, 'right_w1': 1.255563273449707, 'right_w2': -0.6066894009155274, 'right_e0': -1.7429856682434084, 'right_e1': 1.0492428577148438}

	#limb.move_to_joint_positions(wave_8)

	wave_7 = {'right_s0': 0.9639937033411371, 'right_s1': -0.28695082759045815, 'right_w0': -2.959, 'right_w1': 1.5803892485976359, 'right_w2': -2.9352389229115348, 'right_e0': -0.14089182454059407, 'right_e1': 1.8593947307842924}
	limb.move_to_joint_positions(wave_7)


# find the inverse kinematic soloution to achive the desired orientation as found in step 1 and move the hand to that orientation.


def Step3(q1,q2,q3,q4):

	objectFound=True
	
	# create an instance of baxter_interface's Limb class
	limb = baxter_interface.Limb('right')

	angles=limb.joint_angles()




	def ik_test_Luky(limb, poza ):
		ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
		iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
		ikreq = SolvePositionIKRequest()
		hdr = Header(stamp=rospy.Time.now(), frame_id='base')
		print("its happening")
		poses = {
			'right': PoseStamped(
				header=hdr,
				pose=Pose(
					position=Point(
						x= poza['position'].x , 
						y= poza['position'].y , #-0.25, #-0.2837,
						z= poza['position'].z 
					),

	     
	   

	

					orientation=Quaternion(
						x= q1 ,
						y= q2  ,
						z= q3,
						w= q4,
					),
				),
			),
		}

		ikreq.pose_stamp.append(poses[limb])
		try:
			rospy.wait_for_service(ns, 5.0)
			resp = iksvc(ikreq)
		except (rospy.ServiceException, rospy.ROSException), e:
			rospy.logerr("Service call failed: %s" % (e,))
			return 1

		# Check if result valid, and type of seed ultimately used to get solution
		# convert rospy's string representation of uint8[]'s to int's
		resp_seeds = struct.unpack('<%dB' % len(resp.result_type),
								   resp.result_type)
		if (resp_seeds[0] != resp.RESULT_INVALID):
			seed_str = {
						ikreq.SEED_USER: 'User Provided Seed',
						ikreq.SEED_CURRENT: 'Current Joint Angles',
						ikreq.SEED_NS_MAP: 'Nullspace Setpoints',
					   }.get(resp_seeds[0], 'None')
			print("\nSUCCESS - Valid Joint Solution Found from Seed Type: %s" %
				  (seed_str,))
			# Format solution into Limb API-compatible dictionary
			limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
			#print "\nIK Joint Solution:\n", limb_joints
			print "------------------"
			#print "Response Message:\n", resp
		else:
			print("INVALID POSE - No Valid Joint Solution Found.")

		return limb_joints






	def move_to():
	
		for x in range(1, 9):	
	
			poza = limb.endpoint_pose()
			wave_1 = ik_test_Luky('right', poza)
			play(wave_1)
			#wave_2 = ik_test_Luky('right', poza,b)
	
	


	def play(wave_1):
	
		    limb.move_to_joint_positions(wave_1,timeout=15,threshold=0.05)
		    
		   
	
	
			
	

	try:
		move_to()
	
	
	except:
		pass
	        
# Maintaing the Orientation of the hand Track the center of the ball and align the end effector "cue Stick" to the center of the ball.			
def Step4(q1,q2,q3,q4):


	camera = baxter_interface.CameraController('right_hand_camera')
	limb_r = Limb('right')
	camera.resolution = (960, 600)
	camera.exposure = 100
	camera.gain = 15
	camera.fps = 20
	camera.open()

	bridge = CvBridge()

	lol_1 = 0
	cam = 0
	objectFound = 0

	H_MIN = 0;
	H_MAX = 256;
	S_MIN = 0;
	S_MAX = 256;
	V_MIN = 0;
	V_MAX = 256;

	MAX_NUM_OBJECTS=50
	FRAME_WIDTH = 960
	FRAME_HEIGHT = 600
	MIN_OBJECT_AREA = 20*20
	MAX_OBJECT_AREA = FRAME_HEIGHT*FRAME_WIDTH*8

	def createWindows():
		cv2.namedWindow('cam', 1)
	

	def nothing(*arg):
	    pass

	def createTrackbars():    
		cv2.createTrackbar('Exposure', 'cam', 100, 100, nothing)
		cv2.createTrackbar('Gain', 'cam', 15, 79, nothing)
		cv2.createTrackbar('H_MIN', 'HSV trackbars', 10, 500, nothing)
		cv2.createTrackbar('H_MAX', 'HSV trackbars', 30, 500, nothing)
		cv2.createTrackbar('S_MIN', 'HSV trackbars', 50, 500, nothing)
		cv2.createTrackbar('S_MAX', 'HSV trackbars', 500, 500, nothing)
		cv2.createTrackbar('V_MIN', 'HSV trackbars', 50, 500, nothing)
		cv2.createTrackbar('V_MAX', 'HSV trackbars', 500, 500, nothing)

	def changeEG(exposure, gain):
		if camera.exposure != exposure:
			camera.exposure = exposure
		if camera.gain != gain:
			camera.gain = gain

	#def loadPart():
	#	 lol_1 = cv2.imread('/home/baxter/ros_ws/zaznam/ lol.jpg', 1)
	#	print  lol_1
	


	def drawObject(x, y, frame):
		cv2.circle(frame, (int(x), int(y)), 25, (0,255,0), 2)	
		if y-25>0:
			cv2.line(frame,(int(x), int(y)),(int(x), int(y)-20),(0,255,0),2)
		else :
			cv2.line(frame,(int(x), int(y)),(int(x), 0),(0,255,0),2)	
		if y+25<FRAME_HEIGHT:
			cv2.line(frame,(int(x), int(y)),(int(x), int(y)+20),(0,255,0),2)
		else :
			cv2.line(frame,(int(x), int(y)),(int(x), FRAME_HEIGHT),(0,255,0),2)	
		if x-25>0:
			cv2.line(frame,(int(x), int(y)),(int(x)-20, int(y)),(0,255,0),2)
		else :
			cv2.line(frame,(int(x), int(y)),(0, int(y)),(0,255,0),2)	
		if x+25<FRAME_WIDTH:
			cv2.line(frame,(int(x), int(y)),(int(x)+20, int(y)),(0,255,0),2)
		else :
			cv2.line(frame,(int(x), int(y)),(FRAME_WIDTH, int(y)),(0,255,0),2)
		return frame

	greenLower = (10,50,50)
	greenUpper = (30,255,255)
	blueLower = (0,120, 40)
	blueUpper = (255, 255, 255)	
	
	def trackObject(frame):
	    blurred = cv2.GaussianBlur(frame, (11, 11), 0)
	    hsv=cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
	 

	    mask = cv2.inRange(hsv, blueLower, blueUpper)
	    mask = cv2.erode(mask, None, iterations=2)
	    mask = cv2.dilate(mask, None, iterations=2)
	    # find contours in the mask and initialize the current
	    # (x, y) center of the ball
	    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
		cv2.CHAIN_APPROX_SIMPLE)[-2]
	    center = None
	    
	    
	    # only proceed if at least one contour was found
	    if len(cnts) > 0:
		    
		# find the largest contour in the mask, then use
		# it to compute the minimum enclosing circle and
		# centroid
		    c = max(cnts, key=cv2.contourArea)
		    ((x, y), radius) = cv2.minEnclosingCircle(c)
		   

		    M = cv2.moments(c)

		    center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
		    objectFound=True
	    else :
		    objectFound=False

	    return x, y, objectFound,frame
	
	def createPhoto():
		try:
			msg=rospy.wait_for_message('/cameras/right_hand_camera/image', Image)
		except rospy.exceptions:
			print "error"    
		img = bridge.imgmsg_to_cv2(msg, "bgr8")
		cam = img
		exposure = cv2.getTrackbarPos('Exposure', 'cam')
		gain = cv2.getTrackbarPos('Gain', 'cam')
		changeEG(exposure, gain)
		return cam
		

	
	#####
	def prepocet(x, y):
		#cam = (480, 300)
		#object = (x, y)
		# 960px*600px = (a*b*h) 46cm*27cm*19cm
		x_c = 960/14.42	#px/cm				
		y_c = 600/15.34	#px/cm
		def_x_cam = 508	#stred 480################################################
		def_y_cam = 205	#stred 300
		cam = {'x': def_x_cam, 'y': def_y_cam}			#px
		object = {'x': x, 'y': y}				#px
		res_x = cam['x'] - object['x']
		res_y = cam['y'] - object['y']
		if res_x < 0:
			res_x = res_x * (-1)
		if res_y < 0:
			res_y = res_y * (-1)
		res = {'x': res_x, 'y': res_y}			#px
		pos_x = (res['x'] / x_c) /100#I dont know 
		pos_y = (res['y'] / y_c) /100
		if x > def_x_cam and y < def_y_cam:
			pos_x = pos_x * (-1)
		if x < def_x_cam and y > def_y_cam:
			pos_y = pos_y * (-1)
		if x > def_x_cam and y > def_y_cam:
			pos_x = pos_x * (-1)
			pos_y = pos_y * (-1)
		pos = {'y': pos_x, 'x': pos_y}	#cm baxter ma obratene x, y#i dont know 
		return pos

	def ik_test_Luky(limb, poza, x, y):
		ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
		iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
		ikreq = SolvePositionIKRequest()
		hdr = Header(stamp=rospy.Time.now(), frame_id='base')
		poses = {
			'right': PoseStamped(
				header=hdr,
				pose=Pose(
					position=Point(
						x= 0.7, #0.683,
						y= poza['position'].y + y, #-0.25, #-0.2837,
						z=  0.0703770153796
	 #poza['position'].x + x, #0.3 - 19cm; 0.5 - 38cm
					),
					orientation=Quaternion(
						x= q1  ,
						y= q2  ,
						z= q3,
						w= q4 ,
					),
				),
			),
		}

		ikreq.pose_stamp.append(poses[limb])
		try:
			rospy.wait_for_service(ns, 5.0)
			resp = iksvc(ikreq)
		except (rospy.ServiceException, rospy.ROSException), e:
			rospy.logerr("Service call failed: %s" % (e,))
			return 1

		# Check if result valid, and type of seed ultimately used to get solution
		# convert rospy's string representation of uint8[]'s to int's
		resp_seeds = struct.unpack('<%dB' % len(resp.result_type),
								   resp.result_type)
		if (resp_seeds[0] != resp.RESULT_INVALID):
			seed_str = {
						ikreq.SEED_USER: 'User Provided Seed',
						ikreq.SEED_CURRENT: 'Current Joint Angles',
						ikreq.SEED_NS_MAP: 'Nullspace Setpoints',
					   }.get(resp_seeds[0], 'None')
			print("\nSUCCESS - Valid Joint Solution Found from Seed Type: %s" %
				  (seed_str,))
			# Format solution into Limb API-compatible dictionary
			limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
			#print "\nIK Joint Solution:\n", limb_joints
			print "------------------"
			#print "Response Message:\n", resp
		else:
			print("INVALID POSE - No Valid Joint Solution Found.")

		return limb_joints
	
	def move_to(x, y):
		pos = prepocet(x, y)
		poza = limb_r.endpoint_pose()
		pozicie = ik_test_Luky('right', poza, pos['x'], pos['y'])
		limb_r.move_to_joint_positions(pozicie)
	#####

	
		
	createWindows()

	t_end = time.time() + 45
	while time.time() < t_end:
		cam = createPhoto()
		
		x, y, objectFound, tracked = trackObject(cam)
		print x, y
		
		if objectFound :
			drawObject(x, y, cam)
			try:
				move_to(x, y)
			except:
				pass
		cv2.imshow('cam', cam)
		cv2.waitKey(1)		
		
		
# move the hand in small differential motion to make the short.	
	
def Step5(q1,q2,q3,q4,s):

	limb = baxter_interface.Limb('right')

	angles=limb.joint_angles()



	def ik_test_Luky(limb, poza ):
		ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
		iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
		ikreq = SolvePositionIKRequest()
		hdr = Header(stamp=rospy.Time.now(), frame_id='base')
		
		poses = {
			'right': PoseStamped(
				header=hdr,
				pose=Pose(
					position=Point(
						x= poza['position'].x+0.2 , 
						y= poza['position'].y +(0.2*tan(s)), #-0.25, #-0.2837,
						z= poza['position'].z 
					),
					orientation=Quaternion(
						x= q1 ,
						y= q2  ,
						z= q3,
						w= q4,
					),
				),
			),
		}

		ikreq.pose_stamp.append(poses[limb])
		try:
			rospy.wait_for_service(ns, 5.0)
			resp = iksvc(ikreq)
		except (rospy.ServiceException, rospy.ROSException), e:
			rospy.logerr("Service call failed: %s" % (e,))
			return 1

		# Check if result valid, and type of seed ultimately used to get solution
		# convert rospy's string representation of uint8[]'s to int's
		resp_seeds = struct.unpack('<%dB' % len(resp.result_type),
								   resp.result_type)
		if (resp_seeds[0] != resp.RESULT_INVALID):
			seed_str = {
						ikreq.SEED_USER: 'User Provided Seed',
						ikreq.SEED_CURRENT: 'Current Joint Angles',
						ikreq.SEED_NS_MAP: 'Nullspace Setpoints',
					   }.get(resp_seeds[0], 'None')
			print("\nSUCCESS - Valid Joint Solution Found from Seed Type: %s" %
				  (seed_str,))
			# Format solution into Limb API-compatible dictionary
			limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
			#print "\nIK Joint Solution:\n", limb_joints
			print "------------------"
			#print "Response Message:\n", resp
		else:
			print("INVALID POSE - No Valid Joint Solution Found.")

		return limb_joints






	def move_to():
	
	
	
			poza = limb.endpoint_pose()
			wave_1 = ik_test_Luky('right', poza)
			play(wave_1)
			#wave_2 = ik_test_Luky('right', poza,b)
	
	


	def play(wave_1):
	
		    limb.move_to_joint_positions(wave_1)
		    
		   
	
	
			
	
		
	try:
		move_to()
	
	
	except:
		pass

			

	


def main():		
	
	

	val = input("To start the system enter 1: ")
	a=val


	if a==1:	
		print "you entered", val
		x,y,z,w,slope,cam=Step1()		
		print x,y,z,w,slope
	

		t_end = time.time() + 6
		while time.time() < t_end:
			cv2.imshow('cam', cam)
			cv2.waitKey(1)
	
		
		Step2()
		t_end = time.time() + 5
		while time.time() < t_end:
			shiva=1
		Step3(x,y,z,w)
		
		Step4(x,y,z,w)
		
		t_end = time.time() + 3
		while time.time() < t_end:
			shiva=2
		Step5(x,y,z,w,slope)
			
		        
			

if __name__ == "__main__":
	main()
