#! /usr/bin/env python
# coding=utf-8

"""
Created on Fri June 7 00:00:00 2019
The code takes the topic of a Realsense camera d435i using in parallel infrared camera and rgb camera.
With a haarcascade classifier the code does person detection and localize the middle point of the object detect. 
Through a linear control for linear velocity and parabolic_linearized control for angular velocity moves a turtlebot3 in order to do Person Tracking.
@author: Anna Boschi
"""

import cv2
import roslib
import rospy
import numpy as np 
import signal
import sys
import time
import os

from std_msgs.msg import *
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from time import sleep
from cv_bridge import CvBridge


HAAR_CASCADE_FOLDER = '/home/charles/catkin_ws/src/jaffle_tracking/src'
HAAR_CASCADE_FILE = 'haarcascade_frontalface_default.xml' # haarcascade_upperbody  or haarcascade_frontalface_default.xml
haarFile = os.path.join(HAAR_CASCADE_FOLDER, HAAR_CASCADE_FILE)

#LINEAR VELOCITY SETTINGS:
m_vel_upperlimit = 1.2					#upper_limit of stop command movement linear velocity [m]
m_vel_lowerlimit = 0.9					#lower_limit of stop command movement linear velocity [m]

def create_straight_line(x1,y1,x2,y2):											#line use for the linear control

	m=(float(y2-y1))/(x2-x1)
	q=y1-(m*x1)
	return (m, q)

#points coordinates of linear velocity -move on:
x1_1 = 1				
y1_1 = 0.13
x2_1 = 3
y2_1 = 0.26
(m1,q1) = create_straight_line(x1_1,y1_1,x2_1,y2_1)

#points coordinates of linear velocity -move back:
x1_2 = 1
y1_2 = -0.13
x2_2 = 0.3
y2_2 = -0.26
(m2,q2) = create_straight_line(x1_2,y1_2,x2_2,y2_2)

#ANGULAR VELOCITY SETTINGS:
threshold1 = 219
threshold2 = 146
threshold3 = 73
yy=1.8/5	#1.8 rad/s is the max angular velocity value

(m3,q3) = create_straight_line(threshold1,5*yy,threshold2,4*yy)
(m4,q4) = create_straight_line(threshold2,4*yy,threshold3,yy)
(m5,q5) = create_straight_line(threshold3,yy,0,0)
(m6,q6) = create_straight_line(0,0,-threshold3,-yy)
(m7,q7) = create_straight_line(-threshold3,-yy,-threshold2,-4*yy)
(m8,q8) = create_straight_line(-threshold2,-4*yy,-threshold1,-5*yy)
print m1,m2,m3,m4,m5,m6,m7,m8,q1,q2,q3,q4,q5,q6,q7,q8

class INIT():
	def __init__(self):
		rospy.init_node('video_streamer',anonymous=True)
		global imageRGB, imageDEPTH_perRGB, imageDEPTH_perINFRA, imageINFRA
		imageRGB = np.zeros((480,640, 3), np.uint8)
		imageDEPTH_perRGB = np.zeros((480,640), np.uint8)	
		imageDEPTH_perINFRA = np.zeros((480,640), np.uint8)
		imageINFRA  = np.zeros((480,640), np.uint8)



class INPUT():
	def __init__(self, cmd_vel='/cmd_vel', cameraRGB ='/camera/color/image_raw', cameraDEPTH_perRGB = '/camera/aligned_depth_to_color/image_raw', cameraDEPTH_perINFRA ='/camera/depth/image_rect_raw', cameraINFRA = '/camera/infra1/image_rect_raw'): 
		#con false '/camera/depth/image_rect_raw'-->classical launcher con true /camera/aligned_depth_to_color/image_raw'--> launcher with "_c"
		self.cmd_vel=cmd_vel
		self.cameraRGB = cameraRGB
		self.cameraDEPTH_perRGB = cameraDEPTH_perRGB
		self.cameraDEPTH_perINFRA = cameraDEPTH_perINFRA
		self.cameraINFRA = cameraINFRA
		self.pub_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
		self.vel = Twist()



	def run(self):
		self.bridge = CvBridge()
		self.imageRGB_sub = rospy.Subscriber(self.cameraRGB, Image, self.RGBimage)
		self.imageDEPTH_perRGB_sub = rospy.Subscriber(self.cameraDEPTH_perRGB, Image, self.DEPTHimage_perRGB)
		self.imageDEPTH_perINFRA_sub = rospy.Subscriber(self.cameraDEPTH_perINFRA, Image, self.DEPTHimage_perINFRA)
		self.imageINFRA_sub = rospy.Subscriber(self.cameraINFRA, Image, self.INFRAREDimage)
		#rospy.spin()

	def Exit_gracefully(self,signal, frame):
		signal.signal(signal.SIGINT, original_sigint)
		try:
			if raw_input("\Really quit? (y/n)> ").lower().startswith('y'):
				sys.exit(1)
		except KeyboardInterrupt:
			print ("OK. Quitting...")
			sys.exit(1)
		signal.signal(signal.SIGINT, Exit_gracefullty)

	def RGBimage(self,data):													#RGB image callback
		global imageRGB
		imageRGB = self.bridge.imgmsg_to_cv2(data, "bgr8")

	def DEPTHimage_perRGB(self,data):											#DEPTH for RGB image callback
		global imageDEPTH_perRGB
		imageDEPTH_perRGB  = self.bridge.imgmsg_to_cv2(data, "16UC1")
		imageDEPTH_perRGB = np.array(imageDEPTH_perRGB, dtype=np.float32)

	def DEPTHimage_perINFRA(self,data):											#DEPTH for INFRA image callback
		global imageDEPTH_perINFRA
		imageDEPTH_perINFRA  = self.bridge.imgmsg_to_cv2(data, "16UC1")
		imageDEPTH_perINFRA = np.array(imageDEPTH_perINFRA, dtype=np.float32)

	def INFRAREDimage(self,data):												#INFRA image callback
		global imageINFRA
		imageINFRA = self.bridge.imgmsg_to_cv2(data, "16UC1")
		imageINFRA = np.array(imageINFRA, dtype=np.uint8)


	def Velocity_Linear(self, data, min_vel= -0.26, max_vel=0.26):	#computing the linear velocity in linear distribution

		depth_in_m = data/1000

		#CONTROL LINEAR VELOCITY:
		if depth_in_m > m_vel_upperlimit: 														#MOVE ON 
			v_linear_x = (depth_in_m *m1) + q1

		elif (depth_in_m <= m_vel_upperlimit and depth_in_m > m_vel_lowerlimit):			#STOP
			v_linear_x = 0

		elif depth_in_m == 0:																		#STOP
			v_linear_x = 0

		else:																						#MOVE BACK
			v_linear_x = (depth_in_m * m2) + q2

		return v_linear_x 

	def Velocity_Angular(self, data, max_vel=1.8, min_vel= -1.8):	#computing the angular velocity in parabolic distribution

		dx_pixel = data
		
		#CONTROL ANGULAR VELOCITY:
		#if dx_pixel > 0:									 #	LEFT SIDE
		#	v_angular_theta = max_vel*dx_pixel*dx_pixel/(320.0*320.0)

		#else:												#	RIGTH SIDE
		#	v_angular_theta = min_vel*dx_pixel*dx_pixel/(320.0*320.0)

		#return v_angular_theta

		if dx_pixel > threshold1:
			v_angular_theta = 0

		elif(dx_pixel > threshold2 and dx_pixel <= threshold1):
			v_angular_theta = (dx_pixel*m3) + q3

		elif(dx_pixel > threshold3 and dx_pixel <= threshold2):
			v_angular_theta = (dx_pixel*m4) + q4

		elif(dx_pixel > 0 and dx_pixel <= threshold3):
			v_angular_theta = (dx_pixel*m5) + q5
		
		elif(dx_pixel <= 0 and dx_pixel > -threshold3):
			v_angular_theta = (dx_pixel*m6) + q6
		
		elif(dx_pixel <= -threshold3 and dx_pixel >-threshold2):
			v_angular_theta = (dx_pixel*m7) + q7
		
		elif(dx_pixel <= -threshold2 and dx_pixel >-threshold1):
			v_angular_theta = (dx_pixel*m8) + q8
		
		elif dx_pixel <= -threshold1:
			v_angular_theta = 0  
		
		return v_angular_theta


	def detection(self):

		#body Haarcascade:
		#bodyPath = "/home/annaboschi/catkin_ws/src/video_streamer/src/haarcascade_frontalface_default.xml"
		bodyPath = haarFile
		bodyCascade = cv2.CascadeClassifier(bodyPath)	
		if bodyCascade is not None:
			print("bodycascade correctly uploaded")
		else:
			return

		#function Classifier:
		counter=0
		notfound=0
		global P_saved, dx_pixel_saved
		P_saved = 0
		dx_pixel_saved = 0

		while True:
#			start = time.time()
			frameINFRA = imageINFRA.copy()
			frameRGB = imageRGB.copy()

			
			gray = cv2.cvtColor(frameRGB, cv2.COLOR_BGR2GRAY)
			gray = cv2.equalizeHist(gray)

			try:
				#Haarcascade_detection_setting
				bodyRGB = bodyCascade.detectMultiScale(
					gray,
					scaleFactor=1.3,
					minNeighbors=4,
					minSize=(30,30),	#set 20,20 for upperbody / set 30,30 for test with frontal face
					flags=cv2.CASCADE_SCALE_IMAGE
					)

				bodyINFRA = bodyCascade.detectMultiScale(
					frameINFRA,
					scaleFactor=1.3,
					minNeighbors=4,
					minSize=(30, 30),	#set 20,20 for upperbody / set 30,30 for test with frontal face
					flags=cv2.CASCADE_SCALE_IMAGE
					)
			except:
				#No Haarcascade_detected
				bodyINFRA = []
				bodyRGB = []


			# Draw a rectangle around the body:
			cfXcol = 320-1 #centre frame X 0-847
			cfYrig = 240-1 #centre frame Y 0-639
			
			#INFRA ONLY DETECTED: generally in depth or darckness conditions
			if (len(bodyRGB) == 0  and len(bodyINFRA) != 0):
				counter=0
				for (x, y, w, h) in bodyINFRA:
					cv2.rectangle(frameINFRA, (x, y), (x+w, y+h), (255, 255, 255), 2)
					#print('frame info position body: x= %d, y= %d, w=%d, h=%d, and the distance from centre of the video-frame: dX=%d, dY= %d' %( x+w/2, y+h/2, w, h, cfXcol-(x+w/2), cfYrig-(y+h/2) ))
					roi_gray = frameINFRA[y:y+h, x:x+w]
					roi_color = frameINFRA[y:y+h, x:x+w]
					print "Found body INFRA!"


					dx_pixel=(cfXcol - (x+w/2))
					dx_pixel_saved = dx_pixel
					P=imageDEPTH_perINFRA[(y+h/2),(x+w/2)]
					P_saved = P
					print("The mm distance of the body_position center is %f" %P)

					linear_V=self.Velocity_Linear(P)
					angular_V=self.Velocity_Angular(dx_pixel)
					self.vel.linear.x=linear_V
					self.vel.angular.z=angular_V
					self.pub_vel.publish(self.vel)
					

			#RGB PRIORITY ALSO ON INFRA in backlight or other general conditions
			elif (len(bodyRGB)!=0):
				counter=0
				for	(x, y, w, h) in bodyRGB:
					cv2.rectangle(frameRGB, (x, y), (x+w, y+h), (0, 255, 0), 2)
					#print('frame info position body: x= %d, y= %d, w=%d, h=%d, and the distance from centre of the video-frame: dX=%d, dY= %d' %( x+w/2, y+h/2, w, h, cfXcol-(x+w/2), cfYrig-(y+h/2) ))
					roi_gray = gray[y:y+h, x:x+w]
					roi_color = frameRGB[y:y+h, x:x+w]
					print "Found body RGB!"
					
					
					dx_pixel=(cfXcol - (x+w/2))
					dx_pixel_saved = dx_pixel
					P=imageDEPTH_perRGB[(y+h/2),(x+w/2)]
					P_saved = P
					print("The mm distance of the body_position center is %f" %P)

					linear_V=self.Velocity_Linear(P)
					angular_V=self.Velocity_Angular(dx_pixel)
					self.vel.linear.x=linear_V
					self.vel.angular.z=angular_V
					self.pub_vel.publish(self.vel)

			#NOTHING DETECTED: if nothing is found immediatly at the beginning or in 100 times counted after a detection is happened
						# the turtlebot3 stops
			else:
				notfound = notfound+1
				counter = counter+1
				linear_V=self.Velocity_Linear(P_saved)
				angular_V=self.Velocity_Angular(dx_pixel_saved)
				self.vel.linear.x=linear_V
				self.vel.angular.z=angular_V
				self.pub_vel.publish(self.vel)
				print('numero non trovati= %d' %notfound)
				print('numero contati= %d' %counter)
				if (counter ==5):
					dx_pixel_saved = 0

				if (counter == 15):
					P_saved=0
					dx_pixel_saved=0 
					counter = 0
					self.vel.linear.x=0
					self.vel.angular.z=0
					self.pub_vel.publish(self.vel)



			#Normalization of the image in order to obtain a better published resolution:
			cv2.normalize(imageDEPTH_perRGB, imageDEPTH_perRGB, 0,255, cv2.NORM_MINMAX)
			cv2.normalize(imageDEPTH_perINFRA, imageDEPTH_perINFRA, 0,255, cv2.NORM_MINMAX)
			cv2.normalize(imageINFRA, imageINFRA, 0,255, cv2.NORM_MINMAX)


			#Showing_Window_Frame of each camera used:
			#cv2.imshow('VideoINFRA_con_classificatore', frameINFRA)
			#cv2.imshow('VideoBGR_con_classificatore', frameRGB)
			#cv2.imshow('VideoDepth_perINFRA', imageDEPTH_perINFRA)
			#cv2.imshow('VideoDepth_perRGB', imageDEPTH_perRGB)
			#cv2.imshow('VideoINFRA', imageINFRA)



			#stop the program and stop the robot:
			if cv2.waitKey(1) & 0xFF == ord('q'):
				self.vel.linear.x=0
				self.vel.angular.z=0
				self.pub_vel.publish(self.vel)
				break
			#print time.time()-start
		# When everything is done, release the capture:
		cv2.destroyAllWindows()



if __name__ == '__main__':

	init = INIT()

	i = INPUT()
	i.run()

	original_sigint = signal.getsignal(signal.SIGINT)
	signal.signal(signal.SIGINT, i.Exit_gracefully)
	try:
			i.detection()
	except rospy.ROSInterruptException:
		print("ERROR")
		exit
