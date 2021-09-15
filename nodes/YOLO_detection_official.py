#! /usr/bin/env python
# coding=utf-8

"""
Created on Thur June 20 00:00:00 2019
The code takes the topic of a Realsense camera d435i using rgb camera and its aligned depth.
Subscribing to the darknet network topics it is possible to find the number of the detected object and their respectively bounding boxes.
Using this informations the code localizes the middle point of the object detected.
Through a linear control for linear velocity and parabolic control for angular velocity moves a jaffle in order to do Person Follow.
In case of multiple detections the robot stops for a preset time and then restarts the detection of the person, so the complete tracking iter.
When the person is lost the robot keeps moving considering the presence of the person in the previous position and after a preset time it stops
if nothing is detected anymore.

@author: Anna Boschi
"""

import cv2
import roslib
import rospy
import roscpp
import numpy as np
import signal
import sys
import time
import os
import boost
import threading

from std_msgs.msg import *
from darknet_ros_msgs.msg import *
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from time import sleep


m_vel_upperlimit = 1.9					#upper_limit of stop command movement linear velocity [m]
m_vel_lowerlimit = 1.7					#lower_limit of stop command movement linear velocity [m]

def create_straight_line(x1,y1,x2,y2):											#line use for the linear control

		m=(float(y2-y1))/(x2-x1)
		q=y1-(m*x1)
		return (m, q)

#points coordinates of linear velocity -move on:
x1_1 = 1
y1_1 = 0.23
x2_1 = 3
y2_1 = 0.26
(m1,q1) = create_straight_line(x1_1, y1_1, x2_1, y2_1)
#points coordinates of linear velocity -move back:
x1_2 = 1
y1_2 = -0.23
x2_2 = 0.3
y2_2 = -0.26
(m2,q2) = create_straight_line(x1_2, y1_2, x2_2, y2_2)

class INIT():
	def __init__(self):
		rospy.init_node('yolo_detection',anonymous=True)
		global imageDEPTH_perRGB, classname, probability, center_x, center_y, detection
		imageDEPTH_perRGB = np.zeros((480, 640), np.uint8)
		classname, probability, center_x, center_y, detection = '', 0.0, 0, 0, 0

class INPUT(threading.Thread):
	def __init__(self, cmd_vel='/cmd_vel', bounding_boxes = '/darknet_ros/bounding_boxes', cameraDEPTH_perRGB = '/camera/aligned_depth_to_color/image_raw', num_detection = '/darknet_ros/found_object'):
		threading.Thread.__init__(self)
		self.cmd_vel = cmd_vel
		self.bounding_boxes = bounding_boxes
		self.cameraDEPTH_perRGB = cameraDEPTH_perRGB
		self.num_detection = num_detection
		self.pub_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
		self.vel = Twist()

	def run(self):
		self.bridge=CvBridge()
		self.bounding_boxes_sub = rospy.Subscriber(self.bounding_boxes, BoundingBoxes, self.bbox)
		self.imageDEPTH_perRGB_sub = rospy.Subscriber(self.cameraDEPTH_perRGB, Image, self.DEPTHimage_perRGB)
		self.num_detection_sub = rospy.Subscriber(self.num_detection, Int8, self.found_object)
		rospy.spin()	#delete

	def Exit_gracefully(self,signal, frame):
		signal.signal(signal.SIGINT, original_sigint)
		try:
			if raw_input("\Really quit? (y/n)> ").lower().startswith('y'):
				sys.exit(1)
		except KeyboardInterrupt:
			print ("OK. Quitting...")
			sys.exit(1)
		signal.signal(signal.SIGINT, Exit_gracefullty)


	def DEPTHimage_perRGB(self,data):								#DEPTH for RGB image callback
		global imageDEPTH_perRGB
		imageDEPTH_perRGB  = self.bridge.imgmsg_to_cv2(data, "16UC1")
		imageDEPTH_perRGB = np.array(imageDEPTH_perRGB, dtype=np.float32)

	def bbox(self, data):											#information about the detected object(s) and bounding boxes
		global classname, probability, center_x, center_y
		values = data.bounding_boxes
		for data in values:
			classname = data.Class
			probability = data.probability
			bb_xmin = data.xmin
			bb_ymin = data.ymin
			bb_xmax = data.xmax
			bb_ymax = data.ymax
			print classname, probability, bb_xmin, bb_ymin, bb_xmax, bb_ymax
			#print type(classname)
			w = bb_xmax - bb_xmin
			h = bb_ymax - bb_ymin
			center_x = bb_xmin + w/2
			center_y = bb_ymin + h/2
			#print center_x, center_y

	def found_object(self, data):									#number of object found from the network
		global detection
		detection = data.data
		print ("person found %d" %detection)


	def Velocity_Linear(self, data, min_vel= -0.26, max_vel=0.26):	#computing the linear velocity in linear distribution

		depth_in_m = data/1000

		#CONTROL LINEAR VELOCITY:
		if depth_in_m > m_vel_upperlimit: 															#MOVE ON
			#(m1,q1) = self.create_straight_line(x1_1,y1_1,x2_1,y2_1)#(1,0.13,2.5,0.26)
			v_linear_x = (depth_in_m *m1) + q1

		elif (depth_in_m <= m_vel_upperlimit and depth_in_m > m_vel_lowerlimit):					#STOP
			v_linear_x = 0

		elif depth_in_m == 0:																		#STOP
			v_linear_x = 0

		else:																						#MOVE BACK
			#(m2,q2) = self.create_straight_line(x1_2,y1_2,x2_2,y2_2)#(1, -0.13, 0.3,-0.26)
			v_linear_x = (depth_in_m * m2) + q2

		return v_linear_x

	def Velocity_Angular(self, data, max_vel=1.8, min_vel= -1.8):	#computing the angular velocity in parabolic distribution

		dx_pixel = data

		#CONTROL ANGULAR VELOCITY:
		if dx_pixel > 0:									 #	LEFT SIDE
			v_angular_theta = max_vel*dx_pixel*dx_pixel/(320.0*320.0)

		elif dx_pixel == 0:
			v_angular_theta = 0

		else:												#	RIGTH SIDE
			v_angular_theta = min_vel*dx_pixel*dx_pixel/(320.0*320.0)

		return v_angular_theta


	def control(self):												#control movement regulation

		global P_saved, dx_pixel_saved, counter, notfound, detection, moreperson
		counter = 0
		notfound = 0
		moreperson = 0
		cfXcol = 320-1
		P_saved, dx_pixel_saved = 0, 0
		start = time.time()
		#print detection
		while True:
			if (detection == 0):# or detection is None):
				notfound = notfound + 1
				counter = counter + 1

				linear_V=self.Velocity_Linear(P_saved)
				angular_V=self.Velocity_Angular(dx_pixel_saved)

				self.vel.linear.x = linear_V
				self.vel.angular.z = angular_V
				print linear_V, angular_V
				#self.pub_vel.publish(self.vel)

				#print("numero non trovati = %d" %notfound)
				#print("numero contati = %s" %counter)

				#print time.time()
				if ((time.time() - start) > 5):

					print "reset"
					P_saved = 0
					dx_pixel_saved = 0
					counter = 0

					linear_V=self.Velocity_Linear(P_saved)
					angular_V=self.Velocity_Angular(dx_pixel_saved)

					self.vel.linear.x = linear_V
					self.vel.angular.z = angular_V
					print linear_V, angular_V

					start = time.time()

			elif (detection == 1):	#detected only one person

				print "Found body!"
				counter = 0
				dx_pixel = cfXcol - center_x
				print dx_pixel
				dx_pixel_saved = dx_pixel
				P = imageDEPTH_perRGB[center_y, center_x]
				P_saved = P
				print("The mm distance of the body_position centre is %f" %P)

				linear_V=self.Velocity_Linear(P)
				angular_V=self.Velocity_Angular(dx_pixel)

				self.vel.linear.x=linear_V
				self.vel.angular.z=angular_V
				print linear_V, angular_V

				detection = 0


			else:	#detected more than 1 person STOP command

				print "Found body!"
				counter = 0
				print "Found more than one person. Stop the algorithm"

				if ((time.time() - start) > 1):

					print "reset"
					P_saved = 0
					dx_pixel_saved = 0
					counter = 0

					linear_V=self.Velocity_Linear(P_saved)
					angular_V=self.Velocity_Angular(dx_pixel_saved)

					self.vel.linear.x = linear_V
					self.vel.angular.z = angular_V
					print linear_V, angular_V

					start = time.time()
					moreperson = 1
				detection = 0;

			r = rospy.Rate(2)
			self.pub_vel.publish(self.vel)
			r.sleep()

			if (moreperson == 1):
				time.sleep(5); #set 15*60 to stop for 15 min
				moreperson = 0;
				start = time.time();


if __name__ == '__main__':

	init = INIT()

	i = INPUT()
	i.start()

	original_sigint = signal.getsignal(signal.SIGINT)
	signal.signal(signal.SIGINT, i.Exit_gracefully)
	try:
		i.control()
	except rospy.ROSInterruptException:
		print("ERROR")
		exit
