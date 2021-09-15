#! /usr/bin/env python
# coding=utf-8

"""
Created on Thur September 12 00:00:00 2019
The code takes the topic of a Realsense camera d435i using rgb camera and its aligned depth.
Subscribing to the darknet network topics it is possible to find the number of the detected object and their respectively bounding boxes.
Using this informations the code localizes the middle point of the object detected.
If only a person is detected the tracking iter starts and the person is followed according to a move_base Obstacle Avoidance algorithm.
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
import tf.transformations as trans
import math

from std_msgs.msg import *
from darknet_ros_msgs.msg import *
from nav_msgs.msg import *
from cv_bridge import CvBridge
from sensor_msgs.msg import *
from geometry_msgs.msg import *
from tf.msg import tfMessage
from time import sleep


m_vel_upperlimit = 1.9					#upper_limit of stop command movement linear velocity [m]
m_vel_lowerlimit = 1.7					#lower_limit of stop command movement linear velocity [m]

def create_straight_line(x1,y1,x2,y2):											#line use for the linear control

		m=(float(y2-y1))/(x2-x1)
		q=y1-(m*x1)
		return (m, q)

#points coordinates of linear velocity -move on:
x1_1 = 1
y1_1 = 0.13
x2_1 = 3
y2_1 = 0.26
(m1,q1) = create_straight_line(x1_1, y1_1, x2_1, y2_1)
#points coordinates of linear velocity -move back:
x1_2 = 1
y1_2 = -0.13
x2_2 = 0.3
y2_2 = -0.26
(m2,q2) = create_straight_line(x1_2, y1_2, x2_2, y2_2)


class INIT():
	def __init__(self):
		rospy.init_node('yolo_detection',anonymous=True)
		global imageDEPTH_perRGB, classname, probability, center_x, center_y, detection, T_Og, T_Mb, T_Ob
		imageDEPTH_perRGB = np.zeros((480, 640), np.uint8)
		classname, probability, center_x, center_y, detection = '', 0.0, 0, 0, 0
		T_Og, T_Mb, T_Ob=np.zeros((4,4),np.uint8),np.zeros((4,4),np.uint8),np.zeros((4,4),np.uint8)


class INPUT(threading.Thread):
	def __init__(self, cmd_vel='/cmd_vel', bounding_boxes = '/darknet_ros/bounding_boxes', cameraDEPTH_perRGB = '/camera/aligned_depth_to_color/image_raw', num_detection = '/darknet_ros/found_object',
					odom='/odom', scan = '/scan', tnf= '/tf'):
		threading.Thread.__init__(self)
		self.cmd_vel = cmd_vel
		self.bounding_boxes = bounding_boxes
		self.cameraDEPTH_perRGB = cameraDEPTH_perRGB
		self.num_detection = num_detection
		self.odom = odom
		self.scan = scan
		self.tnf = tnf
		self.pub_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
		self.pub_goal = rospy.Publisher('/move_base_simple/goal', PoseStamped)
		self.vel = Twist()
		self.initialpose=PoseWithCovarianceStamped()


	def run(self):
		self.bridge=CvBridge()
		self.bounding_boxes_sub = rospy.Subscriber(self.bounding_boxes, BoundingBoxes, self.bbox)
		self.imageDEPTH_perRGB_sub = rospy.Subscriber(self.cameraDEPTH_perRGB, Image, self.DEPTHimage_perRGB)
		self.num_detection_sub = rospy.Subscriber(self.num_detection, Int8, self.found_object)
		self.sub_lid = rospy.Subscriber(self.scan, LaserScan, self.Lidar)
		self.tnf= rospy.Subscriber(self.tnf,tfMessage, self.Transformation)
		self.sub_odom = rospy.Subscriber(self.odom, Odometry, self.Odometry)
		rospy.spin()

	def Exit_gracefully(self,signal, frame):
		signal.signal(signal.SIGINT, original_sigint)
		try:
			if raw_input("\Really quit? (y/n)> ").lower().startswith('y'):
				sys.exit(1)
		except KeyboardInterrupt:
			print ("OK. Quitting...")
			sys.exit(1)
		signal.signal(signal.SIGINT, Exit_gracefullty)


	def DEPTHimage_perRGB(self,data):											#DEPTH for RGB image callback
		global imageDEPTH_perRGB
		imageDEPTH_perRGB  = self.bridge.imgmsg_to_cv2(data, "16UC1")
		imageDEPTH_perRGB = np.array(imageDEPTH_perRGB, dtype=np.float32)

	def bbox(self, data):														#information about the detected object(s) and bounding boxes
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
			print center_x, center_y;

	def found_object(self, data):												#number of object found from the network
		global detection
		detection = data.data
		print ("person found %d" %detection)


	def Odometry(self, msg):													#compute the transformation from Map to base_footprint
		'''
		Control the odometry from the robot
		:param msg:
		:return: pose [coordinates, euler angles] as numpy array
		'''

		global T_Mb
		position = msg.pose.pose.position
		orientation = msg.pose.pose.orientation
		covariance= msg.pose.covariance


		coordinates = [position.x, position.y, position.z]
		euler = list(trans.euler_from_quaternion(
			(orientation.x, orientation.y, orientation.z, orientation.w)
			))

		initialpose = [coordinates, euler]
		yaw_Mo = euler[2] * 180 / math.pi
		print "the yaw angle is:", yaw_Mo
		T_Mo=np.array([[math.cos(yaw_Mo), -math.sin(yaw_Mo), 0.0, position.x],[math.sin(yaw_Mo), math.cos(yaw_Mo), 0.0, position.y], [0.0,0.0,1.0, position.z], [0.0,0.0,0.0,1.0]])
		T_Mb=T_Mo*T_Ob 	#Trasformation da Map a Base_footprint


	def Lidar(self, msg):														#LiDAR callback
		'''
		Control the LiDAR inputs
		:param msg:
		:return: ranges of the 360 values as numpy array
		'''
		global ranges
		ranges = msg.ranges

	def conversion_to_pose_stamped(self,dx_pixel,P_m, dy_pixel,fx=613.2378540039062, fy=612.938232421875):		#publish the GOAL position wrt the MAP rf.
		if (P_m != 0 and dx_pixel != 0):
			Zcam_m=P_m
			Xcam_m=P_m*dx_pixel/fx
			Ycam_m=P_m*dy_pixel/fy

			yaw_Bg=math.atan(Xcam_m/Zcam_m)

			T_Bg=np.array([[math.cos(yaw_Bg), -math.sin(yaw_Bg), 0.0, Zcam_m],[math.sin(yaw_Bg), math.cos(yaw_Bg), 0.0, Xcam_m], [0.0,0.0,1.0, 0.0], [0.0,0.0,0.0,1.0]])
			#print T_Bg
			T_MG=T_Mb*T_Bg		#Trasformation from Map to Goal
			#print "T_MG", T_MG

			yaw=math.asin(T_MG[1,0])

			posx=T_MG[0,3]
			posy=T_MG[1,3]
			#print posx, posy
			###############################################
			pose=PoseStamped()
			pose.header.stamp=rospy.Time.now()
			pose.header.frame_id='map'
			pose.pose.position.x=posx
			pose.pose.position.y=posy
			pose.pose.position.z=0
			quaternion=trans.quaternion_from_euler(0,0,yaw)
			pose.pose.orientation.x=quaternion[0]
			pose.pose.orientation.y=quaternion[1]
			pose.pose.orientation.z=quaternion[2]
			pose.pose.orientation.w=quaternion[3]

			#r=rospy.Rate(2)
			self.pub_goal.publish(pose)
			#r.sleep()

		else:
			pass


	def Transformation(self, data):																				#compute the TF from /odom to /base_footprint.
		global frame_id,child_frame_id, trns, T_Ob
		values = data.transforms
		for data in values:
			frame_id = data.header.frame_id
			child_frame_id=data.child_frame_id
			trn=data.transform
			#print frame_id , child_frame_id
			#print trn

			if(frame_id=="odom" and child_frame_id=="base_footprint"):
				x_T_Ob=data.transform.translation.x
				y_T_Ob=data.transform.translation.y
				z_T_Ob=data.transform.translation.z
				x_R_Ob=data.transform.rotation.x
				y_R_Ob=data.transform.rotation.y
				z_R_Ob=data.transform.rotation.z
				w_R_Ob=data.transform.rotation.w
				print 'base_footprint respect to odom'
				coordinates=[x_T_Ob,y_T_Ob,z_T_Ob];
				euler=list(trans.euler_from_quaternion((x_R_Ob,y_R_Ob,z_R_Ob,w_R_Ob)))
				pose=[coordinates,euler]
				yaw_Ob=euler[2]*180/math.pi
				#print "the yaw angle is:", yaw_Ob
				T_Ob=np.array([[math.cos(yaw_Ob), -math.sin(yaw_Ob), 0.0, x_T_Ob],[math.sin(yaw_Ob), math.cos(yaw_Ob), 0.0, y_T_Ob], [0.0,0.0,1.0, z_T_Ob], [0.0,0.0,0.0,1.0]])


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
			print "sinistraaaaaaa"

		elif dx_pixel == 0:
			v_angular_theta = 0

		else:												#	RIGTH SIDE
			v_angular_theta = min_vel*dx_pixel*dx_pixel/(320.0*320.0)
			print "destraaaaaaaaa"

		return v_angular_theta

	def control(self):

		global P_saved, dx_pixel_saved, counter, notfound, detection, moreperson
		counter = 0
		notfound = 0
		moreperson = 0
		cfXcol = 320-1
		cfYcol = 240-1
		P_saved, dx_pixel_saved, dy_pixel_saved = 0.01, 1, 0
		start = time.time()
		#print detection
		while True:
			if (detection == 0):# or detection is None):
				notfound = notfound + 1
				counter = counter + 1

				self.pose=self.conversion_to_pose_stamped(dx_pixel_saved,P_saved/1000, dy_pixel_saved)

				if ((time.time() - start) > 5):

					print "reset"
					P_saved = 0
					dx_pixel_saved = 0
					dy_pixel_saved = 0
					counter = 0

					linear_V=self.Velocity_Linear(P_saved)
					angular_V=self.Velocity_Angular(dx_pixel_saved)

					self.vel.linear.x = linear_V
					self.vel.angular.z = angular_V

					self.pub_vel.publish(self.vel)
					start = time.time()

			elif (detection == 1):	#detected only one person -> the person follow iter starts

				print "Found body!"
				counter = 0
				dx_pixel = cfXcol - center_x
				dy_pixel = cfYcol - center_y
				print dx_pixel
				dx_pixel_saved = dx_pixel
				dy_pixel_saved =dy_pixel
				P = imageDEPTH_perRGB[center_y, center_x]
				P_saved = P


				print("The mm distance of the body_position centre is %f" %P)

				self.conversion_to_pose_stamped(dx_pixel,P/1000,dy_pixel)

				detection = 0



			else:	#detected more than 1 person STOP command

				print "Found body!"
				counter = 0
				print "Found more than one person. Stop the algorithm"


				if ((time.time() - start) > 1):

					print "reset"
					P_saved = 0
					dx_pixel_saved = 0
					dy_pixel_saved = 0
					counter = 0

					linear_V=self.Velocity_Linear(P_saved)
					angular_V=self.Velocity_Angular(dx_pixel_saved)

					self.vel.linear.x = linear_V
					self.vel.angular.z = angular_V

					self.pub_vel.publish(self.vel)
					start = time.time()
					moreperson = 1
				detection = 0;


			#self.pub_goal.publish(self.pose)


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
