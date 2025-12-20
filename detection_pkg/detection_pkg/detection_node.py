import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from PyKDL import Frame, Vector, Rotation
from geometry_msgs.msg import PoseStamped

from sensor_msgs.msg import Image, CameraInfo
from rclpy.wait_for_message import wait_for_message
from cv_bridge import CvBridge
import cv2

import argparse
import os.path
import sys
import random
import glob
import time
import math
from scipy.spatial.transform import Rotation as R


class ObjectPositionDetection(Node):

	def __init__(self):
		super().__init__('person_detector')

		self.bridge = CvBridge()
		self.subscription = self.create_subscription(Image, '/camera/image_raw', self.rgb_callback, 1)
		self.subscription = self.create_subscription(Image, '/camera/depth/image_raw', self.depth_callback, 1)
		self.publisher_goal = self.create_publisher(PoseStamped, 'goal_pose', 10) # comment this line to keep the robot static
		timer_period = 0.5 														  # comment this line to keep the robot static
		self.timer = self.create_timer(timer_period, self.timer_callback) 		  # comment this line to keep the robot static
		self.publisher_image = self.create_publisher(Image, '/image_recording', 10)

		self.t_odom = None
		self.t_base = None
		self.tf_buffer = Buffer()
		self.tf_buffer2 = Buffer()
		self.tf_listener = TransformListener(self.tf_buffer, self)
		self.tf_listener2 = TransformListener(self.tf_buffer2, self)
		self.timer_odom = self.create_timer(0.1, self.timer_tf_odom)
		self.timer_base = self.create_timer(0.1, self.timer_tf_base)

		self.odom_frame = "map"
		self.base_frame = "base_link"
		self.camera_frame = "camera_depth_optical_frame" #TODO: check the name of the camera frame
		self.camera_info_topic = "/camera/depth/camera_info"

		self.cam_K = None
		self.cam_D = None
		self.image_coord = None
		self.goal_coord = None
		self.goal_angle = None

		# Load the network
		textGraph = "/home/enrib/turtlebot3_ws/src/detection_pkg/detection_pkg/mask_rcnn_inception_v2_coco_2018_01_28.pbtxt"
		modelWeights = "/home/enrib/turtlebot3_ws/src/detection_pkg/detection_pkg/frozen_inference_graph.pb"

		self.net = cv2.dnn.readNetFromTensorflow(modelWeights, textGraph)
		self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
		self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)

		self.safe_dist = 0.6 # 0.6
		self.pos_treshold = 0.2
		# self.angle_treshold = 0.08 # circa 10 gradi
		self.video_recording = False


	def rgb_callback(self, msg):
		if self.image_coord is not None:
			self.get_logger().info("rgb callback: waiting for depth callback")
			return
		self.get_logger().info("rgb call ")

		self.img_rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
		self.img_rgb = cv2.resize(self.img_rgb, (self.img_rgb.shape[1]//3, self.img_rgb.shape[0]//3), cv2.INTER_AREA)
		#target = cv2.cvtColor(self.img_rgb, cv2.COLOR_BGR2GRAY)

		centersdetect = self.detector(self.img_rgb)

		self.image_coord = centersdetect

	def depth_callback(self, msg):
		if self.t_odom is None:
			self.get_logger().info("t odom none")
			return
		if self.t_base is None:
			self.get_logger().info("t base none")
			return
		if self.image_coord is None :
			self.get_logger().info("depth callback: waiting for rgb callback")
			return
		if len(self.image_coord) == 0:
			self.image_coord = None # reset the image_coord to wait the next detection for the depth callback
			self.get_logger().info("no target fuond")
			return

		self.depth_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding=msg.encoding)
		# self.depth_img = cv2.resize(self.depth_img, (self.depth_img.shape[1]//2, self.depth_img.shape[0]//2), cv2.INTER_AREA)
		
		if self.cam_K is None:
			ret, info_msg =  wait_for_message(CameraInfo, self, self.camera_info_topic)
			self.cam_K = np.array(info_msg.k).reshape(3,3)
			self.cam_D = np.array(list(info_msg.d))

		# Create a Point Cloud
		if self.cam_K is not None and self.depth_img is not None:
			# print("K is not none")
			camera_coord = []
			angles = []
			for i in range(len(self.image_coord)):
				u = self.image_coord[i][0] * 3
				v = self.image_coord[i][1] * 3
				depth_value = self.depth_img[v, u]
				if depth_value > 0 :
					X = depth_value * (u - self.cam_K[0,2]) / self.cam_K[0,0] 
					Y = depth_value * (v - self.cam_K[1,2]) / self.cam_K[1,1]
					Z = depth_value

					angle = math.atan2(X, Z)

					dist = math.sqrt(X**2 + Z**2)

					if dist > self.safe_dist :
						# Subtract the safe distance by taking in account the sign
						X = X * (1 - abs(self.safe_dist * math.sin(angle) / X))
						Z = Z * (1 - abs(self.safe_dist * math.cos(angle) / Z))
						# angle = math.atan2(X, Z)

						camera_coord.append([X, Y, Z])
						angles.append(angle)
					else :  
						self.get_logger().info("Goal reached")
						self.image_coord = None
						return
					
			# Compute the world coordinates
			# world_coord = []
			for i in range(len(camera_coord)):
				coord, quat = self.get_odom_coord(camera_coord[i])

				self.get_logger().info(f"detected taget {i} at: {coord} , {angles[i]}")

				pos_differance = self.pos_treshold + 1 # default value greater than the treshold, so if goal_coord is None I enter in the if to set its fisrt value
				if self.goal_coord is not None :
					pos_differance = math.sqrt((coord[0]-self.goal_coord[0])**2 + (coord[1]-self.goal_coord[1])**2)

				# angle_differance = self.angle_treshold + 1
				# if self.goal_angle is not None :
				# 	angle_differance = abs(angle - self.goal_angle)
				
				if pos_differance > self.pos_treshold : # or angle_differance > self.angle_treshold: 
					# world_coord.append(coord)
					self.goal_coord = coord
					self.goal_angle = quat
					self.get_logger().info(f"taget updated")
				else :
					self.get_logger().info(f"taget neglegible with diff: {pos_differance}") # , {angle_differance * 180/ 3.14}")
					
		self.image_coord = None # reset the image_coord to wait the next detection for the depth callback
			
	def detector(self, frame):

		start_time = time.time()

		confThreshold = 0.5  # Confidence threshold 0.7
		maskThreshold = 0.5  # Mask threshold
		nmsThreshold = 0.9

		# Load names of classes
		classesFile = "/home/enrib/turtlebot3_ws/src/detection_pkg/detection_pkg/mscoco_labels.names"
		with open(classesFile, 'rt') as f:
			classes = f.read().rstrip('\n').split('\n')

		# Create a 4D blob from a frame.\\
		blob = cv2.dnn.blobFromImage(frame, swapRB=True, crop=False)

		# Set the input to the network
		self.net.setInput(blob)

		boxes_list = self.net.forward(['detection_out_final']) 
		boxes = boxes_list[0] 
		# For each frame, extract the bounding box for each detected object

		# Output size of masks is NxCxHxW where
		# N - number of detected boxes
		# C - number of classes (excluding background)
		# HxW - segmentation shape
		# numClasses = masks.shape[1]
		numDetections = boxes.shape[2]

		frameH = frame.shape[0]
		frameW = frame.shape[1]

		classIds = []
		confidences = []
		newboxes = []

		for i in range(numDetections):
			box = boxes[0, 0, i]
			# mask = masks[i]
			score = box[2]
			if score > confThreshold:
				classId = int(box[1])

				# Extract the bounding box
				left = int(frameW * box[3])
				top = int(frameH * box[4])
				right = int(frameW * box[5])
				bottom = int(frameH * box[6])

				left = max(0, min(left, frameW - 1))
				top = max(0, min(top, frameH - 1))
				right = max(0, min(right, frameW - 1))
				bottom = max(0, min(bottom, frameH - 1))

				classIds.append(classId)
				confidences.append(float(score))
				newboxes.append([left, top, right, bottom])

		indices = cv2.dnn.NMSBoxes(newboxes, confidences, confThreshold, nmsThreshold) # Non-maximum suppression of bounding boxes 

		# boxesdetect = []
		centersdetect = []
		font = cv2.FONT_HERSHEY_DUPLEX
		for i in indices:
			#i = i[0]
			box = newboxes[i]
			left = box[0]
			top = box[1]
			right = box[2]
			bottom = box[3]
			# classMask = mask[classIds[i]]
			size_ratio = (bottom-top)/frameH

			if classIds[i] == 0 and size_ratio >= 0.0: 
				# b = np.array([[left], [top], [right-left], [bottom-top]])
				c = np.array([left+(right-left)//2 , bottom]) # center of the bottom side of th box
				
				if self.video_recording :
					# label = '%s' % ('frame', 'ID', str(person_id))
					label = classes[classIds[i]]
					# cv2.putText(frame, label, (left-20, top-10), font, 1, (255, 178, 50))
					cv2.putText(frame, label, (left, bottom+20), font, 1, (255, 178, 50))
					cv2.rectangle(frame, (left, top), (right, bottom), (255, 178, 50), 2)
					cv2.circle(frame, (c[0], c[1]), 3, (0, 255, 0), -1) 
					#self.video_writer.write(frame)
					self.publish_image(frame)
				# boxesdetect.append(b)
				centersdetect.append(c)

		print('Detection time: {}'.format(time.time()-start_time)) # print the time taken for detection in seconds

		return centersdetect

	def get_frame_kdl(self, tf):    # trasforma TransformStamped in KDL frame
		pos = [tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z]
		quat = [tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z, tf.transform.rotation.w]

		frame = Frame(Rotation.Quaternion(quat[0], quat[1], quat[2], quat[3]), Vector(pos[0], pos[1], pos[2]))
		return frame

	def timer_tf_odom(self):    # t_base = transfrom tra il base link e la camera
		try:
			self.t_odom = self.tf_buffer.lookup_transform(self.odom_frame, self.camera_frame, rclpy.time.Time())
		except:
			self.get_logger().info("Could not transform odom-camera!")
		return

	def timer_tf_base(self):    # t_base = transfrom tra il base link e la camera
		try:
			self.t_base = self.tf_buffer2.lookup_transform(self.odom_frame, self.base_frame, rclpy.time.Time())
		except:
			self.get_logger().info("Could not transform odom-base!")
		return

	def get_odom_coord(self, pos): # from robot coord to world coord 
		if self.t_odom is None:
			self.get_logger().info("t_odom is none")
			return
		if self.t_base is None:
			self.get_logger().info("t_base is none")
			return

		frame_odom = self.get_frame_kdl(self.t_odom) # transform between odom and camera frame
		frame_base = self.get_frame_kdl(self.t_base) # transform between odom and base frame

		pos_kdl = Vector(pos[0], pos[1], pos[2]) # pos wrt the camera frame

		# Compute the position wrt the odom frame
		odom_coord_kdl = frame_odom * pos_kdl

		odom_coord = np.array([odom_coord_kdl.x(), odom_coord_kdl.y(), odom_coord_kdl.z()])

		# Compute the orientation
		R_base = frame_base.M
		frame_base_inv = frame_base.Inverse() # to trasform the coord of the target from the odom to the base frame
		base_coord_kdl = frame_base_inv * odom_coord_kdl
		theta =  math.atan2(base_coord_kdl.y() , base_coord_kdl.x()) 
		print("theta ", theta)
		R_base.DoRotZ(theta)

		qx, qy, qz, qw = R_base.GetQuaternion()
		print(qx, qy, qz, qw)
		quat = np.array([0.0, 0.0, qz, qw])

		# # R_camera = Rotation.RPY(0.0, -ang, 0.0)
		# # R = R_camera * frame_odom.M.Inverse()
		# R = frame_odom.M
		# roll, pitch, yaw = R.GetEulerZYX()
		# R.DoRotX(roll)
		# R.DoRotY(pitch)
		# roll, pitch, yaw = R.GetEulerZYX()
		# print(roll, pitch, yaw)
		# R_z = Rotation.RotZ(yaw)
		# qx, qy, qz, qw = R_z.GetQuaternion()
		# print(qx, qy, qz, qw)
		# # quat = np.array([0.0, 0.0, qz, qw]) # TODO: conside only the rotation about z
		# quat = np.array([0.0, 0.0, 0.0, 0.0])

		return odom_coord, quat

	def timer_callback(self):
		if self.goal_coord is None or self.goal_angle is None :
			return

		goal_pose = PoseStamped()
		goal_pose.header.frame_id = 'map' # odom
		goal_pose.header.stamp = self.get_clock().now().to_msg()

		goal_x = float(self.goal_coord[0])
		goal_y = float(self.goal_coord[1])
		goal_w = float(0.0)
		goal_pose.pose.position.x = float(goal_x)
		goal_pose.pose.position.y = float(goal_y)
		goal_pose.pose.orientation.w = float(goal_w)

		# Orientazione in quaternion (qx, qy, qz, qw)
		# roll = 0.0  # Rotazione attorno all'asse X
		# pitch = 0.0  # Rotazione attorno all'asse Y
		# yaw = 0.0  # Rotazione attorno all'asse Z 

		# if self.goal_angle is not None :
		# 	yaw = self.goal_angle

		# quaternion = R.from_euler('xyz', [roll, pitch, yaw]).as_quat()
		goal_pose.pose.orientation.x = float(self.goal_angle[0])
		goal_pose.pose.orientation.y = float(self.goal_angle[1])
		goal_pose.pose.orientation.z = float(self.goal_angle[2])
		goal_pose.pose.orientation.w = float(self.goal_angle[3])

		self.publisher_goal.publish(goal_pose)
		# self.get_logger().info("published")
	
	def publish_image(self, frame):
		if frame is None:
			self.get_logger().info("Immagine non trovata!")
			return

		msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
		self.publisher_image.publish(msg)
		self.get_logger().info("Immagine pubblicata!")


def main(args=None):
	rclpy.init(args=args)

	detect_node = ObjectPositionDetection()

	# try:
	# 	rclpy.spin(detect_node)
	# except KeyboardInterrupt:
	# 	detect_node.video_writer.release()
	# 	print("Registrazione interrotta, file salvato correttamente.")

	rclpy.spin(detect_node)
	detect_node.destroy_node()
	rclpy.shutdown()


if __name__ == "__main__":
	main()

