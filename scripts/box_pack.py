#!/usr/bin/env python

##################################
#        Baxter Box Packing
#            CSCI 5551
#            Fall 2018
#		17th December, 2018
#
#         Santhosh Alladi
#           Nabil Khan
#           Eli Latocki
##################################

#ROS libs

import rospy
import roslib

#CV libs
import cv2;
import cv_bridge

#Standard Python libs
import numpy as np
import math
import os
import sys
import string
import time
import random
import tf

#Baxter, CV, IK
from sensor_msgs.msg import Image
import baxter_interface
from moveit_commander import conversions
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header
import std_srvs.srv
from baxter_core_msgs.srv import SolvePositionIK, SolvePositionIKRequest
from cv_bridge import CvBridge, CvBridgeError



#Define program constants

#Minimum relative significance an object has to be on screen,
#in terms of both size and color intensity
BLOCK_MIN_MOMENT = 1000


#Define the gripper camera resolution (23.8 fps, 1280x800 only yields 14.1 fps)
CAM_W = 640
CAM_H = 400



#Contain everything relevant to the program node in a class for easy data access
class controller():
	def __init__(self):
		self.block_w = 0
		self.block_l = 0

		self.pack_mode = 0
		#Specify the default end effector positions (poses) for viewing the scenes
		print "Defining manipulator poses..."
		#Orientation in radians from Baxter's (0, 0, 0) rotation (use this for all poses)
		self.rot_rol = -1*math.pi	#Default puts camera pointed in, so flip for better view
		self.rot_pit =  0*math.pi	#Default is 0, pitched down
		self.rot_yaw =  0*math.pi	#Default is 0, yawed to center

		#View the candidate block pile (IR should hit table, blocks should be further forward than gripper)
		#Displacement in meters from Baxter's (0, 0, 0) position
		self.cand_px =  0.6;	#Forward amount, 0.6 is about the center of the work space
		self.cand_py = -0.3;	#Lateral amount, 0.0 is dead center, -0.3 is to the right
		self.cand_pz =  0.0;	#Height amount, 0.0 is as high as possible but still in IR rangefinder range

		#View the packing box (IR should hit table, box should be further forward than gripper)
		#Displacement in meters from Baxter's (0, 0, 0) position
		self.pack_px =  0.6;	#Forward amount, 0.6 is about the center of the work space
		self.pack_py =  0.0;	#Lateral amount, 0.0 is dead center, -0.3 is to the left
		self.pack_pz =  0.0;	#Height amount, 0.0 is as high as possible but still in IR rangefinder range

		#Set the left arm's position to somewhere it won't interfere with the right arm's operation
		#Displacement in meters from Baxter's (0, 0, 0) position
		self.left_px =  0.6;	#Forward amount, 0.6 is about the center of the work space
		self.left_py =  0.5;	#Lateral amount, 0.0 is dead center, 0.5 is to the left
		self.left_pz = -0.4;	#Height amount, -0.4 is quite low

		#Wrap the 6 pose parameters into a list defining a manipulator configuration
		#Default the pose configuration to the scene view configuration
		self.cand_pose = [self.cand_px, self.cand_py, self.cand_pz,    self.rot_rol, self.rot_pit, self.rot_yaw]	#View candidate blocks, fixed pose
		self.pick_pose = [self.cand_px, self.cand_py, self.cand_pz,    self.rot_rol, self.rot_pit, self.rot_yaw]	#Pick up a candidate, this pose changes
		self.pack_pose = [self.pack_px, self.pack_py, self.pack_pz,    self.rot_rol, self.rot_pit, self.rot_yaw]	#View the packing area, fixed pose
		self.plce_pose = [self.pack_px, self.pack_py, self.pack_pz,    self.rot_rol, self.rot_pit, self.rot_yaw]	#Place a held block, this pose changes
		self.left_pose = [self.left_px, self.left_py, self.left_pz,    self.rot_rol, self.rot_pit, self.rot_yaw]	#Permanent left arm spot, fixed pose

		self.curr_pose = self.cand_pose
		print "\tDone"



		#Specify the thresholds for identifying blocks of a specific color

		self.blue_color_min = np.array([85, 80, 20])
		self.blue_color_max = np.array([140, 180, 180])
		self.purple_color_min = np.array([140, 10, 30])
		self.purple_color_max = np.array([180, 80, 125])
		#Specify the thresholds for identifying the box
		#Current configuration is set to light blue
		self.box_color_min = np.array([50, 50, 50])#np.array([185, 160, 110])
		self.box_color_max = np.array([255, 255, 255])#np.array([120, 105, 75])

		self.curr1_color_min = np.array([0, 0, 0])
		self.curr1_color_max = np.array([255, 255, 255])
		self.curr2_color_min = np.array([0, 0, 0])
		self.curr2_color_max = np.array([255, 255, 255])


		#Declare x and y coordinates in camera space to be mapped to x and y in Baxter space
		#Coordinates represent the location of a found block with respect to the camera or Baxter frame
		self.cam_x = 0
		self.cam_y = 0

		self.bax_x = 0
		self.bax_y = 0
		self.pack_bound_x = 0
		self.pack_bound_y = 0
		self.block_bound_x = 0
		self.block_bound_y =0
		self.ori_yaw =0


	#Set either the left or the right limb to a 6-parameter pose using MoveIt
	def pose_limb(self, limb, pose):
		#Convert the pose to a quaternion, then request an IK solution
		quat_pose = conversions.list_to_pose_stamped(pose, "base")
		node = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
		ik_service = rospy.ServiceProxy(node, SolvePositionIK)
		ik_request = SolvePositionIKRequest()
		header = Header(stamp=rospy.Time.now(), frame_id="base")
		ik_request.pose_stamp.append(quat_pose)

		#Wait for solution response
		rospy.wait_for_service(node, 5.0)
		ik_response = ik_service(ik_request)

		if ik_response.isValid[0]:
			#Convert the solution to a joint position control dictionary and move
			limb_joints = dict(zip(ik_response.joints[0].name, ik_response.joints[0].position))
			baxter_interface.Limb(limb).move_to_joint_positions(limb_joints)
		else:
			sys.exit("pose_limb: A valid joint configuration could not be found")

		#The result pose only matters for the right arm
		if limb == "right":
			quat_pose = baxter_interface.Limb("right").endpoint_pose()

			#Update the manipulator's current pose
			#Orientation shouldn't change, also endpoint_pose() returns orientation
			#as a quaternion and that's annoying to use to update pose
			self.curr_pose = pose
			self.curr_pose[0] = quat_pose["position"][0]
			self.curr_pose[1] = quat_pose["position"][1]
			self.curr_pose[2] = quat_pose["position"][2]

	#Determine how far away the table is from the specified gripper while in a view pose
	#This distance is critical in converting camera coordinates to Baxter coordinates
	#Also used to determine how far from a view pose to move the gripper to pick up a block
	def find_range(self, limb):
		dist = baxter_interface.analog_io.AnalogIO(limb + "_hand_range").state()

		if dist > 65534:
			return -1

		return dist/1000

	#Camera callback function for interfacing with ROS at block picking time
	#Note that message is an implied argument in rospy.Subscriber()
	#The remaining arguments must be passed as a bound list
	def cam_callback(self, message):
		cv_image = CvBridge().imgmsg_to_cv2(message, "bgr8")				#Convert ROS image to CV image

		#Create a color mask for the color range specified by colors
		hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)						#Convert CV image from BGR to HSV
		hsv2 = cv2.medianBlur(hsv, 11)										#Remove noise from the HSV image
		#Create a mask over the desired colors
		blue_mask = cv2.inRange(hsv2, self.curr1_color_min, self.curr1_color_max)
		purple_mask = cv2.inRange(hsv2, self.curr2_color_min, self.curr2_color_max)
		mask = cv2.bitwise_or(blue_mask, purple_mask)
		# mask = purple_mask#cv2.bitwise_or(mask, green_mask, blue_mask, purple_mask)
		img = cv2.bitwise_and(hsv, hsv, mask=mask)							#Apply the mask to the image

		# imgray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
		# ret, thresh = cv2.threshold(imgray, 40, 255, 0)

		#Use the processed image to find edges in the image
		contours = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
		# contours = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
		if contours[1] == None:
			sys.exit("No block found")
		#Sort the contours list by area, smallest to largest
		contours_sorted = sorted(contours[1], key=lambda contour: cv2.moments(contour)["m00"], reverse=True)
		#Loop over the image edges and try to find object outlines
		for contour in contours_sorted:
			#Compute the centroid of the identified object (hopefully a block)
			M = cv2.moments(contour)

			#If the moment is significant enough, identify it as a block
			if M["m00"] >= BLOCK_MIN_MOMENT*(1 + 9*self.pack_mode): #High-IQ code, multiply area requirement by 10 when looking for box
				#[[x, y], [width, height], [angle of rotation (-90 to 0)]]
				rotated_rect = cv2.minAreaRect(contour)

				if rotated_rect[1][0] < rotated_rect[1][1]:
					self.ori_yaw = -rotated_rect[2]*math.pi/180
					if not self.pack_mode:
						self.block_w = rotated_rect[1][0]
						self.block_l = rotated_rect[1][1]
					else:
						self.pack_bound_x = rotated_rect[1][1]
						self.pack_bound_y = rotated_rect[1][0]
				else:
					self.ori_yaw = math.pi/2 - rotated_rect[2]*math.pi/180
					if not self.pack_mode:
						self.block_w = rotated_rect[1][1]
						self.block_l = rotated_rect[1][0]
					else:
						self.pack_bound_x = rotated_rect[1][0]
						self.pack_bound_y = rotated_rect[1][1]

				M = cv2.moments(contour)
				self.cam_x = int(M["m10"]/M["m00"])
				self.cam_y = int(M["m01"]/M["m00"])

				#Draw the block's contour and centroid over the image
				# cv2.drawContours(cv_image, [contour], -1, (255, 255, 255), 2, 8)
				cv2.circle(cv_image, (self.cam_x, self.cam_y), 5, (0, 255, 0), 1)
				draw_rect = cv2.boxPoints(rotated_rect)
				draw_rect = np.int0(draw_rect)
				cv2.drawContours(cv_image, [draw_rect], 0, (255, 255, 255), 2)

				#Find the block's Baxter coordinates
				#Measured camera window size in cm at 0.3 m above table
				#Came up with 45.406 cm x 28.367 cm
				#0.047 for 960x600
				#0.071 for 640x400
				#0.022 is the camera to gripper x offset
				#0.030 is the camera to gripper y offset
				if not self.pack_mode:
					self.offset_x = (self.cam_y - (CAM_H/2))
				self.bax_x = self.curr_pose[0] + ((self.pack_mode*(-rotated_rect[1][0] + self.block_l)/2 + self.cam_y - (CAM_H/2))*0.00071) + 0.022*math.cos(self.rot_yaw) + 0.030*math.sin(self.rot_yaw)
				self.bax_y = self.curr_pose[1] + ((self.pack_mode*(-rotated_rect[1][1] + self.block_w)/2 + self.cam_x - (CAM_W/2))*0.00071) - 0.030*math.cos(self.rot_yaw) + 0.022*math.sin(self.rot_yaw)

		# cv2.namedWindow("Block Camera", 1)
		cv2.imshow("Block Camera", cv_image)

		#Set ms display wait to roughly twice every time the camera updates
		cv2.waitKey(15)

def main():
	print "##################################"
	print "        Baxter Box Packing        "
	print "            CSCI 5551             "
	print "            Fall 2018             "
	print "                                  "
	print "         Santhosh Alladi          "
	print "           Nabil Khan             "
	print "           Eli Latocki            "
	print "##################################\n\n"



	#Initialize the program as a ROS node
	print "Initializing box_pack_node..."
	rospy.init_node("box_pack_node")
	print "\tDone"

	#Instantiate the controller
	ctrl = controller()

	#Create the OpenCV camera display window
	camera = baxter_interface.CameraController("right_hand_camera")
	camera.resolution = (int(CAM_W), int(CAM_H))
	camera.open()



	print "Enabling robot..."
	#Initialize Baxter properties, then enable Baxter limbs
	baxter_interface.Limb("right").set_joint_position_speed(1)
	baxter_interface.Limb("left").set_joint_position_speed(1)

	#Enable the actuators
	baxter_interface.RobotEnable().enable()
	print "\tDone"

	print "Setting right arm to default pose..."
	#Set the right limb's pose to the candidate block pile view pose
	ctrl.pose_limb("right", ctrl.cand_pose)
	print "\tDone"

	print "Setting left arm to default pose..."
	#Set the left limb's pose to somewhere out of the way
	ctrl.pose_limb("left", ctrl.left_pose)
	print "\tDone"

	#Define and calibrate the right limb's gripper, then open it
	print "Calibrating gripper..."
	gripper = baxter_interface.Gripper("right")
	# gripper.set_velocity(0.1)
	gripper.calibrate()
	gripper.calibrate() #calibrating second time, if the first calibration fails for reasons like: gripper already holding a block
	gripper.open()
	print "\tDone"


	print "Finding gripper-to-table range..."
	#Use the IR rangefinder on the right gripper to determine how far the table is
	ctrl.table_dist = ctrl.find_range("right")

	#Move arm to the desired viewing elevation
	while ctrl.table_dist == -1:
		print "\tRange not found\n\tLowering gripper 4 cm..."
		ctrl.cand_pose[2] -= 0.04	#Lower the candidate pile view z amount by 4 cm
		ctrl.pack_pose[2] -= 0.04	#Lower the packing box view z amount by 4 cm
		ctrl.pose_limb("right", ctrl.cand_pose)
		ctrl.table_dist = ctrl.find_range("right")
		print "\tRange = " + str(ctrl.table_dist) + " meters"

	while abs(ctrl.table_dist - 0.30) > 0.005:
		print "\tRange not equal to desired value\n\tMoving gripper..."
		ctrl.cand_pose[2] -= (ctrl.table_dist - 0.30)	#Move the candidate pile view z amount by difference
		ctrl.pack_pose[2] -= (ctrl.table_dist - 0.30)	#Move the packing box view z amount by difference
		ctrl.pose_limb("right", ctrl.cand_pose)
		ctrl.table_dist = ctrl.find_range("right")
		print "\tRange = " + str(ctrl.table_dist) + " meters"

	print "\tRange = " + str(ctrl.table_dist) + " meters"



	print "Subscribing to pick camera..."
	#Create the subscriber with the block finder callback
	subscriber = rospy.Subscriber("/cameras/right_hand_camera/image", Image, ctrl.cam_callback)
	print "\tDone"

	ix = 0
	iy = 0
	prev_bax_x = 0
	prev_bax_y = 0
	while True:
		ctrl.pack_mode = 0
		ctrl.curr1_color_min = ctrl.blue_color_min
		ctrl.curr1_color_max = ctrl.blue_color_max
		ctrl.curr2_color_min = ctrl.purple_color_min
		ctrl.curr2_color_max = ctrl.purple_color_max
		time.sleep(2)


		gripper.open()
		print "Picking up block..."
		#Pick up the found block

		ctrl.pick_pose = [ctrl.bax_x, ctrl.bax_y, ctrl.cand_pose[2], ctrl.rot_rol, ctrl.rot_pit, ctrl.rot_yaw+ctrl.ori_yaw ]
		prev_bax_x = ctrl.bax_x
		prev_bax_y = ctrl.bax_y
		prev_rot_yaw = ctrl.rot_yaw+ctrl.ori_yaw
		curr_block_w = ctrl.block_w
		curr_block_l = ctrl.block_l
		ctrl.pose_limb("right", ctrl.pick_pose)
		ctrl.pick_pose = [prev_bax_x, prev_bax_y, ctrl.cand_pose[2] - ctrl.table_dist + 0.12, ctrl.rot_rol, ctrl.rot_pit, prev_rot_yaw]#ctrl.rot_yaw+ctrl.ori_yaw]
		ctrl.pose_limb("right", ctrl.pick_pose)
		gripper.close()
		print "\tDone"

		ctrl.pick_pose = [prev_bax_x, prev_bax_y, ctrl.cand_pose[2], ctrl.rot_rol, ctrl.rot_pit, ctrl.rot_yaw]
		ctrl.pose_limb("right", ctrl.pick_pose)

		ctrl.pack_mode = 1
		ctrl.curr1_color_min = ctrl.box_color_min
		ctrl.curr1_color_max = ctrl.box_color_max
		ctrl.curr2_color_min = ctrl.box_color_min
		ctrl.curr2_color_max = ctrl.box_color_max
		time.sleep(2)

		#TODO: Add loop logic to the block grabbing code, repeat if failed to pick up
		#Something like if grippers are able to close fully, open, return to view, try again

		#Take the block to the packing area
		print "Taking block to the packing area..."
		ctrl.pose_limb("right", ctrl.pack_pose)
		print "\tDone"



		#Place the block in the correct spot in the box
		print "Packing the block..."
		time.sleep(2)
		ctrl.plce_pose = [ctrl.bax_x, ctrl.bax_y, ctrl.cand_pose[2] , ctrl.rot_rol, ctrl.rot_pit, ctrl.rot_yaw]
		curr_plce_pose = ctrl.plce_pose
		pack_bound_w = ctrl.pack_bound_x
		pack_bound_l = ctrl.pack_bound_y

		if ctrl.block_bound_x+ 2*curr_block_l> pack_bound_w:
			print "next row"
			ix = 0
			iy += 1
			ctrl.block_bound_x = 0
		if curr_plce_pose[1] + 0.05*iy > pack_bound_l:
			sys.exit("packing boundaries exceeded")

		curr_plce_pose[0] += ctrl.block_bound_x*0.00071
		curr_plce_pose[1] += 0.05*iy
		ctrl.pose_limb("right", curr_plce_pose)
		curr_plce_pose[2] +=  - ctrl.table_dist + 0.12
		ctrl.pose_limb("right", curr_plce_pose)

		gripper.open()
		print "\tDone"

		ctrl.block_bound_x += 14
		ctrl.block_bound_x += curr_block_l

		#Return to the candidate block viewing area
		print "Returning to the block pile..."
		ctrl.pose_limb("right", ctrl.cand_pose)
		gripper.close()
		print "\tDone"

		ix += 1

	print("Execution complete")

if __name__ == "__main__":
	main()
