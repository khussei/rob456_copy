#!/usr/bin/env python3


import rospy
import sys
import numpy as np
from math import atan2, tanh, sqrt, pi

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point
from visualization_msgs.msg import Marker
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion

import actionlib
import tf

from lab2.msg import NavTargetAction, NavTargetResult, NavTargetFeedback


class Driver:
	def __init__(self, position_source, threshold=0.1):
		# Goal will be set later. The action server will set the goal; you don't set it directly
		self.goal = None
		self.threshold = threshold

		self.transform_listener = tf.TransformListener()

		# Publisher before subscriber
		self.cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
		self.target_pub = rospy.Publisher('current_target', Marker, queue_size=1)

		# Subscriber after publisher
		self.sub = rospy.Subscriber('base_scan', LaserScan, self._callback, queue_size=1)

		# Action client
		self.action_server = actionlib.SimpleActionServer('nav_target', NavTargetAction, execute_cb=self._action_callback, auto_start=False)
		self.action_server.start()

	@classmethod
	def zero_twist(cls):
		"""This is a helper class method to create and zero-out a twist"""
		command = Twist()
		command.linear.x = 0.0
		command.linear.y = 0.0
		command.linear.z = 0.0
		command.angular.x = 0.0
		command.angular.y = 0.0
		command.angular.z = 0.0

		return command

	# Respond to the action request.
	def _action_callback(self, goal):
		""" This gets called when an action is received by the action server
		@goal - this is the new goal """
		rospy.loginfo(f'Got an action request for ({goal.goal.point.x:.2f}, {goal.goal.point.y:.2f})')

		# Set the goal.
		self.goal = goal.goal

		# Build a marker for the goal point
		#   - this prints out the green dot in RViz (the current goal)
		marker = Marker()
		marker.header.frame_id = goal.goal.header.frame_id
		marker.header.stamp = rospy.Time.now()
		marker.id = 0
		marker.type = Marker.SPHERE
		marker.action = Marker.ADD
		marker.pose.position = goal.goal.point
		marker.pose.orientation.x = 0.0
		marker.pose.orientation.y = 0.0
		marker.pose.orientation.z = 0.0		
		marker.pose.orientation.w = 1.0
		marker.scale.x = 0.3
		marker.scale.y = 0.3
		marker.scale.z = 0.3
		marker.color.r = 0.0
		marker.color.g = 1.0
		marker.color.b = 0.0
		marker.color.a = 1.0

		# Wait until we're at the goal.  Once we get there, the callback that drives the robot will set self.goal
		# to None.
		while self.goal:
			self.target_pub.publish(marker)
			rospy.sleep(0.1)

		rospy.loginfo('Action completed')

		# Build a result to send back
		result = NavTargetResult()
		result.success.data = True

		self.action_server.set_succeeded(result)

		# Get rid of the marker
		marker.action = Marker.DELETE
		self.target_pub.publish(marker)

	def _callback(self, lidar):
		# If we have a goal, then act on it, otherwise stay still
		if self.goal:
			# Update the timestamp on the goal and figure out where it it now in the base_link frame.
			self.goal.header.stamp = rospy.Time.now()
			target = self.transform_listener.transformPoint('base_link', self.goal)

			rospy.loginfo(f'Target: ({target.point.x:.2f}, {target.point.y:.2f})')

			# Are we close enough?  If so, then remove the goal and stop
			distance = sqrt(target.point.x ** 2 + target.point.y ** 2)

			feedback = NavTargetFeedback()
			feedback.distance.data = distance
			self.action_server.publish_feedback(feedback)

			if distance < self.threshold:
				self.goal = None
				command = Driver.zero_twist()
			else:
				command = self.get_twist((target.point.x, target.point.y), lidar)
		else:
			command = Driver.zero_twist()

		self.cmd_pub.publish(command)

	# This is the function that controls the robot.
	#
	# Inputs:
	# 	target:	a tuple with the (x, y) coordinates of the target point, in the robot's coordinate frame (base_link).
	# 			x-axis is forward, y-axis is to the left.
	# 	lidar:	a LaserScan message with the current data from the LiDAR.  Use this for obstacle avoidance.
	#           This is the same as your go and stop code
	def get_twist(self, target, lidar):
		command = Driver.zero_twist()

		# TODO:
		#  Step 1) Calculate the angle the robot has to turn to in order to point at the target
		#  Step 2) Set your speed based on how far away you are from the target, as before
		#  Step 3) Add code that veers left (or right) to avoid an obstacle in front of it
		# This sets the move forward speed (as before)
		command.linear.x = 0.1
		# This sets the angular turn speed (in radians per second)
		command.angular.z = 0.1

  # YOUR CODE HERE
		# prepping the lidar's thetas for later use
		angle_min, angle_max, num_readings = lidar.angle_min, lidar.angle_max, len(lidar.ranges)
		thetas = np.linspace(angle_min, angle_max, num_readings) # the lidar's thetas (consistent)
		front_mindex, front_maxdex = round(num_readings / 4), round(3 * num_readings / 4)
		thetas_front = thetas[front_mindex : front_maxdex]
		# prepping given target and bot stuff
		target_x, target_y = target
		theta_g = atan2(target_y, target_x)
		bot_width, lidar_max = 0.38, 8 #m
			# for moving fwd
		r_closest = 1.19 #m had originally wanted it to be bot_width + threshold value
		v_max = 0.30 #m/s # max fwd velocity
		t_to_closest = 3 #s # change this to vary responsiveness -- time required to stop
		d_slow_down = v_max * t_to_closest # distance bot will slow to a stop
			# for rotating
		turn_radius =  bot_width * 1.5 #*2
		omega_max = v_max / turn_radius

		# figuring out shortest distance to an obstruction within the relevant front of lidar scope
		y_dists, all_rs, all_thetas, relevant_rs, relevant_thetas = np.zeros(num_readings), [],[], [], []
		for i, r in enumerate(lidar.ranges):
			y_dists[i] = r * np.sin(thetas[i])
			if abs(y_dists[i]) <= r_closest:
				all_rs.append(r)
				all_thetas.append(thetas[i])

				if front_mindex <= i < front_maxdex:
				#obstruction found in front of bot
					relevant_rs.append(r)
					relevant_thetas.append(thetas[i]) #thetas_front[i]
		unbounded_shortest = min(all_rs, default=lidar_max)
		bot_unbounded_lidar_theta = all_thetas[all_rs.index(unbounded_shortest)] if all_rs else thetas[-1]
		shortest = min(relevant_rs, default=lidar_max)
		bot_theta_obj = relevant_thetas[relevant_rs.index(shortest)] if relevant_rs else thetas[-1]
		# default commands and distance to target
		command.linear.x = v_max
		command.angular.z = 0
		distance = sqrt(target_x ** 2 + target_y ** 2)

		follow_goal = False
		count_veer = 0
		if shortest < d_slow_down:
		#if not out of range of lidar and there IS an obstruction between bot and target
			# move around obstruction at a reasonable speed
			print(f"obstruction found")
			count_veer = 3
			command.linear.x = v_max *  tanh(shortest / d_slow_down)
			command.angular.z =  tanh(-bot_theta_obj/turn_radius)
		elif unbounded_shortest < turn_radius and distance > unbounded_shortest:
			count_veer = 3
			print("too close to the sides!")
			command.angular.z = -bot_theta_obj * omega_max
		else: 
			if count_veer > 0:
				count_veer -=1
			else:
				follow_goal = True
		#if nothing between bot and goal
			#move to goal, turning while moving
				print(f"moving straight towards goal!")
				command.linear.x = v_max * tanh(distance/d_slow_down)
				command.angular.z = tanh(theta_g/turn_radius) #bc it's pov of bot, so theta_pov = 0 always
		return command

if __name__ == '__main__':
	rospy.init_node('driver', argv=sys.argv)

	driver = Driver('odom')

	rospy.spin()
