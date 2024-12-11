#!/usr/bin/env python3


import sys
import rospy
import numpy as np

from new_driver import Driver

from math import atan2, sqrt, tanh, sin


class StudentDriver(Driver):
	'''
	This class implements the logic to move the robot to a specific place in the world.  All of the
	interesting functionality is hidden in the parent class.
	'''
	def __init__(self, threshold=0.1):
		super().__init__('odom')
		# Set the threshold to a reasonable number
		self._threshold = threshold
		self.count_veer = 0 #tracks veering actions to avoid rapid direction changes


	def close_enough_to_waypoint(self, distance, target, lidar):
		'''
		This function is called perioidically if there is a waypoint set.  This is where you should put any code that
		has a smarter stopping criteria then just checking the distance. See get_twist for the parameters; distance
		is the current distance to the target.
		'''
		# Default behavior.
		if distance < self._threshold:
			return True
		return False
	def get_twist(self, target, lidar):
		# Bot Parameters:
		#	lidar_max: lidar captures this distance around bot [m]
		#	v_max: maximum forward speed [m/s]
		#	t_to_closest: time to slow down in -- responsiveness [s]
		#	r_closest: maximum distance allowed to an opject [m]
		params = {'bot_width':0.38, 'lidar_max': 8.0, 'v_max': 0.30, 't_to_closest': 3, 'r_closest': 1.19}
		
		command = Driver.zero_twist()

		# prepping the lidar's thetas for later use
		angle_min, angle_max, num_readings = lidar.angle_min, lidar.angle_max, len(lidar.ranges)
		thetas = np.linspace(angle_min, angle_max, num_readings) # the lidar's thetas (consistent)
		front_mindex, front_maxdex = round(num_readings / 4), round(3 * num_readings / 4)
		# prepping given target
		target_x, target_y = target
		theta_g = atan2(target_y, target_x)

		#robot movement calcs from parameters
		d_slow_down = params['v_max'] * params['t_to_closest'] # constant -- distance bot will slow to a stop
		
		# figuring out shortest distance to an obstruction within the relevant front of lidar scope
		relevant_rs, relevant_thetas = [],[]
		for i, r in enumerate(lidar.ranges):
			y_dist = r * sin(thetas[i])
			if abs(y_dist) <= params['r_closest']:
				if front_mindex <= i < front_maxdex:
				#obstruction found in front of bot
					relevant_rs.append(r)
					relevant_thetas.append(thetas[i])
		unbounded_shortest = min(lidar.ranges, default=params['lidar_max'])
		shortest = min(relevant_rs, default=params['lidar_max'])
		bot_theta_obj = relevant_thetas[relevant_rs.index(shortest)] if relevant_rs else thetas[-1]
		bot_unbounded_lidar_theta = thetas[lidar.ranges.index(unbounded_shortest)]
		
		# default commands and distance to target
		command.linear.x = params['v_max']
		command.angular.z = 0
		distance = sqrt(target_x ** 2 + target_y ** 2)
		
		# obstacle avoidance logic
		#reset_veer = False
		if distance < unbounded_shortest:
			#if distance to goal is shorter than distance to anything else within lidar ranges
				#go towards goal!
			rospy.loginfo("going straight to goal!")
			command.linear.x = params['v_max'] * tanh(distance / d_slow_down)
			command.angular.z = tanh(theta_g)  # Turn toward the goal
		elif shortest < d_slow_down:
        	# obstacle in front
				#move away from it
			rospy.loginfo("obstruction directly in front")
			#reset_veer = True
			command.linear.x = params['v_max'] * tanh(shortest / d_slow_down)
			command.angular.z = tanh(-bot_theta_obj)
		elif unbounded_shortest < params['bot_width']:
        	# obstacle on side
				#pivot and don't move forward until there is nothing in front of you
			rospy.loginfo("Too close to an obstacle on the side!")
			#reset_veer = True
			command.linear.x = 0
			command.angular.z = tanh(-bot_unbounded_lidar_theta)
			# if abs(bot_theta_obj - thetas[-1]) < 0.5:
			# 	#if there is no object in the front
			# 	command.angular.z = 0
			# 	command.linear.x = params['v_max'] * tanh(shortest / d_slow_down)
			dot_corner_det = 
		# else:
        # 	# no obstacles
		# 	if self.count_veer > 0:
		# 		self.count_veer -= 1
			# else:
			# 	print("moving straight towards the goal!")
			# 	command.linear.x = params['v_max'] * tanh(distance / d_slow_down)
			# 	command.angular.z = tanh(theta_g)  # Turn toward the goal

    	# # only reset veer if needed
		# if reset_veer and self.count_veer == 0:
		# 	self.count_veer = 3
		
		return command


if __name__ == '__main__':
	rospy.init_node('student_driver', argv=sys.argv)

	driver = StudentDriver()

	rospy.spin()
