#!/usr/bin/env python3

# Bill Smart, smartw@oregonstate.edu
#
# This example gives the robot callback based stopping.


# Import ROS Python basic API and sys
import rospy
import sys

# We're going to do some math
import numpy as np

# Velocity commands are given with Twist messages, from geometry_msgs
from geometry_msgs.msg import Twist

# Laser scans are given with the LaserScan message, from sensor_msgs
from sensor_msgs.msg import LaserScan


# A callback to deal with the LaserScan messages.
def callback(scan):
	# Every time we get a laser scan, calculate the shortest scan distance in front
	# of the robot, and set the speed accordingly.  We assume that the robot is 38cm
	# wide.  This means that y-values with absolute values greater than 19cm are not
	# in front of the robot.  It also assumes that the LiDAR is at the front of the
	# robot (which it actually isn't) and that it's centered and pointing forwards.
	# We can work around these assumptions, but it's cleaner if we don't

	# Pulling out some useful values from scan
	angle_min = scan.angle_min
	angle_max = scan.angle_max
	num_readings = len(scan.ranges)

	# Doing this for you - get out theta values for each range/distance reading
	thetas = np.linspace(angle_min, angle_max, num_readings)

	# TODO: Determine what the closest obstacle/reading is for scans in front of the robot
	#  Step 1: Determine which of the range readings correspond to being "in front of" the robot (see comment at top)
	#    Remember that robot scans are in the robot's coordinate system - theta = 0 means straight ahead
	#  Step 2: Get the minimum distance to the closest object (use only scans "in front of" the robot)
	#  Step 3: Use the closest distance from above to decide when to stop
	#  Step 4: Scale how fast you move by the distance to the closest object (tanh is handy here...)
	#  Step 5: Make sure to actually stop if close to 1 m
	# Finally, set t.linear.x to be your desired speed (0 if stop)
	# Suggestion: Do this with a for loop before being fancy with numpy (which is substantially faster)
	# DO NOT hard-wire in the number of readings, or the min/max angle. You CAN hardwire in the size of the robot

	# Create a twist and fill in all the fields (you will only set t.linear.x).
	t = Twist()
	t.linear.x = 3.0 #/ how fast you move fwd
	t.linear.y = 0.0
	t.linear.z = 0.0
	t.angular.x = 0.0
	t.angular.y = 0.0
	t.angular.z = 0.0 #/ how fast you turn

	#shortest = 0 #/ apparently you can set this to the closest thing youll let near you?
 # YOUR CODE HERE
	bot_width = .38 #m
	r_stop = 1 + bot_width/2 #m, so 1.19m
	v_max = .30 #m/s
	t_to_stop = 3 #s responsiveness -- time required to stop
	d_slow_down = v_max * t_to_stop
	
	#relevant_rs = np.array
	y_dists = np.zeros(num_readings)
	#x_dists = np.zeros(num_readings)
	relevant_rs = []
	for i, r in enumerate(scan.ranges):
		y_dists[i] = r * np.sin(thetas[i]) #thetas in radians
		#x_dists[i] = r * np.cos(thetas[i])
		if abs(y_dists[i]) <= bot_width:
			#obstruction found in front of bot
			relevant_rs.append(r)
	shortest = min(relevant_rs)

	if shortest <= r_stop:
		t.linear.x = 0
	else:
		t.linear.x = v_max * (1 + np.tanh((shortest - r_stop) / d_slow_down))
	# Send the command to the robot.
	publisher.publish(t)

	# Print out a log message to the INFO channel to let us know it's working.
	rospy.loginfo(f'Shortest: {shortest} => {t.linear.x}')
	#rospy.loginfo(f'angle_min = {angle_min} angle_max = {angle_max}')
	#rospy.loginfo(f'num_readings = {num_readings}')
	#rospy.loginfo(f'thetas = {thetas}')


if __name__ == '__main__':
	# Initialize the node, and call it "driver".
	rospy.init_node('stopper', argv=sys.argv)

	# Set up a publisher.  The default topic for Twist messages is cmd_vel.
	publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)

	# Set up a subscriber.  The default topic for LaserScan messages is base_scan.
	subscriber = rospy.Subscriber('base_scan', LaserScan, callback, queue_size=10)

	# Now that everything is wired up, we just spin.
	rospy.spin()
