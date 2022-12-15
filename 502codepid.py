#!/usr/bin/env python
# -*- coding: utf-8 -*-

from math import pi, atan2, cos, sqrt
import numpy as np
import rospy
import tf
from std_msgs.msg import Empty
from sensor_msgs.msg import LaserScan # LaserScan type message is defined in sensor_msgs
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose2D

class Controller:
	def __init__(self, P=0.0, I=0.0, D=0.0, set_point=0, Time_Step = 0.005):
		self.Time_Step = Time_Step
		self.Kp = P
		self.Ki = I
		self.Kd = D
		self.set_point = set_point  # reference (desired value of theta)
		self.error = 0
		self.previous_error = 0
		self.integral_error = 0
		self.derivative_error = 0
		self.output = 0

	def update(self, current_value):
        # current_value is theta
        # calculate P_term and D_term
		self.error = self.set_point - current_value
		derivative_error = (self.error - self.previous_error) / self.Time_Step
		self.integral_error += self.error * self.Time_Step
		P_term = self.Kp*self.error
		I_term = self.Ki*self.integral_error
		D_term = self.Kd*derivative_error
		output = P_term +I_term + D_term
		self.previous_error = self.error
		return output
	
	def setPoint(self, set_point):
		self.set_point = set_point
		self.previous_error = 0

	def setPID(self, P, I, D):
		self.Kp = P
		self.Ki = I
		self.Kd = D
	
class Turtlebot():
	def __init__(self):
		rospy.init_node("turtlebot_move")
		#rospy.loginfo("Press Ctrl + C to terminate")
		self.control = Controller()
		self.vel = Twist()
		# new line above for obstacle override velocity
		self.vel_obs = Twist()
		self.vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
		self.rate = rospy.Rate(10)
		self.goalx = 0
		self.goaly = 0
		self.isObstacle = False
		self.direction = 1
		# Variables that will be used for hit point calculation
		self.hit_pointx = 0
		self.hit_pointy = 0
		self.hit_goal_dist = 0
		self.curr_goal_ydist = 0
		self.hitpoint = 0

		self.inp1 = input("Choose 'PID' or 'PD' for the controller: " )
		self.inp2 = input("Choose 'Bug2' or 'MBug' for the algorithm:")

		self.left_dist = 999999.9 # Left
		self.leftfront_dist = 999999.9 # Left-front
		self.front_dist = 999999.9 # Front
		self.rightfront_dist = 999999.9 # Right-front
		self.right_dist = 999999.9 # Right
 
		
		# reset odometry to zero
		# if subscribe to odom is before this, robot will not move if this section is commented
		self.reset_pub = rospy.Publisher("reset/std_msgs", Empty, queue_size=10)
		# you can also replace the quotes with mobile_base/commands/reset_odometry
		for i in range(10):
			self.reset_pub.publish(Empty())
			self.rate.sleep()

		# subscribe to odometry
		self.pose = Pose2D()
		self.logging_counter = 0
		self.trajectory = list()
		#rospy.loginfo("created traj")
		self.odom_sub = rospy.Subscriber("odom", Odometry, self.odom_callback)

		self.scan_sub = rospy.Subscriber("/scan", LaserScan, self.callback)  # Subscriber object which will listen "LaserScan" type messages
                                                      # from the "/scan" Topic and call the "callback" function
						      # each time it reads something from the Topic

		try:
			self.run()
		except rospy.ROSInterruptException:
			rospy.loginfo("Action terminated.")
		finally:
			# save trajectory into csv file
			rospy.loginfo("Sending csv")
			np.savetxt('trajectory.csv', np.array(self.trajectory), delimiter=',', fmt='%f')

	def run(self):
		# status of the segment we want to travel on, did we reach the goal (1) or not (0)
		seg1stat = 0
		# status of whether linear velocity will start or not
		velstart = 0
		# variable to track the distance traveled
		dist = 0
		# initial x and y, which are outside of the is shutdown loop
		old_x = self.pose.x
		old_y = self.pose.y
		# goal destination of (4,0)
		self.goalx = 4
		self.goaly = 0
		# slope and y-intercept calculation of the line from the intial position to the goal destination
		slope = (self.goaly - old_y) / (self.goalx - old_x)
		yint = self.goaly - slope*old_x
		hitpoint = 0

		# Check that the user inputted the right controller
		if self.inp1 == "PID":
			kp = 1
			ki = 0.03
			kd = 0.005
			kp_g = 0.5
			ki_g = 0.03
			kd_g = 0.005

		elif self.inp1 == "PD":
			kp = 1
			ki = 0
			kd = 0.005
			kp_g = 0.5
			ki_g = 0
			kd_g = 0.005

		else:
			print("Incorrect Controller Input!")
			rospy.signal_shutdown("Incorrect Controller Input!")

		# Check that the user inputted the right algorithm
		if self.inp2 == "Bug2":
			alg = 1
		elif self.inp2 == "MBug":
			alg = 2
		else:
			print("Incorrect Algorithm Input!")
			rospy.signal_shutdown("Incorrect Algorithm Input!")

		while not rospy.is_shutdown():
			while seg1stat == 0:
				new_x = self.pose.x
				new_y = self.pose.y
				# We calculate the desired heading angle or "direction" for theta
				desdir = atan2((self.goaly - new_y),(self.goalx - new_x))
				# We also calculate the distance between current position and the goal
				pos_error = sqrt((self.goalx - new_x)**2 + (self.goaly - new_y)**2)

				Controller.setPoint(self.control,desdir)
				Controller.setPID(self.control, kp , ki, kd)

				# Bug 2 Algorithm
				if alg == 1:
					# Before starting linear velocity, we change the heading
					while velstart == 0:
						# Asking the controller for recommended heading
						rec = Controller.update(self.control, self.pose.theta)
						rospy.loginfo("Rotating before start:" + str(rec))
						# Implementing the heading recommendation
						self.vel.angular.z = rec
						# We start with no forward movement at the robot starting point
						self.vel.linear.x = 0
						self.vel_pub.publish(self.vel)
						# We wait until heading is within a +/- 0.04 threshold of the desired heading
						if ((self.pose.theta < (desdir+0.04)) and (self.pose.theta > (desdir-0.04))):
							# If yes, we start forward velocity
							velstart = 1

					# We check our position error with respect to a +/- 0.1 threshold
					if pos_error > 0.1:
						# Update the controller recommendation for heading
						rec = Controller.update(self.control, self.pose.theta)
						#rospy.loginfo("Recommended from PID:" + str(rec))

						# Check if an obstacle was detected
						if self.isObstacle:

							self.curr_goal_ydist = sqrt((new_y - (new_x*slope + yint))**2)

							# Check if we are on the goal line within 0.05 threshold when we encounter this obstacle
							if self.curr_goal_ydist < 0.1:
								# If yes, check if we have logged a hit point, if not log one
								if self.hitpoint == 0:
									self.hit_pointx = new_x
									self.hit_pointy = new_y

									# Record distance from hit point to goal
									self.hit_goal_dist = sqrt((self.goalx - self.hit_pointx)**2 + (self.goaly - self.hit_pointy)**2)

									self.hitpoint = 1
									rospy.loginfo("Hit Point")
									rospy.loginfo(self.isObstacle)

								# If hitpoint was already 1, it means we are still rotating to get around a previous obstacle while still on our goal line
							
							# If we are not on the goal line, likely travelling around a previously detected obstacle

							# Use the angular and linear velocities from callback to move around the obstacle
							self.vel.angular.z = self.vel_obs.angular.z
							self.vel.linear.x = self.vel_obs.linear.x


						# If no obstacle
						else:

							self.curr_goal_ydist = sqrt((new_y - (new_x*slope + yint))**2)


							# Check if we are on the goal line
							if self.curr_goal_ydist < 0.1:
								# If yes, do we have a hitpoint (previously encountered obstacle on the goal line)?
								if self.hitpoint == 1:

									# determine if this "leave" point is closer to the goal than the hitpoint
									if pos_error < self.hit_goal_dist:
										# if yes, clear the hitpoint so we continue happy path 
										self.hitpoint = 0
										rospy.loginfo("Clear")

									# If its not closer, that means we went backwards and are crossing the goal line, we will likely detect the obstacle again when we turn

								# No previous hitpoint means we are just continuing on our happy path

							# Not on the goal line and no obstacle means we need to travel to the goal line, we can just stay on the current x and move to the y position
							else:
								# Define the y point that we need to travel towards to get back on the goal line
								rospy.loginfo("Going back to goal line")
								rospy.loginfo(self.hitpoint)
								interim_goaly = new_x*slope + yint

								# Find the desired heading to get to that y point
								desdir_g = atan2((interim_goaly - new_y),(self.pose.x - new_x))

								# Make this point our new controller set point, note that once we get onto the goal line we exit this condition
								# and we will go back to the controller set point of the actual destination
								Controller.setPoint(self.control,desdir_g)
								Controller.setPID(self.control, kp_g , ki_g, kd_g)

								# Get the recommendation from the controller for the interim y goal
								rec = Controller.update(self.control, self.pose.theta)


							# Continue following recommendation from controller to stay/get on the goal line
							self.vel.angular.z = rec
							self.vel.linear.x = self.vel_obs.linear.x
					
					# If we are at the target, stop and update seg1stat to exit loop
					else:
						self.vel.linear.x = 0
						self.vel.angular.z = 0
						seg1stat = 1

				if alg == 2:
					# Fill in with MBug
					seg1stat = 1

				# Continuously aggregating the distance traveled
				dist = dist + sqrt((new_x-old_x)**2 + (new_y-old_y)**2)
				old_x = self.pose.x
				old_y = self.pose.y
				self.vel_pub.publish(self.vel)
				#rospy.loginfo(self.vel)
				#rospy.loginfo("running")
				self.rate.sleep()

			# Exited the seg1stat loop, we reached the destination and the program can shut down 
			rospy.loginfo("Distance traveled = " + str(dist))
			rospy.loginfo("finished")
			break

	def callback(self, dt):
		#print ('-------------------------------------------')
		#print("Front")
		#print (dt.ranges[0])
		#print("Right")
		print (dt.ranges[270])
		#print("Back")
		#print (dt.ranges[180])
		#print ('-------------------------------------------')
		self.left_dist = dt.ranges[90]
		self.leftfront_dist = dt.ranges[45]
		self.front30left = dt.ranges[30]
		self.front35left = dt.ranges[35]
		self.front40left = dt.ranges[40]
		self.front25left = dt. ranges[25]
		self.front20left = dt. ranges[20]
		self.front15left = dt. ranges[15]
		self.front5left = dt. ranges[5]
		self.front_dist = dt.ranges[0]
		self.front5right = dt.ranges[355]
		self.front15right = dt.ranges[345]
		self.front20right = dt.ranges[340]
		self.front25right = dt.ranges[335]
		self.front30right = dt.ranges[330]
		self.front35right = dt.ranges[325]
		self.front40right = dt.ranges[320]
		self.rightfront_dist = dt.ranges[315]
		self.right_dist = dt.ranges[270]

		thr1 = 0.35 # Laser scan range threshold
		thr2 = 0.25

		#Constantly scanning for obstacles whether or not obstacle has been detected, then determining turn direction
		
		# if obstacle is in front and open to the left, turn direction is left
		if (self.front_dist < (thr1+0.1) and (self.front20left  > 2*thr1 or self.leftfront_dist > 2*thr1)):
			self.direction = 1

		# if obstacle in front and open to the right, turn direction is right
		elif (self.front_dist < (thr1+0.1) and (self.front20right > 2*thr1 or self.rightfront_dist > 2*thr1)):
			self.direction = -1
			
		# if obstacle slopes to the right, turn direction is left
		elif (self.front5left < self.front_dist/cos(5*180/pi) or self.front15left < self.front_dist/cos(15*180/pi)) and (self.front_dist < (thr1+0.1)
				or self.front5left < (thr1+0.1) or self.front15left < (thr1+0.1)):
			self.direction = -1
			
		# if obstacle slopes to the left, turn direction is right
		elif (self.front5right < self.front_dist/cos(5*180/pi) or self.front15right < self.front_dist/cos(15*180/pi)) and (self.front_dist < (thr1+0.1)
				or self.front5right < (thr1+0.1) or self.front15right < (thr1+0.1)):
			self.direction = 1

		# If we just found or are currently travelling along an obstacle
		if self.hitpoint == 1:

			# Check if the obstacle is still or has appeared in front of us
			if not(self.front_dist>thr1 and self.front15left>thr1 and self.front20left>thr1 and self.front40left>(thr2) and self.front15right>thr1 and self.front20right>thr1 and self.front40right>(thr2)):

				# if there is an obstacle, keep rotating while still
				rospy.loginfo("Just found or turning")
				if self.direction == 1:
					rospy.loginfo("Left")
				if self.direction == -1:
					rospy.loginfo("Right")
				self.vel_obs.angular.z = 0.3*self.direction
				self.vel_obs.linear.x = 0

				# keep reporting an obstacle
				self.isObstacle = True

				#if no, check if the obstacle is to our sides
			elif self.left_dist<thr2 or self.leftfront_dist<thr1 or self.right_dist<thr2 or self.rightfront_dist<thr1:

				#if yes, stop rotating and start moving forward
				rospy.loginfo("Travelling along obstacle")
				self.vel_obs.angular.z = 0
				self.vel_obs.linear.x = 0.2

				# keep reporting an obstacle
				self.isObstacle = True

				# if not in front or at our sides, then we seem to have cleared the obstacle
			else:

				# report no obstacle so the run method can change the hitpoint value and check if this is a leave point
				rospy.loginfo("Seem to have cleared")
				self.isObstacle = False

		# Behavior when hit point is 0, we have not yet encountered an or seemed to have cleared the obstacle
		if self.hitpoint == 0:

			# Check to confirm there is no obstacle in front of us
			if (self.front_dist>thr1 and self.front15left>thr1 and self.front20left>thr1 and self.front40left>thr2 and self.front15right>thr1 and self.front20right>thr1 and self.front40right>thr2):
				
				# There is no obstacle!
				# since this is 0, the run method will use the PID rec value for angular velocity to keep us on the goal line
				self.vel_obs.angular.z = 0.0 
				
				# Move forward
				self.vel_obs.linear.x = 0.2
				
				# Let the run method know there is no obstacle so we continue happy path/leave point trajectory
				self.isObstacle = False

			# if there is an obstacle in front of us
			else:

				rospy.loginfo("Found obstacle")

				# Stop moving forward
				self.vel_obs.linear.x = 0.0
				self.vel_obs.angular.z = 0.3*self.direction

				# Report to the run method to log a new hit point
				self.isObstacle = True




		# If an obstacle is in front of us, turn away
		# If an obstacle is to the side of us, move forward and keep turning away from obstacle if you get closer (don't use rec, use constant)
		# If the obstacle is behind us but not to the side, stop and turn 90 degrees
		# if the obstacle is sloping, keep turning slightly away from the obstacle

		
		# scan for obstacle, log direction we should turn if one is found, stop linear velocity, self.isObstacle = True
		# if none are found, keep driving forward, self.isobstacle = False


		# Constantly scanning for obstacles while an obstacle has not been detected - when detected, save the direction we should turn
		#if not self.isObstacle:
			# if obstacle in front and open to the right, turn right
		#	if (self.front_dist < (thr1+0.1) and (self.front20left  > 2*thr1 or self.leftfront_dist > 2*thr1)):
		#		self.direction = 1
			# if obstacle in front and open to the left, turn left
		#	elif (self.front_dist < (thr1+0.1) and (self.frotn20right > 2*thr1 or self.rightfront_dist > 2*thr1)):
		#		self.direction = -1
			# if obstacle slopes to the right, turn left
		#	elif (self.front5left < self.front_dist/cos(5*180/pi) or self.front15left < self.front_dist/cos(15*180/pi)) and (self.front_dist < (thr1+0.1)
		#			or self.front5left < (thr1+0.1) or self.front15left < (thr1+0.1)):
		#		self.direction = -1
		#	# if obstacle slopes to the left, turn right
		#	elif (self.front5right < self.front_dist/cos(5*180/pi) or self.front15right < self.front_dist/cos(15*180/pi)) and (self.front_dist < (thr1+0.1)
		#			or self.front5right < (thr1+0.1) or self.front15right < (thr1+0.1)):
		#		self.direction = 1
		
		# if no obstacle, drive forward
		#if self.front_dist>thr1 and self.front15left>thr1 and self.front30left>thr2 and self.front15right>thr1 and self.front30right>thr2:
		#	self.vel_obs.linear.x = 0.2
		#	self.vel_obs.angular.z = 0.0 # since this is 0, the run method will use the PID rec value for angular velocity
		#	self.isObstacle = False
		# if there is an obstacle, stop and turn away from it
		#else:
		#	self.vel_obs.linear.x = 0.0 # stop

		#	self.vel_obs.angular.z = 0.2*self.direction # rotate based on direction chosen earlier
		#	self.isObstacle = True

		# if there is still an obstacle at the sides, override turning and keep going straight
		#if self.left_dist<thr2 or self.leftfront_dist<thr1 or self.right_dist<thr2  or self.rightfront_dist<thr1:
		#	self.isObstacle = True


	def odom_callback(self, msg):
		# get pose = (x, y, theta) from odometry topic
		quarternion = [msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
		(roll, pitch, yaw) = tf.transformations.euler_from_quaternion (quarternion)
		self.pose.theta = yaw
		self.pose.x = msg.pose.pose.position.x
		self.pose.y = msg.pose.pose.position.y
		
		# logging once every 100 times (Gazebo runs at 1000Hz; we save it at 10Hz)
		self.logging_counter += 1
		if self.logging_counter == 10:
			#rospy.loginfo("got to if")
			self.logging_counter = 0
			self.trajectory.append([self.pose.x, self.pose.y]) # save trajectory
			#rospy.loginfo("odom: x=" + str(self.pose.x) + "; y=" + str(self.pose.y) + "; theta=" + str(yaw))
			#rospy.loginfo(np.array(self.trajectory))
if __name__ == '__main__':
	whatever = Turtlebot()