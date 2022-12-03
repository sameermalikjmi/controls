#!/usr/bin/env python
# -*- coding: utf-8 -*-

from math import pi, sqrt, atan2, cos, sin
import numpy as np
import rospy
import tf
from std_msgs.msg import Empty
from sensor_msgs.msg import LaserScan  # LaserScan type message is defined in sensor_msgs
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose2D
from math import pi, atan2


class Controller:
    def __init__(self, P=0.0, D=0.0, set_point=0):
        self.Kp = P
        self.Kd = D
        self.set_point = set_point  # reference (desired value of theta)
        self.previous_error = 0

    def update(self, current_value):
        # current_value is theta
        # calculate P_term and D_term
        error = self.set_point - current_value
        P_term = self.Kp * error
        D_term = self.Kd * (error - self.previous_error)
        self.previous_error = error
        return P_term + D_term

    def setPoint(self, set_point):
        self.set_point = set_point
        self.previous_error = 0

    def setPD(self, P, D):
        self.Kp = P
        self.Kd = D


class Turtlebot():
    def __init__(self):
        rospy.init_node("turtlebot_move")
        rospy.loginfo("Press Ctrl + C to terminate")
        self.control = Controller()
        self.vel = Twist()
        # new line above for obstacle override velocity
        self.vel_obs = Twist()
        self.vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.rate = rospy.Rate(10)
        self.isObstacle = False
        self.direction = 1

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
        rospy.loginfo("created traj")
        self.odom_sub = rospy.Subscriber("odom", Odometry, self.odom_callback)

        self.scan_sub = rospy.Subscriber("/scan", LaserScan,
                                         self.callback)  # Subscriber object which will listen "LaserScan" type messages
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
        seg1stat = 0
        velstart = 0
        dist = 0
        old_x = self.pose.x
        old_y = self.pose.y
        while not rospy.is_shutdown():
            # segment 1 (4,0)
            while seg1stat == 0:
                new_x = self.pose.x
                new_y = self.pose.y
                x1 = 4
                y1 = 0
                desdir = atan2((y1 - self.pose.y), (x1 - self.pose.x))
                Controller.setPoint(self.control, desdir)
                Controller.setPD(self.control, 2.0, 2.0)

                while velstart == 0:
                    rec = Controller.update(self.control, self.pose.theta)
                    rospy.loginfo("Rotating before start:" + str(rec))
                    self.vel.angular.z = rec
                    # we assume no obstacle direclty in front of robot starting point
                    self.vel.linear.x = 0
                    self.vel_pub.publish(self.vel)
                    if ((self.pose.theta < (desdir + 0.04)) and (self.pose.theta > (desdir - 0.04))):
                        velstart = 1

                if [self.pose.x, self.pose.y] != [x1, y1]:
                    rec = Controller.update(self.control, self.pose.theta)
                    rospy.loginfo("Recommended from PD:" + str(rec))
                    if self.vel_obs.angular.z > 0 or self.isObstacle:
                        self.vel.angular.z = self.vel_obs.angular.z
                        self.vel.linear.x = self.vel_obs.linear.x
                    else:
                        self.vel.angular.z = rec
                        self.vel.linear.x = self.vel_obs.linear.x

                # Check if we're at the target
                if (self.pose.y > (y1 - 5) and self.pose.y < (y1 + 5)):
                    if (self.pose.x < (x1 - 0.05)):
                        self.vel.linear.x = self.vel_obs.linear.x
                    else:
                        self.vel.linear.x = 0
                        self.vel.angular.z = 0
                        seg1stat = 1
                else:
                    self.vel.linear.x = 0
                    self.vel.angular.z = 0
                    seg1stat = 1
                dist = dist + sqrt((new_x - old_x) ** 2 + (new_y - old_y) ** 2)
                old_x = self.pose.x
                old_y = self.pose.y
                self.vel_pub.publish(self.vel)
                rospy.loginfo(self.vel)
                rospy.loginfo("running")
                self.rate.sleep()

            # rospy.loginfo(self.vel)
            # rospy.loginfo("running")
            # self.rate.sleep()
            rospy.loginfo("Distance traveled = " + str(dist))
            rospy.loginfo("finished")
            break

    # remove the code above and add your code here to adjust your movement based on 2D pose feedback

    def callback(self, dt):
        print('-------------------------------------------')
        print(dt.ranges[0])
        print(dt.ranges[15])
        print(dt.ranges[345])
        print('-------------------------------------------')
        thr1 = 0.35  # Laser scan range threshold
        thr2 = 0.25

    # Each time we start towards a new goal, we need to calculate the start-goal line
    if self.start_goal_line_calculated == False:
        # Make sure go to goal mode is set.
        self.robot_mode = "go to goal mode"

        self.start_goal_line_xstart = self.pose.x
        self.start_goal_line_xgoal = self.goal_x_coordinates[self.goal_idx]
        self.start_goal_line_ystart = self.pose.y
        self.start_goal_line_ygoal = self.goal_y_coordinates[self.goal_idx]

        # Calculate the slope of the start-goal line m
        self.start_goal_line_slope_m = (
                (self.start_goal_line_ygoal - self.start_goal_line_ystart) / (
                self.start_goal_line_xgoal - self.start_goal_line_xstart))

        # Solve for the intercept b
        self.start_goal_line_y_intercept = self.start_goal_line_ygoal - (
                self.start_goal_line_slope_m * self.start_goal_line_xgoal)

        # We have successfully calculated the start-goal line
        self.start_goal_line_calculated = True

        if self.robot_mode == "go to goal mode":
            self.go_to_goal()
        elif self.robot_mode == "wall following mode":
            self.follow_wall()

    def go_to_goal(self):
        if not self.isObstacle:
            # if obstacle in front and open to the right, turn right
            if (dt.ranges[0] < (thr1 + 0.1) and (dt.ranges[20] > 2 * thr1 or dt.ranges[45] > 2 * thr1)):
                self.direction = 1
            # if obstacle in front and open to the left, turn left
            elif (dt.ranges[0] < (thr1 + 0.1) and (dt.ranges[340] > 2 * thr1 or dt.ranges[315] > 2 * thr1)):
                self.direction = -1
            # if obstacle slopes to the right, turn left
            elif (dt.ranges[5] < dt.ranges[0] / cos(5 * 180 / pi) or dt.ranges[15] < dt.ranges[0] / cos(
                    15 * 180 / pi)) and (dt.ranges[0] < (thr1 + 0.1)
                                         or dt.ranges[5] < (thr1 + 0.1) or dt.ranges[15] < (thr1 + 0.1)):
                self.direction = -1
            # if obstacle slopes to the left, turn right
            elif (dt.ranges[355] < dt.ranges[0] / cos(5 * 180 / pi) or dt.ranges[345] < dt.ranges[0] / cos(
                    15 * 180 / pi)) and (dt.ranges[0] < (thr1 + 0.1)
                                         or dt.ranges[355] < (thr1 + 0.1) or dt.ranges[345] < (thr1 + 0.1)):
                self.direction = 1

        desired_theta = math.atan2(
            self.goal.y - self.pose.y,
            self.goal.x - self.pose.x)

        # How far off is the current heading in radians?
        theta_error = desired_theta - self.pose.theta

        # if there is an obstacle
        if not (dt.ranges[0] > thr1 and dt.ranges[15] > thr1 and dt.ranges[30] > thr2 and dt.ranges[345] > thr1 and \
                dt.ranges[330] > thr2):
            # Change the mode to wall following mode.
            self.robot_mode = "wall following mode"

            # Record the hit point  
            self.hit_point_x = self.current_x
            self.hit_point_y = self.current_y

            # Record the distance to the goal from the 
            # hit point
            self.distance_to_goal_from_hit_point = (
                math.sqrt((
                              pow(self.goal_x_coordinates[self.goal_idx] - self.hit_point_x, 2)) + (
                              pow(self.goal_y_coordinates[self.goal_idx] - self.hit_point_y, 2))))

            # Make a hard left to begin following wall
            self.vel_obs.angular.z = 0.5

            # Send command to the robot
            self.publisher_.publish(msg)
            
            self.isObstacle = True

            # Exit this function        
            return
        
        # if there is no obstacle and heading is bad
        elif math.fabs(theta_error) > 0.1:
           if theta_error > 0:
                # Turn left (counterclockwise)
                self.vel_obs.angular.z = self.turning_speed_yaw_adjustment
            else:
                # Turn right (clockwise)
                self.vel_obs.angular.z = -self.turning_speed_yaw_adjustment
                
        # Else drive straight
        else:
            self.vel_obs.linear.x = 0.2
            self.vel_obs.angular.z = 0.0
            self.isObstacle = False
            
            
        # if there is still an obstacle at the sides, override turning and keep going straight
        if dt.ranges[90] < thr2 or dt.ranges[45] < thr1 or dt.ranges[270] < thr2 or dt.ranges[315] < thr1:
            self.isObstacle = True

    def follow_wall(self):
        """
        This method causes the robot to follow the boundary of a wall.
        """
        # Create a geometry_msgs/Twist message
        msg = Twist()
        self.vel_obs.linear.x = 0.0
        self.vel_obs.linear.y = 0.0
        self.vel_obs.angular.z = 0.0

        # Special code if Bug2 algorithm is activated
        if self.bug2_switch == "ON":

            # Calculate the point on the start-goal 
            # line that is closest to the current position
            x_start_goal_line = self.current_x
            y_start_goal_line = (
                                        self.start_goal_line_slope_m * (
                                    x_start_goal_line)) + (
                                    self.start_goal_line_y_intercept)

            # Calculate the distance between current position 
            # and the start-goal line
            distance_to_start_goal_line = math.sqrt(pow(
                x_start_goal_line - self.current_x, 2) + pow(
                y_start_goal_line - self.current_y, 2))

            # If we hit the start-goal line again               
            if distance_to_start_goal_line < self.distance_to_start_goal_line_precision:

                # Determine if we need to leave the wall and change the mode
                # to 'go to goal'
                # Let this point be the leave point
                self.leave_point_x = self.current_x
                self.leave_point_y = self.current_y

                # Record the distance to the goal from the leave point
                self.distance_to_goal_from_leave_point = math.sqrt(
                    pow(self.goal_x_coordinates[self.goal_idx]
                        - self.leave_point_x, 2)
                    + pow(self.goal_y_coordinates[self.goal_idx]
                          - self.leave_point_y, 2))

                # Is the leave point closer to the goal than the hit point?
                # If yes, go to goal. 
                diff = self.distance_to_goal_from_hit_point - self.distance_to_goal_from_leave_point
                if diff > self.leave_point_to_hit_point_diff:
                    # Change the mode. Go to goal.
                    self.robot_mode = "go to goal mode"

                # Exit this function
                return

                # Logic for following the wall
        # >d means no wall detected by that laser beam
        # <d means an wall was detected by that laser beam
        d = thr1
        
        self.leftfront_dist = (dt.ranges[45] + dt.ranges[90])/2
        self.rightfront_dist = (dt.ranges[270] + dt.ranges[315])/2
        self.front_dist = dt.ranges[0]

        if self.leftfront_dist > d and self.front_dist > d and self.rightfront_dist > d:
            self.wall_following_state = "search for wall"
            self.vel_obs.linear.x = 0.2
            self.vel_obs.angular.z = -0.25  # turn right to find wall

        elif self.leftfront_dist > d and self.front_dist < d and self.rightfront_dist > d:
            self.wall_following_state = "turn left"
            self.vel_obs.angular.z = 0.5


        elif (self.leftfront_dist > d and self.front_dist > d and self.rightfront_dist < d):
            if (self.rightfront_dist < self.dist_too_close_to_wall):
                # Getting too close to the wall
                self.wall_following_state = "turn left"
                self.vel_obs.linear.x = 0.2
                self.vel_obs.angular.z = 0.5
            else:
                # Go straight ahead
                self.wall_following_state = "follow wall"
                self.vel_obs.linear.x = 0.2

        elif self.leftfront_dist < d and self.front_dist > d and self.rightfront_dist > d:
            self.wall_following_state = "search for wall"
            self.vel_obs.linear.x = 0.2
            self.vel_obs.angular.z = -0.25  # turn right to find wall

        elif self.leftfront_dist > d and self.front_dist < d and self.rightfront_dist < d:
            self.wall_following_state = "turn left"
            self.vel_obs.angular.z = 0.5

        elif self.leftfront_dist < d and self.front_dist < d and self.rightfront_dist > d:
            self.wall_following_state = "turn left"
            self.vel_obs.angular.z = 0.5

        elif self.leftfront_dist < d and self.front_dist < d and self.rightfront_dist < d:
            self.wall_following_state = "turn left"
            self.vel_obs.angular.z = 0.5

        elif self.leftfront_dist < d and self.front_dist > d and self.rightfront_dist < d:
            self.wall_following_state = "search for wall"
            self.vel_obs.linear.x = 0.2
            self.vel_obs.angular.z = -0.25  # turn right to find wall

        else:
            pass

    def odom_callback(self, msg):
        # get pose = (x, y, theta) from odometry topic
        quarternion = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z,
                       msg.pose.pose.orientation.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quarternion)
        self.pose.theta = yaw
        self.pose.x = msg.pose.pose.position.x
        self.pose.y = msg.pose.pose.position.y

        # logging once every 100 times (Gazebo runs at 1000Hz; we save it at 10Hz)
        self.logging_counter += 1
        if self.logging_counter == 10:
            rospy.loginfo("got to if")
            self.logging_counter = 0
            self.trajectory.append([self.pose.x, self.pose.y])  # save trajectory
            rospy.loginfo("odom: x=" + str(self.pose.x) + "; y=" + str(self.pose.y) + "; theta=" + str(yaw))
            rospy.loginfo(np.array(self.trajectory))


if __name__ == '__main__':
    whatever = Turtlebot()