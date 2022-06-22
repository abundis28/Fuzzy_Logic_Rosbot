#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data
import time

class MinimalPublisher(Node):
	def __init__(self):
		super().__init__('forward_demo')
		self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
		self.cmd = Twist()
		time.sleep(15)
		timer_period = 0.2 # seconds
		self.timer = self.create_timer(timer_period, self.timer_callback)
		self.scan_sub = self.create_subscription(LaserScan, 'scan', self.laser_callback, qos_profile_sensor_data)
		self.i = 0
		self.errorList = []
		self.errorTotal = 0
	
	def checkRightRange(self):
		# Get minimum laser measurement in the range.
		# 		Note that the range is heavily biased to the upper segment
		# 		of the right-side measurements. This should help the robot,
		# 		react better to mo
		closest = min(self.laser_ranges[900:1400])
		if closest > 1:
			closest = 1
		if closest <= 0.16:
			closest = 0.16
		return closest

	def PID(self, kP, kI, kD):
		# Define linear speed to be used regardless of PID functioning.
		self.cmd.linear.x = 0.25
		# Define desired distance with a little else to provide breathing 
		# 		room against oscillation/overshoot.
		desiredDistanceRight = 0.55
		# Get minimum distance to the right.
		actualDistance = self.checkRightRange()
		# Calculate base error.
		error = desiredDistanceRight - actualDistance
		# Accumulate error for integrative part of PID.
		self.errorTotal += error
		# Get last error for derivative part of PID.
		lastError = 0
		if self.errorList:
			lastError = self.errorList.pop()
		# Calculate each part of the PID values.
		proportional = kP * error
		integral = kI * self.errorTotal
		derivative = kD * (error-lastError)
		# Add the individual PID components and assign them to steering 
		# 		component of cmd publication.
		self.cmd.angular.z = proportional + integral + derivative
		# Store current error for the derivative component of next 
		# 		iteration.
		self.errorList.append(error)
	
	def laser_callback(self, msg):
		self.laser_ranges = msg.ranges
		# Increment i to secure that laser measurements have been taken
		# 		before running PID controller and trying to acces them.
		self.i += 1
	
	def timer_callback(self):
		# Check laser measurements have been taken before running PID 
		# 		controller.
		if self.i > 0:
			# Execute PID controller.
			self.PID(0.5, 0.0001, 0.85)
			# Publish message with xLinear and zAngular for rosbot to move.
			self.publisher_.publish(self.cmd)
	
def main(args=None):
	rclpy.init(args=args)
	minimal_publisher = MinimalPublisher()
	rclpy.spin(minimal_publisher)
	
	minimal_publisher.destroy_node()
	rclpy.shutdown()
	
if __name__ == '__main__':
	main()
