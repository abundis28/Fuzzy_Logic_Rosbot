from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data
import time

class MinimalPublisher(Node):
	def __init__(self):
		super().__init__('forward_demo')
		self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
		self.cmd = Twist()
		# Period of time to allow robot to be placed in the arena.
		time.sleep(10)
		# Reduced timer to execute timer_callback faster.
		timer_period = 0.2 # seconds
		self.timer = self.create_timer(timer_period, self.timer_callback)
		# Variable used to make sure laser_callback has been executed befor trying to 
		#		access the laser_ranges measurements.
		self.i = 0
		self.scan_sub = self.create_subscription(LaserScan, 'scan', self.laser_callback, qos_profile_sensor_data)
		# Define input figures by possible value and its corresponding shapes and their 
		# 		start/end.
		self.inputFigureDescriptionOA = [
			(["low",	"flat"],	    0, 		0.2),
			(["low", 	"falling"], 	0.2, 	0.4),
			(["med", 	"rising"], 	    0.2, 	0.4),
			(["med", 	"falling"], 	0.4, 	0.65),
			(["high", 	"rising"],		0.4, 	0.65),
			(["high", 	"flat"],		0.65, 	1.01)
		]
		# Define rule base with the center of the figures.
		self.ruleBaseOA = {
			"lowlowlow": 	[0.005, 	-1],
			"lowlowmed": 	[0.095, 	-1],
			"lowlowhigh": 	[0.095, 	-1],
			"lowmedlow": 	[0.005,		0],
			"lowmedmed": 	[0.095, 	-1],
			"lowmedhigh": 	[0.095, 	-1],
			"lowhighlow": 	[0.005, 	0],
			"lowhighmed": 	[0.095, 	-1],
			"lowhighhigh":	[0.095, 	-1],
			"medlowlow": 	[0.095, 	1],
			"medlowmed": 	[0.095, 	1],
			"medlowhigh": 	[0.095, 	-1],
			"medmedlow": 	[0.095, 	1],
			"medmedmed": 	[0.3, 		0],
			"medmedhigh": 	[0.3, 		-1],
			"medhighlow": 	[0.095, 	1],
			"medhighmed": 	[0.3, 		0],
			"medhighhigh":	[0.095, 	-1],
			"highlowlow": 	[0.095, 	1],
			"highlowmed": 	[0.095, 	1],
			"highlowhigh":	[0.095, 	1],
			"highmedlow": 	[0.095, 	1],
			"highmedmed": 	[0.095, 	1],
			"highmedhigh":	[0.095, 	0],
			"highhighlow":	[0.095, 	1],
			"highhighmed":	[0.095, 	1],
			"highhighhigh": [0.3, 		0]
		}
	
	# Takes readings from LID sensor.
	def laser_callback(self, msg):
		self.laser_ranges = msg.ranges
		self.i += 1
	
	# Executes fuzzification process and publishes message with outputs for rosbot to apply.
	def timer_callback(self):
		if self.i > 0:
			self.fuzzify()
			self.publisher_.publish(self.cmd)
	
	# Checks different ranges of the sensor measurements depending on if right, middle or left 
	# 		sensors in the front were requested.
	def checkFrontRange(self, charChooseSensor):
		if charChooseSensor == 'R':
			closestDistance = min(self.laser_ranges[1200:1380])
		elif charChooseSensor == 'M':
			closestDistance = min(min(self.laser_ranges[1380:1440]),min(self.laser_ranges[0:60]))
		elif charChooseSensor == 'L':
			closestDistance = min(self.laser_ranges[60:240])
		if closestDistance <= 0.16:
			closestDistance = 0.05
		if closestDistance > 1:
			closestDistance = 1
		return closestDistance

	def getDescriptionFigure(self, distance):
		# Loop through input figure and append the descriptions in which the 
		# 		measured distance fits.
		res = []
		for descrip in self.inputFigureDescriptionOA:
			if distance < descrip[2] and distance >= descrip[1]:
				res.append(descrip)
		return res
	
	def getMembershipValues(self, descriptions, distance):
		# Loop through the descriptions and calculate the membership value for 
		# 		each possible figure shape: "flat", "falling", "rising". Append the possible
		# 		and its membership value.
		res = []
		for description in descriptions:
			if description[0][1] == "flat":
				res.append([description[0][0], 1])
			elif description[0][1] == "rising":
				res.append([description[0][0], self.calcMembership([description[1],description[2]], distance, True)])
			elif description[0][1] == "falling":
				res.append([description[0][0], self.calcMembership([description[1], description[2]], distance, False)])
		return res
	
	def calcMembership(self, positions, x, isRisingBool):
		# Calculate membership value according to the diretion of the slope.
		if isRisingBool:
			a = positions[0]
			b = positions[1]
			return (x-a)/(b-a)
		else:
			d = positions[1]
			c = positions[0]
			return (d-x)/(d-c)

	def getFiredRulesOA(self, flsMembers, fmsMembers, frsMembers):
		# Loop through the lists (each list contains pairs of a possible value 
		# 		and its membership value, e.g.: ["low", 0.4]).
		outputCenters = []
		firingStrenghts = []
		for leftMember in flsMembers:
			for middleMember in fmsMembers:
				for rightMember in frsMembers:
					rule = leftMember[0]
					rule += middleMember[0]
					rule += rightMember[0]
					firingStrenghts.append(min(leftMember[1], middleMember[1], rightMember[1]))
					outputCenters.append(self.ruleBaseOA[rule])
		return outputCenters, firingStrenghts

	def fuzzify(self):
		# Get all three front readings.
		flsDistance = self.checkFrontRange('L')
		fmsDistance = self.checkFrontRange('M')
		frsDistance = self.checkFrontRange('R')

		# Process front left sensor reading.
		flsDescriptions = self.getDescriptionFigure(flsDistance)
		flsMembers = self.getMembershipValues(flsDescriptions, flsDistance)

		# Process front middle sensor reading.
		fmsDescriptions = self.getDescriptionFigure(fmsDistance)
		fmsMembers = self.getMembershipValues(fmsDescriptions, fmsDistance)
		
		# Process front right sensor reading.
		frsDescriptions = self.getDescriptionFigure(frsDistance)
		frsMembers = self.getMembershipValues(frsDescriptions, frsDistance)

		# Get output centers and its related firing strength.
		outputCenters, firingStrenghts = self.getFiredRulesOA(flsMembers, fmsMembers, frsMembers)

		#Calculate outputs to be published to the rosbot.
		xLinearSum = 0
		zAngularSum = 0
		for i in range(len(outputCenters)):
			xLinearSum += outputCenters[i][0] * firingStrenghts[i]
			zAngularSum += outputCenters[i][1] * firingStrenghts[i]
		totalMembVal = sum(firingStrenghts)
		if totalMembVal > 0:
			self.cmd.linear.x = xLinearSum/totalMembVal
			self.cmd.angular.z = zAngularSum/totalMembVal
		else:
			self.cmd.linear.x = 0.0
			self.cmd.angular.z = 0.0
		
def main(args=None):
	rclpy.init(args=args)
	minimal_publisher = MinimalPublisher()
	rclpy.spin(minimal_publisher)
	
	minimal_publisher.destroy_node()
	rclpy.shutdown()
	
if __name__ == '__main__':
	main()
