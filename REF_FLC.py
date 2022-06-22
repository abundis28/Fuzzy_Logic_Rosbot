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
		self.inputFigureDescriptionREF = [
			(["low",	"flat"], 		0, 		0.25),
			(["low", 	"falling"], 	0.25, 	0.5),
			(["med", 	"rising"], 		0.25, 	0.5),
			(["med", 	"falling"], 	0.5, 	0.75),
			(["high", 	"rising"], 		0.5, 	0.75),
			(["high", 	"flat"], 		0.75, 	1.01)
		]
		# Define rule base with the center of the figures.
		self.ruleBase = {
			"lowlow": [0.1, 1],
			"lowmed": [0.1, 1],
			"lowhigh": [0.1, 1],
			"medlow": [0.3, -1],
			"medmed": [0.3, 0],
			"medhigh": [0.1, 1],
			"highlow": [0.1, -1],
			"highmed": [0.3, -1],
			"highhigh": [0.3, -1]
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
	
	# Checks different ranges of the sensor measurements depending on if front or back 
	# 		sensors were requested.
	# Back sensor is significantly less wide than front sensor because, given that the robot
	#		is moving forwards only, that is the one we would worry about the most.
	def checkRightRange(self, isFrontBool):
		if isFrontBool:
			closestDistance = min(self.laser_ranges[1080:1260])
		else:
			closestDistance = min(self.laser_ranges[900:1080])
		# Protects the sensor against inf values that are too far or too close.
		if closestDistance > 1:
			closestDistance = 1
		if closestDistance <= 0.16:
			closestDistance = 0.05
		return closestDistance
	
	def getDescriptionFigure(self, distance):
		res = []
		# Loop through input figure and append the descriptions in which the 
		# 		measured distance fits.
		for descrip in self.inputFigureDescriptionREF:
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

	def getFiredRules(self, rfsMembers, rbsMembers):
		# Loop through the lists (each list contains pairs of a possible value 
		# 		and its membership value, e.g.: ["low", 0.4]).
		outputCenters = []
		firingStrengths = []
		for frontMember in rfsMembers:
			for backMember in rbsMembers:
				# Create string with fired rule.
				rule = frontMember[0]
				rule += backMember[0]
				# Get minimum membership value of the combination that created the fired rule.
				firingStrengths.append(min(frontMember[1], backMember[1]))
				# Fetch center of output figures to be multiplied by min membership value
				outputCenters.append(self.ruleBase[rule])
		return outputCenters, firingStrengths

	def fuzzify(self):
		# Process front right sensor reading.
		rfsDistance = self.checkRightRange(True)
		rfsDescriptions = self.getDescriptionFigure(rfsDistance)
		rfsMembers = self.getMembershipValues(rfsDescriptions, rfsDistance)

		#Process back right sensor reading.
		rbsDistance = self.checkRightRange(False)
		rbsDescriptions = self.getDescriptionFigure(rbsDistance)
		rbsMembers = self.getMembershipValues(rbsDescriptions, rbsDistance)

		# Get output centers and its related firing strength.
		outputCenters, firingStrengths = self.getFiredRules(rfsMembers, rbsMembers)

		#Calculate outputs to be published to the rosbot.
		xLinearSum = 0
		zAngularSum = 0
		for i in range(len(outputCenters)):
			xLinearSum += outputCenters[i][0] * firingStrengths[i]
			zAngularSum += outputCenters[i][1] * firingStrengths[i]
		self.cmd.linear.x = xLinearSum/sum(firingStrengths)
		self.cmd.angular.z = zAngularSum/sum(firingStrengths)
		
def main(args=None):
	rclpy.init(args=args)
	minimal_publisher = MinimalPublisher()
	rclpy.spin(minimal_publisher)
	
	minimal_publisher.destroy_node()
	rclpy.shutdown()
	
if __name__ == '__main__':
	main()
