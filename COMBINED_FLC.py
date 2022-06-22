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
			(["low", 	"falling"], 	0.25, 	0.45),
			(["med", 	"rising"], 		0.25, 	0.45),
			(["med", 	"falling"], 	0.45, 	0.75),
			(["high", 	"rising"], 		0.45, 	0.75),
			(["high", 	"flat"], 		0.75, 	1.01)
		]
		self.inputFigureDescriptionOA = [
			(["low",	"flat"],	    0, 		0.2),
			(["low", 	"falling"], 	0.2, 	0.4),
			(["med", 	"rising"], 	    0.2, 	0.4),
			(["med", 	"falling"], 	0.4, 	0.65),
			(["high", 	"rising"],		0.4, 	0.65),
			(["high", 	"flat"],		0.65, 	1.01)
		]
		self.inputFigureDescriptionGEN = [
			(["oa", 	"flat"], 		0, 		0.4),
			(["oa",		"falling"],	    0.4, 	0.6),
			(["ref", 	"rising"], 		0.4, 	0.6),
			(["ref", 	"flat"], 		0.6, 	1.01)
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
			"highhighmed":	[0.3, 		1],
			"highhighhigh": [0.3, 		0]
		}
		self.ruleBaseREF = {
			"lowlow": 	[0.1, 1],
			"lowmed": 	[0.1, 1],
			"lowhigh": 	[0.1, 1],
			"medlow": 	[0.3, -1],
			"medmed": 	[0.3, 0],
			"medhigh": 	[0.1, 1],
			"highlow": 	[0.1, -1],
			"highmed": 	[0.3, -1],
			"highhigh": [0.3, -1]
		}

	# Takes readings from LID sensor.
	def laser_callback(self, msg):
		self.laser_ranges = msg.ranges
		self.i += 1
	
	# Executes fuzzification process and publishes message with outputs for rosbot to apply.
	def timer_callback(self):
		if self.i > 0:
			self.fuzzifyGEN()
			self.publisher_.publish(self.cmd)
	
	# Checks different ranges of the sensor measurements depending on if right, middle or left 
	# 		sensors in the front were requested.
	def checkFrontRange(self, charChooseSensor, boolIsGeneral):
		if charChooseSensor == 'M':
			closestDistance = min(min(self.laser_ranges[1380:1440]),min(self.laser_ranges[0:60]))
		elif charChooseSensor == 'L':
			closestDistance = min(self.laser_ranges[60:240])
		if charChooseSensor == 'R':
			if boolIsGeneral:
				closestDistance = min(self.laser_ranges[1260:1380])
			else:
				closestDistance = min(self.laser_ranges[1180:1380])
		if closestDistance > 1:
			closestDistance = 1
		if closestDistance <= 0.16:
			closestDistance = 0.05
		return closestDistance
	
	# Checks different ranges of the sensor measurements depending on if front or back 
	# 		sensors were requested.
	# Back sensor is significantly less wide than front sensor because, given that the robot
	#		is moving forwards only, that is the one we would worry about the most.
	def checkRightRange(self, isFrontBool):
		if isFrontBool:
			closestDistance = min(self.laser_ranges[1080:1440])
		else:
			closestDistance = min(self.laser_ranges[900:1080])
		if closestDistance > 1:
			closestDistance = 1
		if closestDistance <= 0.16:
			closestDistance = 0.05
		return closestDistance
	
	def getDescriptionFigure(self, distance, stringTypeFuzzify):
		# Loop through input figure (depending on which fuzzification was requested) and 
		# 		append the descriptions in which the measured distance fits.
		res = []
		if stringTypeFuzzify == "OA":
			figureDescription = self.inputFigureDescriptionOA
		elif stringTypeFuzzify == "REF":
			figureDescription = self.inputFigureDescriptionREF
		elif stringTypeFuzzify == "GEN":
			figureDescription = self.inputFigureDescriptionGEN
		for descrip in figureDescription:
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
		firingStrengths = []
		for leftMember in flsMembers:
			for middleMember in fmsMembers:
				for rightMember in frsMembers:
					rule = leftMember[0]
					rule += middleMember[0]
					rule += rightMember[0]
					firingStrengths.append(min(leftMember[1], middleMember[1], rightMember[1]))
					outputCenters.append(self.ruleBaseOA[rule])
		return outputCenters, firingStrengths

	def getFiredRulesREF(self, rfsMembers, rbsMembers):
		# Loop through the lists (each list contains pairs of a possible value 
		# 		and its membership value, e.g.: ["low", 0.4]).
		outputCenters = []
		firingStrengths = []
		for frontMember in rfsMembers:
			for backMember in rbsMembers:
				rule = frontMember[0]
				rule += backMember[0]
				firingStrengths.append(min(frontMember[1], backMember[1]))
				outputCenters.append(self.ruleBaseREF[rule])
		return outputCenters, firingStrengths

	def getFiringStrenghtsGEN(self, genMembers):
		fsOA = 0
		fsREF = 0
		for member in genMembers:
			if member[0] == "oa":
				fsOA = member[1]
			elif member[0] == "ref":
				fsREF = member[1]
		return fsOA, fsREF

	def fuzzifyREF(self):
		# Get both readings at the beggining.
		rfsDistance = self.checkRightRange(True)
		rbsDistance = self.checkRightRange(False)

		# Process right front reading.
		rfsDescriptions = self.getDescriptionFigure(rfsDistance, "REF")
		rfsMembers = self.getMembershipValues(rfsDescriptions, rfsDistance)

		# Process right back reading.
		rbsDescriptions = self.getDescriptionFigure(rbsDistance, "REF")
		rbsMembers = self.getMembershipValues(rbsDescriptions, rbsDistance)

		# Get output centers and its related firing strength.
		outputCenters, firingStrengths = self.getFiredRulesREF(rfsMembers, rbsMembers)

		# Calculate outputs to be used in higher deffuzification.
		xLinearSum = 0
		zAngularSum = 0
		for i in range(len(outputCenters)):
			xLinearSum += outputCenters[i][0] * firingStrengths[i]
			zAngularSum += outputCenters[i][1] * firingStrengths[i]
		return [xLinearSum/sum(firingStrengths), zAngularSum/sum(firingStrengths)]

	def fuzzifyOA(self):
		# Get all three front readings.
		flsDistance = self.checkFrontRange('L', False)
		fmsDistance = self.checkFrontRange('M', False)
		frsDistance = self.checkFrontRange('R', False)

		# Process front left sensor reading.
		flsDescriptions = self.getDescriptionFigure(flsDistance, "OA")
		flsMembers = self.getMembershipValues(flsDescriptions, flsDistance)

		# Process front middle sensor reading.
		fmsDescriptions = self.getDescriptionFigure(fmsDistance, "OA")
		fmsMembers = self.getMembershipValues(fmsDescriptions, fmsDistance)
		
		# Process front right sensor reading.
		frsDescriptions = self.getDescriptionFigure(frsDistance, "OA")
		frsMembers = self.getMembershipValues(frsDescriptions, frsDistance)

		# Get output centers and its related firing strength.
		outputCenters, firingStrenghts = self.getFiredRulesOA(flsMembers, fmsMembers, frsMembers)

		# Calculate outputs to be used in higher deffuzification.
		xLinearSum = 0
		zAngularSum = 0
		for i in range(len(outputCenters)):
			xLinearSum += outputCenters[i][0] * firingStrenghts[i]
			zAngularSum += outputCenters[i][1] * firingStrenghts[i]
		totalMembVal = sum(firingStrenghts)
		if totalMembVal > 0:
			xLinear = xLinearSum/totalMembVal
			zAngular = zAngularSum/totalMembVal
		else:
			xLinear = 0.0
			zAngular = 0.0

		return [xLinear, zAngular]

	def fuzzifyGEN(self):
		# Get all three front readings and choose the closest one.
		flsDistance = self.checkFrontRange('L', True)
		fmsDistance = self.checkFrontRange('M', True)
		frsDistance = self.checkFrontRange('R', True)
		minDistance = min(flsDistance, fmsDistance, frsDistance)

		# Process chosen distance in higher deffuzification.
		genDescriptions = self.getDescriptionFigure(minDistance, "GEN")
		genMembers = self.getMembershipValues(genDescriptions, minDistance)

		# Get firing strengths for OA and REF.
		fsOA, fsREF = self.getFiringStrenghtsGEN(genMembers)
		self.cmd.linear.x = 0.0
		self.cmd.angular.z = 0.0
		# Based on the firing strengths decided wether to execute each
		# 		fuzzification process.
		if fsOA > 0:
			# If yes, multiply outputs times the firing strength and accumulate
			#		in xLinear and zAngular.
			outputsOA = self.fuzzifyOA()
			self.cmd.linear.x += outputsOA[0] * fsOA
			self.cmd.angular.z += outputsOA[1] * fsOA
		if fsREF > 0:
			# If yes, multiply outputs times the firing strength and accumulate
			#		in xLinear and zAngular.
			outputsREF = self.fuzzifyREF()
			self.cmd.linear.x += outputsREF[0] * fsREF
			self.cmd.angular.z += outputsREF[1] * fsREF
		
def main(args=None):
	rclpy.init(args=args)
	minimal_publisher = MinimalPublisher()
	rclpy.spin(minimal_publisher)
	
	minimal_publisher.destroy_node()
	rclpy.shutdown()
	
if __name__ == '__main__':
	main()
