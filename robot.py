#!/usr/bin/env python


import rospy
import numpy as np
import tf
import operator
from math import atan2
from sensor_msgs.msg import PointCloud,LaserScan
from geometry_msgs.msg import Twist,Pose
from scipy.spatial import Voronoi, voronoi_plot_2d
import matplotlib.pyplot as plt
from nav_msgs.msg import Odometry
from colorizeVoronoi import createDiagram
from Graph import Vertex,shortest,planning
from tf.transformations import euler_from_quaternion
from collections import defaultdict

x = 0.0
y = 0.0
theta = 0.0
scanValue =[] #Array for the Pointcloud
laserValue = [] #Array for the LaserScan 
listener = tf.TransformListener() # tf transformation

def obstacleDistance():
	#Using LaserScan to compute the distance from the obstacles, we have 3 options:
	# freeway -> no obstacles ahead
	#change Node -> close enough to an obstacle , go to the next voronoi node
	#Change Diagram -> very close to an obstacle, compute a new voronoi diagram
	#Compute the mean for each option and perform the option with the highest mean
	# In case of a tie or the difference of the mean values of changeNode and freeway is small (<0.3), perform the changeNode option
	#return a list of the means
 
	global laserValue
	rospy.Subscriber('/RosAria/sim_lms1xx_1_laserscan',LaserScan,laserCallback)
	rospy.wait_for_message('/RosAria/sim_lms1xx_1_laserscan',LaserScan,timeout=1)

	freeway =0.0
	changeNode =0.0
	changeDiagram =0.0
	for l in laserValue:
		if abs(l) >=2.5:
			freeway= freeway+1
		if abs(l) >1 and abs(l) < 2.5:
			changeNode = changeNode +1
		if abs(l) <1:
			changeDiagram = changeDiagram +1
	
	freeway = float(freeway/len(laserValue))
	changeNode = float(changeNode/len(laserValue))
	changeDiagram = float(changeDiagram/len(laserValue))
	
	if abs(changeNode-freeway)< 0.3:
		changeNode = 1
	return [changeNode,freeway,changeDiagram]

def laserCallback(data):
	#function callback for the LaserScan subscriber, find the laserbeam in front the robot ,the laserbeams which are 10 degrees from right and left of the robot and append them to a  list
	global laserValue
	
	laserValue =[]
	middle = int(len(data.ranges)/2)
	for i in range(middle -10,middle +10):
		laserValue.append(data.ranges[i])

def scanCallback(data):
	#function callback for the PointCloud subscriber,create a list which contains all points from -135 degrees to 135 degrees and append to a list
	global scanValue

	scanValue =[]
	for c in data.points:
		scanValue.append([c.x,c.y])
	

def odometryCallback(msg):
	#function callback for the Odometry subscriber,wait for the tf transformation and get the translations and rotations values,also calculate theta and assign the values to the global variables
	global theta
	global x
	global y
	
	listener.waitForTransform('odom','base_link',rospy.Time(0),rospy.Duration(4.0))
	(trans,rot) = listener.lookupTransform('odom','base_link',rospy.Time(0))
	(roll,pitch,theta) = euler_from_quaternion([rot[0],rot[1],rot[2],rot[3]])
	
	
	x = trans[0]
	y = trans[1]
	

	

def moveRobot(pos,path,vertices,lastPoint,targetTheta,finish):
	
	global laserValue
	
	pub = rospy.Publisher('/RosAria/cmd_vel', Twist,queue_size = 10)
	rate = rospy.Rate(200)

	speed = Twist()

	# Take out from the list the current position and the first voronoi node from the path
	path.pop(0)
	path.pop(1)
	

	for i in path:

		nextPoint = vertices.get(int(i)) #Get the next node from the voronoi diagram 
		print('nextPoint ',nextPoint,' pos ',pos)

		#Go to the next Node
		while np.linalg.norm(nextPoint-pos) >= 0.2:

			pos = [x,y]
			difX = nextPoint[0]-pos[0]
			difY = nextPoint[1]-pos[1]
			angleToGoal = atan2(difY,difX)

			if abs(angleToGoal-theta) > 0.1:
				speed.angular.z = 0.2*(angleToGoal-theta)
			else:
				speed.linear.x = 0.1*np.linalg.norm(nextPoint-pos)


			pub.publish(speed)
			rate.sleep()
		
		#Face the target and perform one of the 3 options
		pos =[x,y]
		difX = lastPoint[0]-pos[0]
		difY = lastPoint[1]-pos[1]
		angleToGoal = atan2(difY,difX)
		while abs(angleToGoal-theta) >0.1:
			pos = [x,y]
			speed.linear.x = 0.0
			speed.angular.z = 0.2*(angleToGoal-theta)
			pub.publish(speed)
	
		plan = obstacleDistance()
		choice = plan.index(max(plan)) # Get the max mean value of the 3 options 
		if  choice == 1: # No obstacles ahead
			#Go to the target point, meanwhile check for any obstacles ahead using LaserScan, if there is an obstacle ahead ,stop moving and create a new voronoi diagram
			while np.linalg.norm(np.asarray(lastPoint)-np.asarray(pos)) >= 0.2:
				stop = False
				scan = rospy.wait_for_message('/RosAria/sim_lms1xx_1_laserscan',LaserScan,timeout=1)
				if scan.ranges  :
					middle = int(len(scan.ranges)/2)
					forwardScan = operator.itemgetter(middle-20,middle,middle+20)(scan.ranges)
					forwardScan = min(forwardScan)
					print(forwardScan)
					if forwardScan > 2:
						speed.linear.x = 0.2*np.linalg.norm(np.asarray(lastPoint)-np.asarray(pos))
					else:
						speed.linear.x = 0.0
						stop = True
					pub.publish(speed)
					rate.sleep()
					pos = [x,y]

					if stop:
						return False
			#When you reach the goal fix the direction to the wanted target theta
			while abs(targetTheta-theta) > 0.1:
				speed.linear.x = 0.0
				speed.angular.z = 0.2*(targetTheta-theta)
				pub.publish(speed)
			return True
			
		elif choice == 0: # Go to the next Node
			continue
		else: # Create a new voronoi diagram
			return False

if __name__ == "__main__":


	rospy.init_node('Robot')

	target = [14,5]
	targetTheta = 1.5708 # 90 degrees
	pos = []
	
	pub = rospy.Publisher('/RosAria/cmd_vel', Twist,queue_size = 10)
	rospy.Subscriber('/RosAria/pose',Odometry,odometryCallback)
	
	#######face the goal#######
	pos = [x,y]
	difX = target[0]-pos[0]
	difY = target[1]-pos[1]
	
	angleToGoal = atan2(difY,difX)
	speed = Twist()
	while abs(angleToGoal-theta) >0.1:
		speed.linear.x = 0.0
		speed.angular.z = 0.2*(angleToGoal-theta)
		pub.publish(speed)
	
	#######face the goal########

	finish = False
	while not finish:
		#Get the pointclouds && laserscan measures
		rospy.Subscriber('/RosAria/sim_lms1xx_1_pointcloud',PointCloud,scanCallback)
		rospy.Subscriber('/RosAria/sim_lms1xx_1_laserscan',LaserScan,laserCallback)

		if len(scanValue)>4: #Length of the points of the pointclound
			vor = Voronoi(np.asarray(scanValue)) #Create the voronoi diagram
			if len(vor.vertices) > 400: #We need small graph for less computation
				continue
			[path,vertices] = planning(pos,target,vor) # Find the path from the current position to the goal
			if len(path)==2:
				continue
			finish = moveRobot(pos,path,vertices,target,targetTheta,finish) # Move the robot 
			pos =[x,y]
		
	rospy.spin()
		


	

