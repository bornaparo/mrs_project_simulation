#!/usr/bin/env python3
import math
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Point32
from sensor_msgs.msg import PointCloud
from nav_msgs.msg import Odometry
from mrs_project_simulation.msg import Neighbours
import numpy as np

class BoidNode():
	def __init__(self):
		self.PUB_RATE = 10

		#ovo dobiva od odometrije
		self.x = 0 
		self.y = 0 
		self.vel_x = 0 
		self.vel_y = 0 

		self.separation_factor = 1.2 #TODO: promjenit
		self.alignment_factor = 1 #TODO: promjenit
		self.cohesion_factor = 10/2 #adjust if needed
		self.mass = 1

		self.neighbours_odoms = []
		self.publisher_vel = rospy.Publisher(f"{rospy.get_name()}/cmd_vel", Twist, queue_size=self.PUB_RATE) #ovo uzima stage i pomice robota
		rospy.Subscriber(f"{rospy.get_name()}/odom", Odometry, self.odom_callback, queue_size=1) #stage publisha na ovaj topic
		rospy.Subscriber(f"{rospy.get_name()}/neighbours", Neighbours, self.neighbours_callback, queue_size=1) #tu calc_neighbours_node publisha odom susjeda od ovog node-a
		rospy.Subscriber(f"{rospy.get_name()}/obstacles", PointCloud, self.obstacle_callback, queue_size=1) #tu calc_neighbours_node publisha odom susjeda od ovog node-a
		self.rate = rospy.Rate(self.PUB_RATE) #frekvencija kojom publisha poruke, nece affectat to da missas poruke koje dobivas, ovo utjece samo na publishanje

	def odom_callback(self, odom_msg):
		self.x = odom_msg.pose.pose.position.x
		self.y = odom_msg.pose.pose.position.y
		self.vel_x = odom_msg.twist.twist.linear.x
		self.vel_y = odom_msg.twist.twist.linear.y

	def neighbours_callback(self, neighbours: Neighbours):
		self.neighbours_odoms = neighbours.neighbours_odoms #Neighbours je lista Odometry poruka

	def map_callback(self, obstacles: PointCloud):
		self.obstacles = obstacles

	def calc_separation(self) -> np.ndarray:
		#TODO: calculate separation force with respect to neighbours
		#vraca np.ndarray, shape: (2,)
		return np.array([1,1]) * 0 #promjenit

	def calc_alignment(self) -> np.ndarray:
		#TODO: calculate alignment force with respect to neighbours
		#vraca np.ndarray, shape: (2,)
		return np.array([1,1]) * 0 #promjenit

	def calc_cohesion(self) -> np.ndarray:
		"""
			Calculates cohesion force with respect to neighbours

			Returns:
				np.ndarray of shape (2,): cohesion force
		"""
		if len(self.neighbours_odoms) == 0: #if there isn't any neighbour
			return np.zeros([2,])
		
		#find centroid of neighbours
		centroid = np.array([[neighbour.pose.pose.position.x, neighbour.pose.pose.position.y] for neighbour in self.neighbours_odoms])
		centroid = np.mean(centroid, axis=0)

		#calculate force with respect to centroid
		F = np.array([centroid[0] - self.x, centroid[1] - self.y])
		return F #shape: (2,)
	
	def calc_avoidance(self) -> np.ndarray:
		"""
			Calculates obstacle avoidance force

			Returns:
				np.ndarray of shape(2,): avoidance force
		"""
		safety_direction = np.zeros([2,])
		count = 0

        # Calculate repulsive force for each obstacle in sight.
		for obst in self.obstacles:
			obst_position = obst
			d = obst_position.norm()
			obst_position *= -1        # Make vector point away from obstacle.
			obst_position.normalize()  # Normalize to get only direction.
            # Additionally, if obstacle is very close...
			if d < self.avoid_radius:
                # Scale lineary so that there is no force when agent is on the
                # edge of minimum avoiding distance and force is maximum if the
                # distance from the obstacle is zero.
				safety_scaling = -2 * self.max_force / self.avoid_radius * d + 2 * self.max_force
				safety_direction += obst_position * safety_scaling
			count += 1

            # For all other obstacles: scale with inverse square law.
			# obst_position = obst_position / (self.avoid_scaling * d**2)
			# main_direction += obst_position
			#
		if self.obstacles:
            # Calculate the approach vector.
			diff = (self.old_heading - main_direction.arg() + 180) % 360
			if diff >= 180:
				diff -= 360
            # We mustn't allow scaling to be negative.
			side_scaling = max(math.cos(math.radians(diff)), 0)
            # Divicde by number of obstacles to get average.
			main_direction = main_direction / len(self.obstacles) * side_scaling
			safety_direction /= count
			
		rospy.logdebug("avoids*:      %s", main_direction)
        # Final force is sum of two componets.
        # Force is not limited so this rule has highest priority.
		return main_direction + safety_direction

	def run(self):
		while not rospy.is_shutdown():
			#s obzirom na susjede, izracunat separation, alignment, cohesion sile i to pretvorit u brzine onda (integracija), F=m*a, m=1, F=a
			separation = self.calc_separation() * self.separation_factor
			alignment = self.calc_alignment() * self.alignment_factor
			cohesion = self.calc_cohesion() * self.cohesion_factor

			force = separation + alignment + cohesion #ogranicit mozda jos da ne bude veci od nekog max forcea
			a = force / self.mass

			velocity = a * (1/self.PUB_RATE) #T = 1/f
			vel = Twist()
			vel.linear.x = velocity[0]
			vel.linear.y = velocity[1]

			self.publisher_vel.publish(vel)

			self.rate.sleep()
			pass

if __name__ == '__main__':
	# rospy.init_node('boid_node')
	rospy.init_node('robot_0') #debug
	try:
		boid_node = BoidNode()
		boid_node.run()
	except rospy.ROSInterruptException: pass
