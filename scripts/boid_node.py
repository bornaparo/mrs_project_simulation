#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
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

		goal_positions = np.array([(1.0,2.0),(2.0,3.0),(2.0,4.5),(4.6,5.0),(5.5,8.0)])  #pozicije kroz koje svaki robot mora proci
		self.goal_positions = np.tile(goal_positions, (10,1))	# array pozicija za svakog od 10 robota

		self.separation_factor = 1.2 #TODO: promjenit
		self.alignment_factor = 1 #TODO: promjenit
		self.cohesion_factor = 10/2 #adjust if needed
		self.mass = 1

		self.neighbours_odoms = []
		self.publisher_vel = rospy.Publisher(f"{rospy.get_name()}/cmd_vel", Twist, queue_size=self.PUB_RATE) #ovo uzima stage i pomice robota
		rospy.Subscriber(f"{rospy.get_name()}/odom", Odometry, self.odom_callback, queue_size=1) #stage publisha na ovaj topic
		rospy.Subscriber(f"{rospy.get_name()}/neighbours", Neighbours, self.neighbours_callback, queue_size=1) #tu calc_neighbours_node publisha odom susjeda od ovog node-a
		self.rate = rospy.Rate(self.PUB_RATE) #frekvencija kojom publisha poruke, nece affectat to da missas poruke koje dobivas, ovo utjece samo na publishanje

	def odom_callback(self, odom_msg):
		self.x = odom_msg.pose.pose.position.x
		self.y = odom_msg.pose.pose.position.y
		self.vel_x = odom_msg.twist.twist.linear.x
		self.vel_y = odom_msg.twist.twist.linear.y

	def neighbours_callback(self, neighbours: Neighbours):
		self.neighbours_odoms = neighbours.neighbours_odoms #Neighbours je lista Odometry poruka

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
	
	def calc_migration_force(curr_point, goal_point):

		"Calculates the force needed for moving to destination point"
		return goal_point - curr_point
	
	def navigation_to_point(self):
		
		navigation_forces = []

		for position, neighbour in enumerate(self.neighbours_odoms):
			
			neighbour_pos_x = neighbour.pose.pose.position.x
			neighbour_pos_y = neighbour.pose.pose.position.y

			navigation_forces.append(self.calc_migration_force(np.array([neighbour_pos_x, neighbour_pos_y]),
													   np.array([self.goal_positions[position][0], self.goal_positions[position][1]])))

			#if np.linalg.norm([neighbour_pos_x - self.goal_positions[position][0], neighbour_pos_y - self.goal_positions[position][1]]) < 0.5:
			if np.sqrt((neighbour_pos_x - self.goal_positions[position][0])**2 + (neighbour_pos_y - self.goal_positions[position][1]**2)) < 0.5:
				if len(self.goal_positions[position]) > 0:
					self.goal_positions[position].pop(0)
					print("Moving on to the next position!")
				
				elif len(self.goal_positions[position]) == 0:
					print(f"Robot number {position+1} has reached the goal.")
			
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
