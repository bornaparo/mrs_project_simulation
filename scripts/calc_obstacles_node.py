#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist, Point32
from sensor_msgs.msg import PointCloud
from nav_msgs.msg import Odometry, OccupancyGrid
import numpy as np
from typing import List

#subscribea se na odom svakog robota, ide po svim robotima, s obzirom na pose (odom) racuna da li je taj robot unutar tog kruznog isjecka vidnog polja - ako je to je susjed od tog robota i stavlja ga u listu susjeda za tog robota (salje mu odom tog robota), na kraju saljes tom robotu njegove susjede

class CalcObstaclesNode():
    def __init__(self):
        rospy.loginfo("Calc obstacles node started")
        
        PUB_RATE = 10

        #FOV kao kruzni isjecak
        self.r = 1 #TODO: promjenit (ili ostavit)
        
        alpha = 30
        self.theta1 = np.deg2rad(180 + alpha)
        self.theta1 = (self.theta1 + np.pi) % (2 * np.pi) #iz [0,2pi] u [-pi,pi] jer arctan2 vraca vrijednost u intervalu [-pi,pi]
        self.theta2 = np.deg2rad(360 - alpha)
        self.theta2 = (self.theta2 + np.pi) % (2 * np.pi) - np.pi #iz [0,2pi] u [-pi,pi] jer arctan2 vraca vrijednost u intervalu [-pi,pi]

        self.publisher = [rospy.Publisher(f"/robot_{i}/obstacles", PointCloud, queue_size=PUB_RATE) for i in range(10)]
        rospy.Subscriber(f"{rospy.get_name()}/map", OccupancyGrid, self.map_callback, queue_size=1) #tu calc_neighbours_node publisha odom susjeda od ovog node-a

        rospy.Subscriber("/robot_0/odom", Odometry, self.robot0_odom_callback, queue_size=1) 
        rospy.Subscriber("/robot_1/odom", Odometry, self.robot1_odom_callback, queue_size=1) 
        rospy.Subscriber("/robot_2/odom", Odometry, self.robot2_odom_callback, queue_size=1)
        rospy.Subscriber("/robot_3/odom", Odometry, self.robot3_odom_callback, queue_size=1)
        rospy.Subscriber("/robot_4/odom", Odometry, self.robot4_odom_callback, queue_size=1)
        rospy.Subscriber("/robot_5/odom", Odometry, self.robot5_odom_callback, queue_size=1)
        rospy.Subscriber("/robot_6/odom", Odometry, self.robot6_odom_callback, queue_size=1)
        rospy.Subscriber("/robot_7/odom", Odometry, self.robot7_odom_callback, queue_size=1)
        rospy.Subscriber("/robot_8/odom", Odometry, self.robot8_odom_callback, queue_size=1)
        rospy.Subscriber("/robot_9/odom", Odometry, self.robot9_odom_callback, queue_size=1)
        
        self.odoms: List[Odometry] = [None] * 10 #"prazna" lista od 10 elem
        
        self.rate = rospy.Rate(PUB_RATE) #frekvencija kojom publisha poruke, nece affectat to da missas poruke koje dobivas, ovo utjece samo na publishanje

    def map_callback(self, grid: OccupancyGrid):
        matrix_data = np.array(grid.data).reshape((grid.info.height, grid.info.width))
        self.obstacles = np.array()
        for i, row in enumerate(matrix_data):
            for j, element in enumerate(row):
                if element == 100:
                    self.obstacles.append(({i}, {j}))

    

    def robot0_odom_callback(self, robot_odom: Odometry):
        self.odoms[0] = robot_odom

    def robot1_odom_callback(self, robot_odom: Odometry):
        self.odoms[1] = robot_odom
    
    def robot2_odom_callback(self, robot_odom: Odometry):
        self.odoms[2] = robot_odom
    
    def robot3_odom_callback(self, robot_odom: Odometry):
        self.odoms[3] = robot_odom

    def robot4_odom_callback(self, robot_odom: Odometry):
        self.odoms[4] = robot_odom

    def robot5_odom_callback(self, robot_odom: Odometry):
        self.odoms[5] = robot_odom

    def robot6_odom_callback(self, robot_odom: Odometry):
        self.odoms[6] = robot_odom

    def robot7_odom_callback(self, robot_odom: Odometry):
        self.odoms[7] = robot_odom

    def robot8_odom_callback(self, robot_odom: Odometry):
        self.odoms[8] = robot_odom

    def robot9_odom_callback(self, robot_odom: Odometry):
        self.odoms[9] = robot_odom

    def run(self):
        while not rospy.is_shutdown():
            nearest_obs = {
                    "robot_0": {
                        np.ndarray(),
                    },
                    "robot_1": {
                        np.ndarray(),
                    },
                    "robot_2": {
                        np.ndarray(),
                    },
                    "robot_3": {
                        np.ndarray(),
                    },
                    "robot_4": {
                        np.ndarray(),
                    },
                    "robot_5": {
                        np.ndarray(),
                    },
                    "robot_6": {
                        np.ndarray(),
                    },
                    "robot_7": {
                        np.ndarray(),
                    },
                    "robot_8": {
                        np.ndarray(),
                    },
                    "robot_9": {
                        np.ndarray(),
                    },
                }
            
            if None not in self.odoms: #ako je popunjena lista sa ne None vrijednostima tj popunjena je odom podacima
                for i in range(len(self.odoms)):
                    new_array = np.ndarray()
                    curr_odom = self.odoms[i]
                    curr_x, curr_y = curr_odom.pose.pose.position.x, curr_odom.pose.pose.position.y 

                    for obst in self.obstacles:
                        d = np.sqrt((obst[0] - curr_x)**2 + (obst[1] - curr_y)**2)
                        if d <= self.r: #unutar kruga
                            angle = np.arctan2(obst[1] - curr_y, obst[0] - curr_x) #u intervalu [-pi, pi]
                            if not (angle > self.theta1 and angle < self.theta2): #ako nije u tom rasponu kuteva naci da je unutar FoV
                                new_array.append(Point32(obst))

                    nearest_obs[f"robot_{i}"] = new_array


            #publish neighours
            for i, pub in enumerate(self.publisher):
                pub.publish(nearest_obs[f"robot_{i}"])

            self.rate.sleep()
            pass

if __name__ == '__main__':
    rospy.init_node('calc_neighbours_node')
    try:
        calc_neighbours_node = CalcObstaclesNode()
        calc_neighbours_node.run()
    except rospy.ROSInterruptException: pass
