#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from mrs_project_simulation.msg import Neighbours
import numpy as np
from typing import List

#subscribea se na odom svakog robota, ide po svim robotima, s obzirom na pose (odom) racuna da li je taj robot unutar tog kruznog isjecka vidnog polja - ako je to je susjed od tog robota i stavlja ga u listu susjeda za tog robota (salje mu odom tog robota), na kraju saljes tom robotu njegove susjede

class CalcNeighboursNode():
    def __init__(self):
        rospy.loginfo("Calc neighbours node started")
        
        PUB_RATE = 10

        #FOV kao kruzni isjecak
        self.r = 1 #TODO: promjenit (ili ostavit)
        
        alpha = 30
        self.theta1 = np.deg2rad(180 + alpha)
        self.theta1 = (self.theta1 + np.pi) % (2 * np.pi) #iz [0,2pi] u [-pi,pi] jer arctan2 vraca vrijednost u intervalu [-pi,pi]
        self.theta2 = np.deg2rad(360 - alpha)
        self.theta2 = (self.theta2 + np.pi) % (2 * np.pi) - np.pi #iz [0,2pi] u [-pi,pi] jer arctan2 vraca vrijednost u intervalu [-pi,pi]

        self.publisher_neighbours = [rospy.Publisher(f"/robot_{i}/neighbours", Neighbours, queue_size=PUB_RATE) for i in range(10)]
        self.odoms: List[Odometry] = [None] * 10 #"prazna" lista od 10 elem

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
        
        self.rate = rospy.Rate(PUB_RATE) #frekvencija kojom publisha poruke, nece affectat to da missas poruke koje dobivas, ovo utjece samo na publishanje

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
            neighbours_dict = {
                    "robot_0": {
                        "neighbours_odom": [],
                    },
                    "robot_1": {
                        "neighbours_odom": [],
                    },
                    "robot_2": {
                        "neighbours_odom": [],
                    },
                    "robot_3": {
                        "neighbours_odom": [],
                    },
                    "robot_4": {
                        "neighbours_odom": [],
                    },
                    "robot_5": {
                        "neighbours_odom": [],
                    },
                    "robot_6": {
                        "neighbours_odom": [],
                    },
                    "robot_7": {
                        "neighbours_odom": [],
                    },
                    "robot_8": {
                        "neighbours_odom": [],
                    },
                    "robot_9": {
                        "neighbours_odom": [],
                    },
                }
            
            if None not in self.odoms: #ako je popunjena lista sa ne None vrijednostima tj popunjena je odom podacima
                for i in range(len(self.odoms)):
                    curr_odom = self.odoms[i]
                    curr_x, curr_y = curr_odom.pose.pose.position.x, curr_odom.pose.pose.position.y 
                    for j in range(len(self.odoms)):
                        if i == j:
                            continue
                        
                        neigh_odom = self.odoms[j] #odom potencijalnog susjeda
                        neigh_x, neigh_y = neigh_odom.pose.pose.position.x, neigh_odom.pose.pose.position.y

                        d = np.sqrt((neigh_x - curr_x)**2 + (neigh_y - curr_y)**2)
                        if d <= self.r: #unutar kruga
                            angle = np.arctan2(neigh_y - curr_y, neigh_x - curr_x) #u intervalu [-pi, pi]
                            if not (angle > self.theta1 and angle < self.theta2): #ako nije u tom rasponu kuteva naci da je unutar FoV
                                neighbours_dict[f"robot_{i}"]["neighbours_odom"].append(self.odoms[j])


            #publish neighours
            for i, pub in enumerate(self.publisher_neighbours):
                msg = Neighbours()
                msg.neighbours_odoms = neighbours_dict[f"robot_{i}"]["neighbours_odom"]
                pub.publish(msg)

            self.rate.sleep()
            pass

if __name__ == '__main__':
    rospy.init_node('calc_neighbours_node')
    try:
        calc_neighbours_node = CalcNeighboursNode()
        calc_neighbours_node.run()
    except rospy.ROSInterruptException: pass
