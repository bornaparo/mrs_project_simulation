#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from mrs_project_simulation.msg import Neighbours
from typing import List
import numpy as np

PATHS = {
    "simple_maze": [(0,-4), (3,-4), (3,-1.5), (0,-1.5), (-3,-1.5), (-3,1), (0,1), (3,1), (3,3.5), (0,3.5), (-4,3.5)],
    "hard_maze": [(-4,-4), (-4,-2), (-2,0), (4,0), (4,2), (0,2), (0,4)], #mozda jos tocaka dodat
    "empty": [(4,4), (-4,4), (4,-4), (-4,-4)],
    "test": [(-4.3,0), (-3,4.2), (4,3), (4,0), (4.5,-1), (4.5,-2), (3.5,-3), (2.5,-4.2), (0,-4)],
}

class MigrationForceNode():
    def __init__(self):
        PUB_RATE = 10
        self.close_radius = 0.6
        self.shutoff_radius = 0.1
        self.queue_size = 5
        path_param = rospy.get_param('~path_kind', default="simple_maze")
        self.path = PATHS[path_param]
        self.robots_point_index = 0 #to get point from path to which robots are currently travelling to
        self.odoms: List[Odometry] = [None] * 10 #odometry for every robot, we need only position.x i position.y

        
        #dodat publishere slicno kao i u calc_neighbours
        self.publisher_migration_force = [rospy.Publisher(f"/robot_{i}/migration_force", Point, queue_size=PUB_RATE) for i in range(10)]
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
        min_norm = 4
        while not rospy.is_shutdown():
            if None not in self.odoms: #ako je popunjena lista sa ne None vrijednostima tj popunjena je odom podacima
                closeness = 0
                updated = False

                for i, (curr_odom, pub) in enumerate(zip(self.odoms, self.publisher_migration_force)):
                    msg = Point()
                    if self.robots_point_index == len(self.path): #came to the end point (goal), publish force 0
                        msg.x = 0
                        msg.y = 0
                        pub.publish(msg)
                        continue
                    #gets x,y position from robot
                    curr_x, curr_y = curr_odom.pose.pose.position.x, curr_odom.pose.pose.position.y
                    px, py = self.path[self.robots_point_index] #point x and y
                    
                    #create migration force
                    mig_force = np.array([px - curr_x, py - curr_y])
                    if np.linalg.norm(mig_force) < self.close_radius: #came close to the point, get another point
                        closeness += 1
                    if np.linalg.norm(mig_force) < self.shutoff_radius:
                        msg.x = 0
                        msg.y = 0
                        pub.publish(msg)
                        continue
                    if closeness > self.queue_size and not updated:
                        updated = True
                        self.robots_point_index += 1
                        
                    if np.linalg.norm(mig_force) < min_norm and self.robots_point_index != len(self.path) - 1: #if norm is lower than minimum norm and if it is not the last point (because we want to slow down when approaching last point - required from assignment)
                        s = min_norm / np.linalg.norm(mig_force) #scaling factor to get desired norm
                        mig_force *= s

                    msg.x, msg.y = mig_force
                    pub.publish(msg)

            else: #if there are not yet odometry data, publish force 0
                msg = Point()
                msg.x = msg.y = 0
                for pub in self.publisher_migration_force:
                    pub.publish(msg)

            self.rate.sleep()
            pass

if __name__ == '__main__':
    rospy.init_node('migration_force_node')
    try:
        migration_force_node = MigrationForceNode()
        migration_force_node.run()
    except rospy.ROSInterruptException: pass
