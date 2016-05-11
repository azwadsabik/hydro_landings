#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Thu Apr 21 17:19:31 2016

@author: sabik
"""

import numpy as np
import rospy
import tf

from geometry_msgs.msg import Pose
from std_msgs.msg import Bool
from std_msgs.msg import Empty

TARGET_FRAME = "ar_marker_{0}".format(rospy.get_param("/target_marker_id", 12))
DRONE_FRAME = rospy.get_param("/quadrotor_base", "/base_link")

class Searcher(object):
    def __init__(self):
        self.goal_reached = False
        self.target_in_sight = False
        self.degrees = 0
        self.goal = Pose()
        self.searching = True
        self.target = Pose()
        self.goal_publisher = rospy.Publisher("goal", Pose, 
                                              queue_size=1000)
        self.goal_reached_subscriber = rospy.Subscriber("goal_reached", 
                                                        Bool, 
                                                        self.goal_reached_callback)
        self.target_in_sight_subscriber = rospy.Subscriber("target_in_sight",
                                                           Bool,
                                                           self.target_in_sight_callback)
        self.tf_listener = tf.TransformListener()

        self.landing_publisher = rospy.Publisher('/ardrone/land', Empty, queue_size=1000)                

        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            if self.target_in_sight:
                self.assess_target()
            if self.searching:
                self.goal_publisher.publish(self.goal)
            rate.sleep()

        rospy.spin();        

    def assess_target(self):
        try:
            print "Target in Sight!"
            target_in_drone = np.array(self.tf_listener.lookupTransform(DRONE_FRAME, TARGET_FRAME, rospy.Time()))
            drone_in_world = np.array(self.tf_listener.lookupTransform("world", DRONE_FRAME, rospy.Time())) 
            drone_in_world_position = np.array(drone_in_world[0])
            target_in_drone_position = np.array(target_in_drone[0])
            if DRONE_FRAME == "/ardrone_base_link":
                correction = tf.transformations.rotation_matrix(-np.pi/2, [0, 0, 1])[:3, :3]
                target_in_drone_position = correction.dot(target_in_drone_position)
            target_in_world_position = drone_in_world_position + target_in_drone_position
            print target_in_world_position[0:2]
            self.target.position.x = target_in_world_position[0]
            self.target.position.y = target_in_world_position[1]
            self.target.position.z = drone_in_world[0][2]
            err = np.linalg.norm([target_in_world_position[i] - drone_in_world_position[i] for i in range(2)])  
            if (err < 0.10):
                self.target.position.z = 0
                print "Landing!"
                err = np.linalg.norm(np.array(target_in_world_position) - np.array(drone_in_world_position))
                if (err < 50):
                    self.landing_publisher.publish(Empty())
            return True
        except( tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return False

    def goal_reached_callback(self, goal_reached):
#        if not self.searching:
#            self.searching = True
        if self.goal_reached != goal_reached.data:            
            if goal_reached.data == True:
                self.update_goal()    
                self.goal_reached = goal_reached.data
            else:
                self.goal_reached = goal_reached.data

    def target_in_sight_callback(self, target_in_sight):
        self.target_in_sight = target_in_sight.data
        if self.target_in_sight:
            can_pursue = self.assess_target()
            if can_pursue:
                self.update_goal()
                
    def update_goal(self):
        if self.target_in_sight:
            self.goal = self.target
            return
        print "Reached ", self.degrees
        self.degrees += 36
        self.degrees %= 360
        radians = self.degrees*np.pi/180;
        position = (np.cos(radians)*0.5 - 1, np.sin(radians)*0.5, 1.5)
        print "Moving to", self.degrees, "at", position
        goal = Pose()
        goal.position.x, goal.position.y, goal.position.z = position
        self.goal = goal
        
if __name__ == "__main__":
    rospy.init_node('ardrone_searcher')
    searcher = Searcher()
