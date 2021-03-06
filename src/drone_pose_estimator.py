#!/usr/bin/env python

# -*- coding: utf-8 -*-
"""
Created on Wed Apr 20 10:29:35 2016

@author: Azwad Sabik
"""

import numpy as np
import rospy
import tf
import time

from ar_track_alvar_msgs.msg import AlvarMarkers
from ardrone_autonomy.msg import Navdata
from geometry_msgs.msg import Pose
from std_msgs.msg import Empty
from std_msgs.msg import Bool

CAGE_X = rospy.get_param("/cage_x", 20)*0.3048
CAGE_Y = rospy.get_param("/cage_y", 10)*0.3048
MARKER_X = np.array([-3, -1, 1, 3])/8.0
MARKER_Y = np.array([2, 0, -2])/6.0
DRONE_FRAME = rospy.get_param("/quadrotor_base", "/base_link")
TARGET_MARKER_ID = rospy.get_param("/target_marker_id", 12)
TARGET_VANISH_TIME = 2
ALPHA_YAW = 0.4
ALPHA_POS = 0.4

class State(object):
    def __init__(self):
        self.active = False
        self.position = np.zeros((3,))
        self.quaternion = np.zeros((4,))
        self.quaternion[-1] = 1

        self.time = 0
        self.velocity = np.zeros((3,))
        self.rotz = 0

        self.cov = np.eye(4)        
        self.Q = np.eye(4)
        self.Q[0:3, :] *= 0.03
        self.Q[3, :] *= 0.01
        self.H = np.eye(4)
        self.R = np.eye(4) * 0.0003

    def _yaw_from_quaternion(self, quaternion):
        _, _, yaw = tf.transformations.euler_from_quaternion(quaternion)
        return yaw
    def _quaternion_from_yaw(self, yaw):
        return tf.transformations.quaternion_from_euler(0, 0, yaw)
    def _add_yaw_to_quaternion(self, quaternion, dyaw):
        yaw = self._yaw_from_quaternion(self.quaternion) + dyaw
        return self._quaternion_from_yaw(yaw)
    def _add_quaternion_to_yaw(self, yaw, dquaternion):
        return yaw + self._yaw_from_quaternion(dquaternion)
    def _state_vector(self, position, quaternion):
        vector = np.zeros((4, 1))
        vector[0:3, 0] = position
        vector[3, 0] = self._yaw_from_quaternion(quaternion)     
        return vector
    def get_state(self):
        return self._state_vector(self.position, self.quaternion)
    def get_pose(self):
        pose = Pose()
        pose.position.x = self.position[0]
        pose.position.y = self.position[1]
        pose.position.z = self.position[2]
        pose.orientation.x = self.quaternion[0]
        pose.orientation.y = self.quaternion[1]
        pose.orientation.z = self.quaternion[2]
        pose.orientation.w = self.quaternion[3]
        return pose
    def inv(self):
        matrix = tf.transformations.quaternion_matrix(self.quaternion)
        matrix[0:3, 3] = self.position
        inverse = np.linalg.inv(matrix)
        position_inv = inverse.copy()[:3, 3]
        inverse[:3, 3] = np.zeros((3,))
        quaternion_inv = tf.transformations.quaternion_from_matrix(inverse)
        return (position_inv, quaternion_inv)
    def kalman_predict(self, velocity, rotz, altd, time):
        if self.active:
            v = velocity
            dyaw = rotz - self.rotz
            dyaw -= 0 if abs(dyaw) < np.pi else np.sign(dyaw)*2*np.pi
            previous_yaw = self._yaw_from_quaternion(self.quaternion)
            yaw = previous_yaw + dyaw/2
            self.quaternion = self._add_yaw_to_quaternion(self.quaternion, dyaw)
            dt = time - self.time
            delta = v*dt
            dx, dy, dz = delta
            rotation = tf.transformations.rotation_matrix(yaw, [0, 0, 1])[:3, :3]
            self.position += rotation.dot(delta)
            self.position[2] = altd
            F = np.eye(4)
            F[0, 3] = -np.sin(yaw)*dx - np.cos(yaw)*dy
            F[1, 3] = np.cos(yaw)*dx - np.sin(yaw)*dy
            self.cov = F.dot(self.cov).dot(F.T) + self.Q
        self.velocity = velocity
        self.rotz = rotz
        self.time = time
        self.active = True
        
    def kalman_update(self, position, quaternion):
        if not self.active:
            self.position[0:2] = position[0:2]
            self.quaternion = quaternion
            self.active = True
            return
        yaw_sensor = self._yaw_from_quaternion(quaternion)
        yaw_prev = self._yaw_from_quaternion(self.quaternion)
        yaw = ALPHA_YAW*yaw_sensor + (1 - ALPHA_YAW)*yaw_prev
        self.quaternion = self._quaternion_from_yaw(yaw)           
        self.position[0:2] *= (1 - ALPHA_POS)
        self.position[0:2] += (ALPHA_POS)*position[0:2]

            
#        sensor = self._state_vector(position, quaternion)
#        prior = self._state_vector(self.position, self.quaternion)
#        error = sensor - self.H.dot(prior)
#        S = self.H.dot(self.cov).dot(self.H.T) + self.R
#        K = self.cov.dot(self.H.T).dot(np.linalg.inv(S))
#        posterior = prior + K.dot(error)
#        self.position[:] = posterior[0:3][0] 
#        self.quaternion = self._quaternion_from_yaw(posterior[3][0])
#        
#       self.cov = (np.eye(4) - K.dot(self.H)).dot(self.cov)  
            
class Target(object):
    def __init__(self):
        self.last_seen = 0
    def in_sight(self):
        return (time.time() - self.last_seen) < TARGET_VANISH_TIME
    def detect(self):        
        self.last_seen = time.time()
    
class DronePoseEstimator(object):
    def __init__(self):
        self.target = Target()
        self.state = State()        
        
        self.static_translations = {}
        marker_frame_count = -1
        for y in MARKER_Y:
            for x in MARKER_X:
                marker_frame_count += 1
                parent = "world"
                child = "ar_marker_%d"%(marker_frame_count)
                self.static_translations[(parent, child)] = (x*CAGE_X, y*CAGE_Y, 0)    
                
        self._reset_time()
        self.marker_subscriber = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self.receive_markers)
        self.sensors_subscriber = rospy.Subscriber('/ardrone/navdata', Navdata, self.receive_navdata)
        self.target_in_sight_publisher = rospy.Publisher('/target_in_sight', Bool, queue_size=100)
        self.pose_estimate_publisher = rospy.Publisher('/pose_estimate', Pose, queue_size=100)
        self.tf_listener = tf.TransformListener()
        self.tf_broadcaster = tf.TransformBroadcaster()
        

        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            self.target_in_sight_publisher.publish(self.target.in_sight())
            self.pose_estimate_publisher.publish(self.state.get_pose())
            world_in_drone_position, world_in_drone_quaternion = self.state.inv()
            self.tf_broadcaster.sendTransform(world_in_drone_position, world_in_drone_quaternion, rospy.Time.now(), "world", DRONE_FRAME)
            rate.sleep()        
        rospy.spin()
       
    def receive_navdata(self, nav):
        velocity = np.array([nav.vx, nav.vy, nav.vz])/1000.0
        rotz = (nav.rotZ + 180)*np.pi/180
        altd = nav.altd/1000.0
        time = nav.tm/float(1e6)
        self.state.kalman_predict(velocity, rotz, altd, time)
    
    def receive_markers(self, markers):
        visible_markers = []
        for marker in markers.markers:
            if (0 <= marker.id < 12):
                visible_markers.append(marker.id)
            if marker.id == TARGET_MARKER_ID:
                self.target.detect()
        estimate = self._estimate_state_from_markers(visible_markers)
        success, position, quaternion = estimate
        if success:
            self.state.kalman_update(position, quaternion)

    def _reset_time(self):
        reset = rospy.Publisher('/reset_time', Empty, queue_size=10)
        reset.publish(Empty())
    
    def _estimate_state_from_markers(self, visible_markers):
        marker_position_estimates = []
        marker_quaternion_estimates = [] 
        
        for marker_id in visible_markers:
            marker_frame = "ar_marker_%d"%(marker_id)
            marker_in_world = np.array(self.static_translations[("world", marker_frame)])
            
            try:
                drone_in_marker = np.array(self.tf_listener.lookupTransform(marker_frame, DRONE_FRAME, rospy.Time()))
            except( tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
            
            position_estimate = drone_in_marker[0]
            quaternion_estimate = drone_in_marker[1]
            
            marker_position_estimates.append(position_estimate + marker_in_world)
            marker_quaternion_estimates.append(quaternion_estimate)
            
        success = False
        position_estimate = None
        quaternion_estimate = None
        if len(marker_position_estimates) > 0:
            success = True
            marker_position_estimates = np.stack((marker_position_estimates))
            position_estimate = np.mean(marker_position_estimates, axis=0)
            marker_quaternion_estimates = np.stack((marker_quaternion_estimates))
            quaternion_estimate = np.mean(marker_quaternion_estimates, axis=0)
            if DRONE_FRAME == "/ardrone_base_link":
                quaternion_estimate = tf.transformations.quaternion_multiply(quaternion_estimate, [0, 0, 1, 0])
            
        return (success, position_estimate, quaternion_estimate)

if __name__ == "__main__":
    rospy.init_node('ardrone_pose_estimator')
    estimator = DronePoseEstimator()

