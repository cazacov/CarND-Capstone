#!/usr/bin/env python

import rospy
import numpy as np

from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Int32
from scipy.spatial import KDTree

import math

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number

MAX_DECEL = 5   # Max decelearation

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below


        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.pose = None

        self.base_waypoints = None
        self.waypoints_2d = None
        self.waypoint_tree = None

        self.stopline_wp_idx = -1

        self.loop()
    
    def loop(self):
        rate = rospy.Rate(50)   # Target 50 Hz publishing frequency
        while not rospy.is_shutdown():
            if self.pose and self.base_waypoints:   # Internal state is initialized by callbacks
                # Find index of waypoint closest to car position
                #closest_waypoint_idx = self.get_closest_waypoint_idx()
                self.publish_waypoints()
            rate.sleep()

    def get_closest_waypoint_idx(self):
        # Car coordinates
        x = self.pose.pose.position.x
        y = self.pose.pose.position.y

        # Use KDTree to find 1 waypoint closest to car coordinates
        closest_idx = self.waypoint_tree.query([x,y], 1)[1]

        # Ensure the waypoint is in front of the car
        closest_coord = self.waypoints_2d[closest_idx]
        prev_coord = self.waypoints_2d[closest_idx - 1]

        # Check if closest coord is ahead or behind car
        cl_vect = np.array(closest_coord)
        prev_vect = np.array(prev_coord)
        pos_vect = np.array([x,y])

        # Check in angle between (cl_vect - prev_vect) and (pos_vect - cl_vect) vectors is less or greater than 90 degrees

        val = np.dot(cl_vect-prev_vect, pos_vect-cl_vect)

        if val > 0:     # cosine of angle > 0  =>  angle < 90 degrees  =>  point cl_vect is behind the car
            # take next point considering the length of array
            closest_idx = (closest_idx + 1) % len(self.waypoints_2d)
        
        return closest_idx

    def publish_waypoints(self):
        final_lane = self.generate_lane()
        # Publish to final_waypoints topic
        self.final_waypoints_pub.publish(final_lane)

    def generate_lane(self):
        lane = Lane()   # create Lane message
        lane.header = self.base_waypoints.header    # Use the same header

        closest_idx = self.get_closest_waypoint_idx()
        farthest_idx = closest_idx + LOOKAHEAD_WPS
        
        base_waypoints = self.base_waypoints.waypoints[closest_idx:farthest_idx]
        if self.stopline_wp_idx == -1 or self.stopline_wp_idx >= farthest_idx:
            lane.waypoints = base_waypoints
        else:
            lane.waypoints = self.decelerate_waypoints(base_waypoints, closest_idx)

        return lane

    def decelerate_waypoints(self, waypoints, closest_idx):
        temp = []
        stop_idx = max(self.stopline_wp_idx - closest_idx - 4, 0)   # Three waypoints back from line
        #rospy.logwarn("Decelerating waypoints from %s to %s, s-idx: %s", closest_idx, self.stopline_wp_idx, stop_idx)

        for i, wp in enumerate(waypoints):
            p = Waypoint()
            p.pose = wp.pose

            if i < stop_idx:
                dist = self.distance(waypoints, i, stop_idx)
                #### Default deceleration profile with constant negative acceleration = MAX_DECEL
                vel = min(math.sqrt(2 * MAX_DECEL * dist), wp.twist.twist.linear.x)
                if vel < 1.0:
                    vel = 0.0
                p.twist.twist.linear.x = vel
            else:
                p.twist.twist.linear.x = 0
            temp.append(p)

###################################        
#### Experimental more smooth deceleration profile with  a proportional to sin(distance*k)
###################################

        t = 0
        s = 0
        a = MAX_DECEL * 0.9


#             T = 1.1 * v0 * (math.pi - 2 * p) / 2
#             p2 = (math.pi - 2*p) / T
#             s_max = v0 / 2.0 * (T - 1.1 / p2 * math.sin(p2 * (T + p)))     
#             if dist < s_max:
#                 while s < dist:
#                     t = t + 0.005
#                     s = v0 / 2 * (t - 1.1 / T * math.sin(p2*(t + p)))
#                 v = v0 / 2 * (1 - 1.1 * math.cos(p2 * (t + p)))


        for i in reversed(xrange(stop_idx)):
            dist = self.distance(waypoints, i, stop_idx)
            v0 = waypoints[i].twist.twist.linear.x

            T = math.pi * v0 / 2.0 / a
            s_max =  T * v0 / 2

            if dist < s_max:
                
                while s < dist:
                    t = t + 0.005
                    s = v0 / 2 * (t - T/math.pi*math.sin(t * math.pi/T))
                v = v0 / 2 * (1 - math.cos(t*math.pi/T))
                if v < 1:
                    v = 0.
                if v > v0:
                    v = v0

                temp[i].twist.twist.linear.x = v
                #rospy.logwarn("i: %s dist: %s s:%s, v: %s", i, dist, s, v)
            else:
                break
##############################    
        
        return temp



    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):

        # message styx_msgs/Lane has following structure:
        # std_msgs/Header header
        #     ...
        # styx_msgs/Waypoint[] waypoints
        #     geometry_msgs/PoseStamped pose
        #         std_msgs/Header header
        #             ...
        #         geometry_msgs/Pose pose
        #             geometry_msgs/Point position
        #                 float64 x
        #                 float64 y
        #                 float64 z
        # ...

        # Base waypoints are sent only once and never changed later, store them for later use
        self.base_waypoints = waypoints

        # Store a KDTree structure. We will later use it to efficiently find a waypoint closest to car position
        if not self.waypoints_2d:   
            # Extract 2d coordinnates from waypoint positions
            self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints]
            self.waypoint_tree = KDTree(self.waypoints_2d)


    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        self.stopline_wp_idx = msg.data
#        rospy.logwarn("Stoplight waypoint: %s", self.stopline_wp_idx )
        

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
