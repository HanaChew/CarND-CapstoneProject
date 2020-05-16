#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
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


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below


        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        # add variables to store received data
        self.pose = None
        self.base_waypoints = None
        self.waypoints_2d = None
        self.waypoints_tree = None

        # old version: rospy.spin()
        # rospy.spin()

        # new version: loop --> gives control about publishing frequency
        self.loop()

    # main loop
    # results will be received by waypoint follower, which runs at 30 Hz
    def loop(self):
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            # if vars are initialized
            if self.pose and self.base_waypoints:
                # get closest waypoint
                closest_waypoint_idx = self.get_closes_waypoint_id()
                self.publish_waypoints(closest_waypoint_idx)
            rate.sleep()

    def get_closest_waypoint_id(self):
        x = self.pose.pose.position.x
        y = self.pose.pose.position.y
        # x,y : position of ourself, find waypoint next to us
        closest_idx = self.waypoint_tree.query([x,y], 1)[1]

        # check if closest waypoint is ahead or behind of us / vehicle
        closest_coord = self.waypoints_2d[closest_idx]
        prev_coord = self.waypoints_2d[closest_idx-1]

        # equation for hyper plane through closest_coord
        cl_vect = np.array(closest_coord)
        prev_vect = np.array(prev_coord)
        pos_vect = np.array([x,y])

        # if vector(cl_vect - prev_vect) * vector(pos_vect - cl_vect) are in same directions, 
        # dot-product will be > 0
        val = np.dot(cl_vect - prev_vect, pos_vect - cl_vect)
        if val > 0:
            closest_idx = (closest_idx + 1) % len(self.waypoints_2d)
        
        return closest_idx

    # publish waypoints between closest_idx and LOOKAHEAD_WPS 
    def publish_waypoints(self, closest_idx):
        lane = Lane()
        lane.header = self.base_waypoints.header
        # no manuel slicing needed due to python slicing
        lane.waypoints = self.base_waypoints.waypoints[closest_idx:closest_idx + LOOKAHEAD_WPS]
        self.final_waypoints_pub.publish(lane)

    # Store received msg to internal pose; called at 50 Hz
    def pose_cb(self, msg):
        self.pose = msg

    # Store received msg to internal waypoints
    def waypoints_cb(self, waypoints):
        self.base_waypoints = waypoints     # latched Subscriber --> callback called once for base waypoints
        # initialize waypoints_2d before the subscriber is initialized; avoid race-conditions
        if not self.waypoints_2d:
            # find closest waypoint to car with KDTree ("from scipy.spatial import KDTree" needed)
            # 1. convert coordinates to 2d-coord [x,y] and
            # 2. add to KDTree
            self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints]
            # 3. initialize tree with these 2d-waypoints [x,y]
            self.waypoint_tree = KDTree(self.waypoints_2d)


    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        pass

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
