#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32
from tf.transformations import euler_from_quaternion

import math
import csv

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

ZERO_WAYPOINTS_BEFORE_TL = 25
ZERO_WAYPOINTS_AFTER_TL = 2



class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater', log_level=rospy.DEBUG)

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        self.base_subscriber = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below

        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        rospy.Subscriber('/current_velocity', TwistStamped, self.current_velocity_cb)


        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.nextWaypoint = None # current waypoint in front of car
        self.stopfortrafficlight = False # if we have detected red light, then we stop updating final waypoints from pose_cb
        self.tl_waypoints = False # this state is used to denote that slow down TL waypoints have been calculated and published
        self.previous_red_tl_waypoint = None

        rospy.spin()

    def pose_cb(self, msg):

        # get nextWaypoint in front of car
        self.nextWaypoint = self.getNextWaypoint(msg.pose)

        self.final_waypoints = self.base_waypoints.waypoints[self.nextWaypoint:(self.nextWaypoint + LOOKAHEAD_WPS)]

        # display_velocity = []
        # for waypoint in self.final_waypoints:
        #     display_velocity.append(waypoint.twist.twist.linear.x)

        # rospy.loginfo('final_waypoints_velocity_POSE_CB: {}'.format(display_velocity))

        self.publish()



    def current_velocity_cb(self, msg):
        self.current_velocity = msg.twist.linear.x
    #rospy.loginfo('current_velocity: {}'.format(self.current_velocity))

    def waypoints_cb(self, waypoints):
        self.base_waypoints = waypoints
        # we got points no need to be subscribed anymore
        self.base_subscriber.unregister()


    def set_velocity_around_tl(self, tl_waypoint, velocity):

        # The stopping location is offset by 3 wp's because otherwise the car center stops on the stop line
        # instead of the front of the car stopping on the stop line
        ZERO_WAYPOINTS_START = tl_waypoint - ZERO_WAYPOINTS_BEFORE_TL - 3
        ZERO_WAYPOINTS_END   = tl_waypoint + ZERO_WAYPOINTS_AFTER_TL - 3

        zero_waypoints_slice = self.base_waypoints.waypoints[ZERO_WAYPOINTS_START:ZERO_WAYPOINTS_END]

        # Find the average velocity previously set leading up to the traffic light
        sum = 0.0
        for waypoint in self.base_waypoints.waypoints[tl_waypoint-ZERO_WAYPOINTS_BEFORE_TL:tl_waypoint]:
            sum += waypoint.twist.twist.linear.x
        avg_velocity = sum/ZERO_WAYPOINTS_BEFORE_TL

        # Calculate a linear increment/decrement for the WP's leading up to the TL
        delta = (velocity-avg_velocity)/ZERO_WAYPOINTS_BEFORE_TL
        for waypoint in zero_waypoints_slice:
            wp_velocity = avg_velocity+delta
            waypoint.twist.twist.linear.x = wp_velocity
            avg_velocity = wp_velocity
            # waypoint.twist.twist.linear.x = 11.11111111111111

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement

        # msg = -1 if there is no red traffic light in front of car
        if msg.data != -1:

            red_traffic_waypoint_id = msg.data

            if red_traffic_waypoint_id != self.previous_red_tl_waypoint:
                rospy.loginfo('RED Light waypoint ahead. Setting zero velocity waypoints')

                self.previous_red_tl_waypoint = red_traffic_waypoint_id
                self.set_velocity_around_tl(red_traffic_waypoint_id, 0.0)

        else:
            # No RED traffic light case
            # reset velocity to 11.11111111111111
            if self.previous_red_tl_waypoint is not None:
                rospy.loginfo('RED Light waypoint gone. Resetting velocity waypoints to normal')
                # Hardcoded velocity = bad, should get the setpoint velocity from the base waypoints
                self.set_velocity_around_tl(self.previous_red_tl_waypoint, 11.11111111111111)
                self.previous_red_tl_waypoint = None


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

    def euclidianDistance(self, x1, y1, x2, y2):
        return math.sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1))


    def publish(self):
        lane = Lane()
        lane.header.frame_id = '/world'
        lane.header.stamp = rospy.Time(0)
        lane.waypoints = self.final_waypoints
        self.final_waypoints_pub.publish(lane)


    def getClosestWaypoint(self, pose):

        closestLen = 100000
        closestWaypoint = 0

        # get car x and y from pose message
        car_x = pose.position.x
        car_y = pose.position.y

        for i in range(len(self.base_waypoints.waypoints)):

            # get x and y for waypoint being processed
            waypoint_x = self.base_waypoints.waypoints[i].pose.pose.position.x
            waypoint_y = self.base_waypoints.waypoints[i].pose.pose.position.y

            distance = self.euclidianDistance(car_x, car_y, waypoint_x, waypoint_y)

            if distance < closestLen:
                closestLen = distance
                closestWaypoint = i

        return closestWaypoint


    def getNextWaypoint(self, pose):

        # car location and heading
        car_angles = euler_from_quaternion([
            pose.orientation.x, pose.orientation.y,
            pose.orientation.z, pose.orientation.w
        ])
        car_heading = car_angles[2]
        car_x = pose.position.x
        car_y = pose.position.y

        closestWaypoint = self.getClosestWaypoint(pose)

        waypoint_x = self.base_waypoints.waypoints[closestWaypoint].pose.pose.position.x
        waypoint_y = self.base_waypoints.waypoints[closestWaypoint].pose.pose.position.y

        # this calculates heading between carpoint/position and waypoint in radians
        heading = math.atan2((waypoint_y - car_y),(waypoint_x - car_x))

        # angle between car heading and heading
        angle = abs(car_heading - heading)

        if angle > (math.pi / 2):
            closestWaypoint += 1

        return closestWaypoint



if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
