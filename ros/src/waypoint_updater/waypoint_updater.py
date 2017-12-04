#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32
from tf.transformations import euler_from_quaternion

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
        rospy.init_node('waypoint_updater', log_level=rospy.DEBUG)

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below

        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        rospy.Subscriber('/current_velocity', TwistStamped, self.current_velocity_cb)


        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.nextWaypoint = None # current waypoint in front of car
        self.stopfortrafficlight = False # if we have detected red light, then we stop updating final waypoints from pose_cb
        self.tl_waypoints = False # this state is used to denote that slow down TL waypoints have been calculated and published


        rospy.spin()

    def pose_cb(self, msg):

        # get nextWaypoint in front of car
        self.nextWaypoint = self.getNextWaypoint(msg.pose)

        rospy.loginfo('trafficlightbool u pose_cb: {}'.format(self.stopfortrafficlight))
        if self.stopfortrafficlight is False:

            rospy.loginfo('in pose_cb no traffic light')

            self.final_waypoints = self.base_waypoints.waypoints[self.nextWaypoint:(self.nextWaypoint + LOOKAHEAD_WPS)]
            self.last_pose_final_waypoints = self.final_waypoints

            self.publish()
            self.tl_waypoints = False # since we updated final_waypoints with non TL waypoints



    def current_velocity_cb(self, msg):
        self.current_velocity = msg.twist.linear.x
    #rospy.loginfo('current_velocity: {}'.format(self.current_velocity))

    def waypoints_cb(self, waypoints):
        self.base_waypoints = waypoints

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        rospy.loginfo('bool ovo je moja test poruka u traffic cb(): {}'.format(msg))
        #print(msg)

        # msg -1 = there is not red traffic light in front of car
        if msg.data != -1:

            #we have red traffic light and want to prevent pose_cv overwriting final_waypoints
            self.stopfortrafficlight = True
            rospy.loginfo('trafficlightbool ako ima msg: {}'.format(self.stopfortrafficlight))

            red_traffic_waypoint_id = msg.data
            # # decreasing for 5 waypoints to stop sooner
            # # red_traffic_waypoint_id -= 5

            # #velocity_at_next_waypoint = self.get_waypoint_velocity(self.base_waypoints.waypoints[self.nextWaypoint])
            # #rospy.loginfo('velocity at next waypoint: {}'.format(velocity_at_next_waypoint))
            waypoints_from_car_to_tl = int(red_traffic_waypoint_id) - int(self.nextWaypoint)


            # if waypoints_from_car_to_tl > 0:
            #     decr_velocity_step = self.current_velocity / waypoints_from_car_to_tl

            #     rospy.loginfo('vaypoints from car to tl: {}'.format(waypoints_from_car_to_tl))
            #     rospy.loginfo('decr velocity step: {}'.format(decr_velocity_step))


            #     # if self.current_velocity < 1 and waypoints_from_car_to_tl > 3:
            #     #     velocity_acc = 1
            #     # else:
            #     #     velocity_acc = self.current_velocity

            #     velocity_acc = self.current_velocity

            #     decelerated_waypoints = self.base_waypoints.waypoints[self.nextWaypoint:(red_traffic_waypoint_id+ 1)]

            #     for index, waypoint in enumerate(decelerated_waypoints):
            #         velocity_acc -= decr_velocity_step
            #         waypoint.twist.twist.linear.x = velocity_acc

            #      # rospy.loginfo('nr of decelerated_waypoints: {}'.format(len(decelerated_waypoints)))
            #     self.final_waypoints = decelerated_waypoints + self.base_waypoints.waypoints[red_traffic_waypoint_id:(red_traffic_waypoint_id+ (LOOKAHEAD_WPS - waypoints_from_car_to_tl))]
            #     # self.final_waypoints = decelerated_waypoints
            #     # rospy.loginfo('decl_wps: {}, fin_wps: {}'.format(len(decelerated_waypoints), len(self.final_waypoints)))


            #     display_velocity = []
            #     for waypoint in self.final_waypoints:
            #         display_velocity.append(waypoint.twist.twist.linear.x)

            #     rospy.loginfo('final_waypoints_velocity: {}'.format(display_velocity))

            #     self.publish()

            if self.tl_waypoints is False:
                CAR_WAYPOINT = self.nextWaypoint
                TL_WAYPOINT = red_traffic_waypoint_id
                SLOWDOWN_WAYPOINTS = int(waypoints_from_car_to_tl * 0.1)
                ZERO_WAYPOINTS = int(waypoints_from_car_to_tl * 0.8)
                ZERO_WAYPOINTS_START = TL_WAYPOINT - ZERO_WAYPOINTS
                ZERO_WAYPOINTS_END = TL_WAYPOINT
                SLOWDOWN_WAYPOINTS_START = ZERO_WAYPOINTS_START - SLOWDOWN_WAYPOINTS
                SLOWDOWN_WAYPOINTS_END = ZERO_WAYPOINTS_START
                AFTER_TL_WAYPOINTS_START = TL_WAYPOINT
                AFTER_TL_WAYPOINTS_END = TL_WAYPOINT + 100


                waypoints_before_slowdown = self.base_waypoints.waypoints[CAR_WAYPOINT:SLOWDOWN_WAYPOINTS_START]
                slowdown_waypoints_before_tl = self.base_waypoints.waypoints[SLOWDOWN_WAYPOINTS_START:SLOWDOWN_WAYPOINTS_END]
                zero_waypoints_before_tl = self.base_waypoints.waypoints[ZERO_WAYPOINTS_START:ZERO_WAYPOINTS_END]
                waypoints_after_tl = self.base_waypoints.waypoints[AFTER_TL_WAYPOINTS_START:AFTER_TL_WAYPOINTS_END]

                # initial_velocity = 11.11
                # for waypoint in slowdown_waypoints_before_tl:
                #     initial_velocity /= 2
                #     waypoint.twist.twist.linear.x = initial_velocity

                velocity_acc = self.current_velocity
                decr_velocity_step = self.current_velocity / waypoints_from_car_to_tl

                for waypoint in slowdown_waypoints_before_tl:
                    velocity_acc -= decr_velocity_step
                    waypoint.twist.twist.linear.x = velocity_acc

                for waypoint in zero_waypoints_before_tl:
                    waypoint.twist.twist.linear.x = 0.0

                self.final_waypoints = waypoints_before_slowdown + slowdown_waypoints_before_tl + zero_waypoints_before_tl + waypoints_after_tl
                # self.final_waypoints = zero_waypoints_before_tl + waypoints_after_tl

                display_velocity = []
                for waypoint in self.final_waypoints:
                    display_velocity.append(waypoint.twist.twist.linear.x)

                rospy.loginfo('final_waypoints_velocity: {}'.format(display_velocity))

                self.publish()
                # self.tl_waypoints = True

        else:
            self.stopfortrafficlight = False
            rospy.loginfo('trafficlightbool ako nema msg: {}'.format(self.stopfortrafficlight))


        # if we have fully stopped in front of light, our current pose is not changing
            # so let's give car a little nudge
        #if self.current_velocity == 0:
        #    self.final_waypoints = self.last_pose_final_waypoints



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
