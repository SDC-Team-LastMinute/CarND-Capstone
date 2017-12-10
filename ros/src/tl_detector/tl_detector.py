#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
import yaml
import math
from tf.transformations import euler_from_quaternion

STATE_COUNT_THRESHOLD = 3

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []
        self.distance_sorted_stop_line_positions = []

        # Subscriptions
        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)

        config_string = rospy.get_param('/traffic_light_config')
        self.config = yaml.load(config_string)

        # Publisher
        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints

    def traffic_cb(self, msg):
        self.lights = msg.lights

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint
        Args:
            msg (Image): image from car-mounted camera
        """
        self.has_image = True
        self.camera_image = msg
        light_wp, state = self.process_traffic_lights()

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            light_wp = light_wp if state == TrafficLight.RED else -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1


    def euclidianDistance(self, x1, y1, x2, y2):
        return math.sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1))

    def getNextWaypoint(self):
        """
        Checks the angle between the car and the waypoint to determine if we
        are beyond the closest waypoint.

        Returns:
            int: 0 if waypoint is still ahead of us, 1 if waypoint is behind
        """

        # car location and heading
        pose = self.pose.pose

        # get car pose
        car_angles = euler_from_quaternion([
            pose.orientation.x, pose.orientation.y,
            pose.orientation.z, pose.orientation.w
        ])
        car_heading = car_angles[2]
        car_x = pose.position.x
        car_y = pose.position.y

        # get waypoint position
        waypoint_x = self.distance_sorted_stop_line_positions[0][0]
        waypoint_y = self.distance_sorted_stop_line_positions[0][1]

        # this calculates heading between carpoint/position and waypoint in radians
        heading = math.atan2((waypoint_y - car_y),(waypoint_x - car_x))

        # angle between car heading and heading
        angle = abs(car_heading - heading)

        # 1 if behind, 0 if in front
        if angle > (math.pi / 2):
            return 1

        return 0

    def get_closest_waypoint(self, x, y):
    #     """Identifies the closest path waypoint to the given position
    #         https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
    #     Args:
    #         pose (Pose): position to match a waypoint to

    #     Returns:
    #         int: index of the closest waypoint in self.waypoints

    #     """

        closestLen = 100000     # large number
        closestWaypoint = 0

        # get car x and y from pose message
        car_x = x
        car_y = y

        # loop over all waypoints for closest
        for i in range(len(self.waypoints.waypoints)):
            # get x and y for waypoint being processed
            waypoint_x = self.waypoints.waypoints[i].pose.pose.position.x
            waypoint_y = self.waypoints.waypoints[i].pose.pose.position.y

            # distance between car and waypoint
            distance = self.euclidianDistance(car_x, car_y, waypoint_x, waypoint_y)

            # update if this is the closest waypoint
            if distance < closestLen:
                closestLen = distance
                closestWaypoint = i

        return closestWaypoint

    def get_light_state(self, light):
        """Determines the current color of the traffic light
        Args:
            light (TrafficLight): light to classify
        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)
        """
        # check if we have an image
        if(not self.has_image):
            self.prev_light_loc = None
            return False

        # image for classification
        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

        #Get classification
        return self.light_classifier.get_classification(cv_image)

    def distance_sort_stoplines(self, distance_map, stop_line_positions):
        """
        Sorts the stop line positions
        Args:
            distance_map:
            stop_line_positions: positions of all the traffic light stop positions
        Returns:
            List of sorted stop line positions
        """
        distance_sorted_stop_line_positions = []

        for index in sorted(distance_map, key=distance_map.get):
            distance_sorted_stop_line_positions.append(stop_line_positions[index])

        return distance_sorted_stop_line_positions

    def waypoint_xy(self, waypoint_id):
        """
        Gets waypoints x and y coordinates from an id
        Args:
            waypoint_id: id of the waypoint whose position we want
        Returns:
            int: x position
            int: y position 
        """
        x = self.waypoints.waypoints[waypoint_id].pose.pose.position.x
        y = self.waypoints.waypoints[waypoint_id].pose.pose.position.y
        return x, y

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color 
        Returns:
            int: index of waypoint closest to the upcoming stop line for a
                 traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)
        """
        light = None
        light_wp = None
        wp_closest_to_light = None

        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']

        # find waypoint of closest traffic light stop point in view
        if(self.pose):
            # get closest waypoint to the car
            car_pose_x = self.pose.pose.position.x
            car_pose_y = self.pose.pose.position.y
            wp_closest_to_car = self.get_closest_waypoint(car_pose_x, car_pose_y)

            # get distance from all traffic lights
            self.distance_stopline_map = {}
            for i, light_position in enumerate(stop_line_positions):
                light_x, light_y = light_position
                distance = self.euclidianDistance(car_pose_x, car_pose_y, light_x, light_y)

                # add only those that are in view
                if distance < self.config['view_distance']:
                  self.distance_stopline_map[i] = distance

            # sort remaining stop line positions by distance
            self.distance_sorted_stop_line_positions = self.distance_sort_stoplines(
                self.distance_stopline_map, stop_line_positions)

            # check if closest is in front/behind us
            if len(self.distance_sorted_stop_line_positions):
              closest_next_stopline_id = self.getNextWaypoint()

            if len(self.distance_sorted_stop_line_positions) > 0 and
               closest_next_stopline_id is not None:
              try:
                wp_closest_to_light = self.get_closest_waypoint(
                    self.distance_sorted_stop_line_positions[closest_next_stopline_id][0],
                    self.distance_sorted_stop_line_positions[closest_next_stopline_id][1])
              except:
                wp_closest_to_light = None

        light_wp = wp_closest_to_light

        # check to see if we have a stop point in view
        if light_wp is not None:
            light = True
        else:
            light = False

        # return the light waypoint and state if there is one, 
        # otherwise return -1 and TrafficLight error
        if light:
            state = self.get_light_state(light)
            return light_wp, state

        #self.waypoints = None ## commenting out for now since it breaks
        return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
