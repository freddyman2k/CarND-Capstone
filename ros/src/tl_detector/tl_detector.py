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
from scipy.spatial import KDTree

STATE_COUNT_THRESHOLD = 3

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.waypoint_tree = None
        self.camera_image = None
        self.lights = []

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

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

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
        waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] 
                         for waypoint in waypoints.waypoints]
        self.waypoint_tree = KDTree(waypoints_2d)

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
            if state != TrafficLight.RED:
                light_wp = -1  
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1

    def get_closest_waypoint(self, x, y):
        """Identifies the closest path waypoint to the given position.
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        if self.waypoint_tree:
            return self.waypoint_tree.query([x, y], 1)[1]
        else:
            return -1

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        return light.state
        # if(not self.has_image):
        #     self.prev_light_loc = None
        #     return False

        # cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

        # #Get classification
        # return self.light_classifier.get_classification(cv_image)

    def get_closest_traffic_light(self, car_closest_waypoint_idx):
        """ Finds the closest traffic light (and corresponding stopline) ahead of the vehicle
        
        Args:
            car_closest_waypoint_idx (int): index of the waypoint closest to the car

        Returns:
            closest_stopline_waypoint_idx (int): index of the stop line corresponding to closest_light
            closest_light (TrafficLight): closest upcoming traffic light, if none exist, return None 
        
        """
        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']
        
        # Loop through all traffic light positions to find closest
        diff = len(self.waypoints.waypoints)
        closest_light = None
        closest_stopline_waypoint_idx = None
        for i, light in enumerate(self.lights):
            # Get waypoint index corresponding to current stop line
            line = stop_line_positions[i]
            current_wp_idx = self.get_closest_waypoint(line[0], line[1])
            # Check if current stopline is closest to the car
            d = current_wp_idx - car_closest_waypoint_idx
            if d >= 0 and d < diff:
                diff = d
                closest_light = light
                closest_stopline_waypoint_idx = current_wp_idx
        
        return closest_stopline_waypoint_idx, closest_light
            

    def process_traffic_lights(self):
        """Gets closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closest to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        closest_light = None
        closest_stopline_waypoint_idx = -1 # in case no traffic light is detected
        state = TrafficLight.UNKNOWN # in case no traffic light is detected

        if(self.pose):
            car_closest_waypoint_idx = self.get_closest_waypoint(self.pose.pose.position.x, 
                                                                 self.pose.pose.position.y)

            if car_closest_waypoint_idx != -1:
                closest_stopline_waypoint_idx, closest_light = self.get_closest_traffic_light(car_closest_waypoint_idx)


        if closest_light:
            # If a traffic light was found nearby, check its (color) state
            state = self.get_light_state(closest_light)

        #self.waypoints = None
        return closest_stopline_waypoint_idx, state

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
