#!/usr/bin/env python

import rospy
import actionlib
import ids_coordinates_setter.msg as upmsg
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import RCIn
import math
import threading
import collections

from ids_coordinates_setter.image_editor import set_gps_location
from ids_coordinates_setter.coordinates import Coordinates


class ImageGPSEditorActionServer(object):
    def __init__(self):
        """Init class
        """
        rospy.loginfo("Running %s", rospy.get_caller_id())

        self.refLat = 0  # I'll  Get From ROS Param
        self.refLong = 0  # I'll Get From ROS Param
        self.rEarth = 6378137  # I'll Get From ROS Param
        self.startupTopic = "/RC_signal"  # I'll Get From ROS Param
        self.odometry_topic = "/camera/odom/sample"
        self.altitude_topic = "/mavros/local_position/pose"
        self.is_running = False

        # Camera coordinates buffer
        self.cameraCoordinates = collections.deque(maxlen=300)

        # Drone altitude buffer
        self.droneAltitudes = collections.deque(maxlen=300)

        # Define Semaphores
        self.semaphoreCoordinates = threading.Semaphore()
        self.semaphoreAltitude = threading.Semaphore()

        # Load launch parameters
        self.refLat = rospy.get_param("~latitude")
        self.refLong = rospy.get_param("~longitude")
        self.rEarth = rospy.get_param("~earthRadius")
        self.startupTopic = rospy.get_param("startupTopic")
        self.odometry_topic = rospy.get_param("~odometry_topic")
        self.altitude_topic = rospy.get_param("~altitude_topic")
        self.rc_channel_max = rospy.get_param("trigger_channel_max_val")
        self.rc_channel_min = rospy.get_param("trigger_channel_min_val")

        # init variable for new subscriber
        self.sub_camera = None
        self.sub_altitude = None

        # Create ROS Action Server
        self.action_server = actionlib.SimpleActionServer(
            "change_exif", upmsg.ChangeEXIFAction, execute_cb=self.execute_cb, auto_start=False
        )

        # Start Action Server
        self.action_server.start()

        # Create subscriber for camera pose topic
        rospy.Subscriber(self.startupTopic, RCIn, self.callback)

    def start_action_client(self):
        """ Method to start action server and subscribe camera topic
        """
        # Create subscriber for camera pose topic
        self.sub_camera = rospy.Subscriber(self.odometry_topic, Odometry, self.camera_pose_callback)

        self.sub_altitude = rospy.Subscriber(self.altitude_topic, PoseStamped, self.drone_altitude_callback)

    def stop_action_client(self):
        """ Method to unsubscribe camera topic
        """
        if self.sub_camera is not None:
            # Unsubscribe camera topic
            self.sub_camera.unregister()

    def callback(self, data):
        """ RC Signal Callback
            Based on input, start or stop action client
        """
        try:
            channel_value = data.channels[5]
        except:
            return

        if channel_value >= self.rc_channel_max and self.is_running == False:
            self.start_action_client()
            self.is_running = True
        elif channel_value <= self.rc_channel_min and self.is_running:
            self.stop_action_client()
            self.is_running = False

    def execute_cb(self, goal):
        """ Action Server do work Callback
        Set IMAGE GPS based on camera coordinates
        """
        rospy.loginfo("Start Action")
        # create messages that are used to publish feedback/result
        feedback_action = upmsg.ChangeEXIFActionFeedback()
        result_action = upmsg.ChangeEXIFActionResult()

        # Update feedback
        feedback_action.feedback.operation = "Starting Action. Get image on " + goal.path
        rospy.loginfo(feedback_action.feedback.operation)
        self.action_server.publish_feedback(feedback_action.feedback)

        # Update feedback
        feedback_action.feedback.operation = "Try to open image"
        rospy.loginfo(feedback_action.feedback.operation)
        self.action_server.publish_feedback(feedback_action.feedback)

        # Verify if exist GPS information in buffer
        if len(self.cameraCoordinates) and len(self.droneAltitudes) > 0:
            try:
                target_timestamp = float((float(goal.timestamp_in_milliseconds) / 1000) % 60)
                rospy.loginfo("Received timestamp camera: %s seconds", str(target_timestamp))

                # Update feedback
                feedback_action.feedback.operation = "Try to get Camera Position based on image timestamp"
                rospy.loginfo(feedback_action.feedback.operation)
                self.action_server.publish_feedback(feedback_action.feedback)

                # acquire semaphore to access list
                self.semaphoreCoordinates.acquire()
                self.semaphoreAltitude.acquire()

                # Get closest Coordinates based on target time
                closest_coordinates = self.closest_coordinates(
                    self.cameraCoordinates, self.droneAltitudes, target_timestamp
                )
                # Release semaphore
                self.semaphoreCoordinates.release()
                self.semaphoreAltitude.release()

                # Update feedback
                feedback_action.feedback.operation = "Try to convert camera position to GPS coordinates"
                rospy.loginfo(feedback_action.feedback.operation)
                self.action_server.publish_feedback(feedback_action.feedback)

                # Get new GPS Coordinates based on closesCoordinates XY
                x = closest_coordinates.get_coordinate_x()
                y = closest_coordinates.get_coordinate_y()
                z = closest_coordinates.get_coordinate_z()
                lat1, long1 = self.get_new_gps_coordinates(x, y)

                # Update feedback
                feedback_action.feedback.operation = "Try to modify image GPS information"
                rospy.loginfo(feedback_action.feedback.operation)
                self.action_server.publish_feedback(feedback_action.feedback)

                # Modify Image GPS information
                set_gps_location(goal.path, lat1, long1, z)

                # Send Result
                result_action.result.success = True
                rospy.loginfo("Action run with success: %s", True)
                self.action_server.set_succeeded(result_action.result)
                return

            except Exception as e:
                feedback_action.feedback.operation = "Error processing image " + str(e)
                rospy.logerr(feedback_action.feedback.operation)
                self.action_server.publish_feedback(feedback_action.feedback)

        else:
            # Update feedback
            feedback_action.feedback.operation = "Do not have GPS information"
            rospy.loginfo(feedback_action.feedback.operation)
            self.action_server.publish_feedback(feedback_action.feedback)

        # Send result
        result_action.result.success = False
        rospy.loginfo("Unsuccessful Action Operation")
        self.action_server.set_succeeded(result_action.result)

    def get_new_gps_coordinates(self, dx, dy):
        """ Estimate GPS coordinates based on (latitude, longitude) reference and distance in (x,y)
        Keyword arguments:
        dx -- travelled distance in X (float)
        dy -- travelled distance in Y (float)
        return new gps coordinates
        """
        new_lat = self.refLat + (dy / self.rEarth) * (180.0 / math.pi)
        new_long = self.refLong + (dx / self.rEarth) * (180.0 / math.pi) / math.cos(self.refLat * math.pi / 180.0)
        return new_lat, new_long

    # noinspection PyMethodMayBeStatic
    def camera_pose_callback(self, data):
        """ Camera topic callback
        Get correct timestamp and save in deque an Coordinates object
        Keyword arguments:
        data -- message received (nav_msgs odometry)
        """
        # Acquire semaphore to save new data in cameraTimestamps object
        self.semaphoreCoordinates.acquire()

        # Calculate message timestamp
        timestamp = data.header.stamp.secs + data.header.stamp.nsecs / float(10 ** 9)

        # Only save (x,y)
        self.cameraCoordinates.append(Coordinates(timestamp, data.pose.pose.position.x, data.pose.pose.position.y, 0))

        # Release semaphore
        self.semaphoreCoordinates.release()

    # noinspection PyMethodMayBeStatic
    def drone_altitude_callback(self, data):
        """ Camera topic callback
        Get correct timestamp and save in deque an Coordinates object
        Keyword arguments:
        data -- message received (nav_msgs odometry)
        """
        # Acquire semaphore to save new data in cameraTimestamps object
        self.semaphoreAltitude.acquire()

        # Calculate message timestamp
        timestamp = data.header.stamp.secs + data.header.stamp.nsecs / float(10 ** 9)

        # Only save Altitude Z
        self.droneAltitudes.append(Coordinates(timestamp, 0, 0, abs(data.pose.position.z)))
        # Release semaphore
        self.semaphoreAltitude.release()

    # noinspection PyMethodMayBeStatic
    def closest_coordinates(self, lstxy, lstz, time):
        """
        Find closest object in lists based on value input
        Keyword arguments:
        lstxy -- list with Coordinates object
        lstz -- list with altitude values
        value -- target value to fins closes time in coordinates object
        Return closest object in list based on value input
        """
        try:
            xy = lstxy[min(range(len(lstxy)), key=lambda i: abs(lstxy[i].get_time() - time))]
            z = lstz[min(range(len(lstz)), key=lambda i: abs(lstz[i].get_time() - time))]
            coordinates = Coordinates(time, xy.get_coordinate_x(), xy.get_coordinate_y(), z.get_coordinate_z())
        except:
            return
        return coordinates


if __name__ == "__main__":
    rospy.init_node("image_gps_editor_action_server")
    a = ImageGPSEditorActionServer()
    rospy.on_shutdown(a.stop_action_client)
    rospy.spin()

