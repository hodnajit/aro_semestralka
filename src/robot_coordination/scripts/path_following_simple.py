#!/usr/bin/env python
# -*- encoding: utf-8 -*-
# An example script to send a trajectory to a robot, start and stop the motion.
import time
import geometry_msgs.msg
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Int32, Bool
from robot_coordination.msg import Waypoint
from robot_coordination.srv import AddPath
from robot_coordination.srv import StartMovement
from robot_coordination.srv import StopMovement
import rospy
from nav_msgs.msg import Path
from sensor_msgs.msg import LaserScan
from laser_geometry.laser_geometry import LaserProjection
import tf2_ros
import tf2_geometry_msgs


class PathFollowing:
    def __init__(self):
        self.state_sub = rospy.Subscriber('waypoints_ahead', Int32, self.callback_state)
        self.state_sub = rospy.Subscriber('path_waypoints', Path, self.callback_path)
        self.stop_sub = rospy.Subscriber('stop', Bool, self.callback_stop)
        self.odom_frame = rospy.get_param('~odom_frame', 'odom')
        self.robot_frame = rospy.get_param('~robot_frame', 'base_link')
        self.waypoints_ahead = 0
        self.waypoints_ahead_updated = False
        self.tfBuffer = tf2_ros.Buffer()
        # Use the tfBuffer to obtain transformation as needed
        tfListener = tf2_ros.TransformListener(self.tfBuffer)

    def callback_state(self, msg):
        self.waypoints_ahead = msg.data
        self.waypoints_ahead_updated = True

    def callback_path(self, msg):
        self.stop_movement()
        waypoint_list = self.create_trajectory(msg)
        if not self.add_path(waypoint_list):
            rospy.logerr('Could not add path, exiting')
        rospy.loginfo('Path added')
        if not self.start_movement(backwards=False):
            rospy.logerr('Could not start motion, exiting')
        rospy.loginfo('Movement started')

    def callback_stop(self, msg):
        # pro opticky bumper, zatim nefacha uplne dobre
        '''if msg.data == True:
            self.stop_movement
            print("Nikam nepojedu. Tecka.")
        else:
            print("Uz frcim.")'''


    def wait_for_service(self, srv_name):
        """Wait for a service called srv_name."""
        while not rospy.is_shutdown():
            try:
                rospy.wait_for_service(srv_name, 5)
                return True
            except rospy.ROSException:
                rospy.logwarn('Could not connect to service {}, trying again'.format(srv_name))
            except (rospy.ROSInterruptException, KeyboardInterrupt):
                return False

    def start_movement(self, backwards=False):
        """Start the robot motion by calling the service 'start_movement'"""
        srv_name = '/start_movement'
        if not self.wait_for_service(srv_name):
            return False
        try:
            start_movement_srv = rospy.ServiceProxy(srv_name, StartMovement)
            reply = start_movement_srv(backwards)
            return reply.ack
        except rospy.ServiceException as e:
            rospy.logerr('Service call failed: {}'.format(e))
        self.waypoints_ahead_updated = False
        return False

    def stop_movement(self):
        """Stop the robot motion by calling the service 'stop_movement'"""
        srv_name = '/stop_movement'
        if not self.wait_for_service(srv_name):
            return False
        try:
            stop_movement_srv = rospy.ServiceProxy(srv_name, StopMovement)
            reply = stop_movement_srv()
            return reply.ack
        except rospy.ServiceException as e:
            rospy.logerr('Service call failed: {}'.format(e))
        return False

    def add_path(self, waypoint_list):
        """Add a path with data from trajectory"""
        srv_name = '/add_path'
        if not self.wait_for_service(srv_name):
            return False
        try:
            add_path_srv = rospy.ServiceProxy(srv_name, AddPath)
            reply = add_path_srv(waypoint_list)
            return reply.result
        except rospy.ServiceException as e:
            rospy.logerr('Service call failed: {}'.format(e))
        return False

    def path_finished(self):
        return (self.waypoints_ahead <= 0) and self.waypoints_ahead_updated

    def create_trajectory(self,data):
        """ Create a trajectory in the odometry frame. """
        # x,y,time
        """
        gtr = [
            [-0.1, -0.1, 0],
            [-0.2, -0.2, 0],
            [-0.3, -0.3, 0],
            [-0.4, -0.4, 0],
            [-0.5, -0.5, 0],
        ]
        """
        waypoint_list = []
        """
        for r in gtr:
            waypoint = Waypoint()
            waypoint.pose = Pose()
            waypoint.pose.position.x = r[0]
            waypoint.pose.position.y = r[1]
            waypoint.timepoint = rospy.Duration.from_sec(r[2])
            waypoint_list.append(waypoint)
        """
        tmpData = data.poses#[::2]
        tmpData = tmpData[3:len(tmpData)-5]
        transforming = True
        try:
            trans = self.tfBuffer.lookup_transform("odom", "map", rospy.Time(), rospy.Duration(0.5))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn("Cannot get the odom position!")
            transforming = False

        for waypoint_tmp in tmpData:
            waypoint = Waypoint()
            waypoint.pose = Pose()
            if transforming:
                pose_transformed = tf2_geometry_msgs.do_transform_pose(waypoint_tmp, trans)
                print(trans)
            else: 
                pose_transformed = waypoint_tmp
            waypoint.pose = pose_transformed.pose
            waypoint.timepoint = rospy.Duration.from_sec(5)
            waypoint_list.append(waypoint)

        return waypoint_list

def main():
    rospy.init_node('fetch_barbie')

    pf = PathFollowing()
    rate = rospy.Rate(10)


    while not rospy.is_shutdown():

        rate.sleep()



if __name__ == '__main__':
    main()
