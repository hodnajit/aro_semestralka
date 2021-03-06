#!/usr/bin/env python
from __future__ import absolute_import, division, print_function
import rospy
from geometry_msgs.msg import Pose2D, Pose, Point, Vector3, PoseStamped, Twist
from nav_msgs.msg import Path
from std_msgs.msg import Header, ColorRGBA, Int32
from visualization_msgs.msg import Marker, MarkerArray
from exploration.srv import AnyFrontiersLeft, AnyFrontiersLeftRequest, AnyFrontiersLeftResponse, GenerateFrontier, GenerateFrontierResponse
from exploration.srv import PlanPath,  PlanPathRequest, PlanPathResponse
import numpy as np
import math
import time

from robot_coordination.srv import StopMovement, StartMovement

sentPath = 0
newPath = False
barbiex = -1
barbiey = -1
barbieDetected = False
rcvPath = 0

def path_cb(msg):
    global newPath
    newPath = False
    global rcvPath
    rcvPath = msg.data # pocet kolik waypoints zbyva do konce
    traveledPath = sentPath - rcvPath
    change = sentPath/3*2
    print("sentPath "+str(sentPath)+" - recieve path " +str(rcvPath)+ " = travelled "+str(traveledPath)+" < "+str(change) )
    if traveledPath > change:
        newPath = True
        print("Preplanovavam "+str(traveledPath)+">"+str(change))


def barbie_cb(msg):
    global barbiex
    barbiex = msg.x
    global barbiey
    barbiey = msg.y
    global barbieDetected
    barbieDetected = True
    print("barbie at "+str(barbiex)+","+str(barbiey))
    global newPath
    newPath = True

def wait_for_service( srv_name):
    """Wait for a service called srv_name."""
    while not rospy.is_shutdown():
        try:
            rospy.wait_for_service(srv_name, 1)
            return True
        except rospy.ROSException:
            rospy.logwarn('Could not connect to service {}, trying again'.format(srv_name))
        except (rospy.ROSInterruptException, KeyboardInterrupt):
            return False

def start_movement( backwards=False):
    """Start the robot motion by calling the service 'start_movement'"""
    srv_name = '/start_movement'
    if not wait_for_service(srv_name):
        return False
    try:
        start_movement_srv = rospy.ServiceProxy(srv_name, StartMovement)
        reply = start_movement_srv(backwards)
        return reply.ack
    except rospy.ServiceException as e:
        rospy.logerr('Service call failed: {}'.format(e))
        waypoints_ahead_updated = False
    return False

def stop_movement():
    """Stop the robot motion by calling the service 'stop_movement'"""
    srv_name = '/stop_movement'
    if not wait_for_service(srv_name):
        return False
    try:
        stop_movement_srv = rospy.ServiceProxy(srv_name, StopMovement)
        reply = stop_movement_srv()
        return reply.ack
    except rospy.ServiceException as e:
        rospy.logerr('Service call failed: {}'.format(e))
    return False

def rotate360():
    print("tocime")
    stop_movement()
    vel_msg = Twist()
    speed = 4
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = 2
    for x in xrange(1,10):
        cmdPub.publish(vel_msg)
        time.sleep(3.5)
    vel_msg.angular.z = 0
    print("mocime")
    cmdPub.publish(vel_msg)



if __name__ == "__main__":
    rospy.init_node("frontier_planner")
    print("start")

    pathSubscriber = rospy.Subscriber('waypoints_ahead', Int32, path_cb)
    barbieSubscriber = rospy.Subscriber('barbie_position_final', Point, barbie_cb)
    cmdPub = rospy.Publisher('/cmd_vel_mux/safety_controller',Twist, queue_size=10)

    cmd = rospy.get_param("~cmd", "random")
    frontierMarker = rospy.Publisher("frontier", Marker, queue_size=10)
    pathMarkers = rospy.Publisher("path", MarkerArray, queue_size=10)
    path_pub = rospy.Publisher("path_waypoints", Path, queue_size=10)
    rospy.wait_for_service("plan_path")
    pathPlanner = rospy.ServiceProxy("plan_path", PlanPath)

    print("frontier planning started")
    any = True
    while any and not rospy.is_shutdown():


        #rospy.wait_for_service("any_frontiers_left")
        #caller = rospy.ServiceProxy("any_frontiers_left", AnyFrontiersLeft)
        #rospy.loginfo(caller.call())
        #print("mame dalsi frontiery?")
        #response = caller.call()
        #any = response.any_frontiers_left
        #print(any)
        if not barbieDetected:
            #rotate360()
            print("searching frontiers")
            if cmd.lower() == "random":
                rospy.wait_for_service("get_random_frontier")
                caller = rospy.ServiceProxy("get_random_frontier", GenerateFrontier)

            elif cmd.lower() == "near":
                rospy.wait_for_service("get_closest_frontier")
                caller = rospy.ServiceProxy("get_closest_frontier", GenerateFrontier)

            response = caller.call()
            #print("TADYYYYYYYYYYY vypisu path")
            rospy.loginfo(response)
            #print("DOPSAAAAAAAAAAL path")
            rospy.sleep(0.5)
            header = Header(stamp=rospy.Time.now(), frame_id="map")
            msg = Marker(header=header, pose=Pose(position=Point(response.goal_pose.x, response.goal_pose.y, 0)), id=np.random.randint(0, 1e9), type=Marker.CUBE, scale=Vector3(0.1, 0.1, 0.1), color=ColorRGBA(0.5, 1, 0, 1), lifetime=rospy.Duration(0))
            frontierMarker.publish(msg)
            print("RESPONSE"+str(response.goal_pose))
            if response.goal_pose is None:
                print("nenasel frontiera, toci se")
                rotate360()
                print("neplanuju nikam, hledam znova")
                continue


            posx = response.goal_pose.x
            posy = response.goal_pose.y
        if barbieDetected:
            posx = barbiex
            posy = barbiey
            print("planning to barbie")
            global barbieDetected
            barbieDetected = False

        request = PlanPathRequest(Pose2D(posx, posy, 0.0))
        response = pathPlanner.call(request)
        rospy.loginfo(response)

        global sentPath
        sentPath = len(response.path)
        global rcvPath
        rcvPath = sentPath

        header = Header(stamp=rospy.Time.now(), frame_id="map")
        msg = MarkerArray([Marker(header=header, pose=Pose(position=Point(p.x, p.y, 0)), id=np.random.randint(0, 1000), type=1, scale=Vector3(0.05, 0.05, 0.05), color=ColorRGBA(0.5, 0.5, 1, 0.8)) for p in response.path])
        pathMarkers.publish(msg)

        path_points = Path()
        path_points.header = header
        poses = []
        for p in response.path:
            poseStamped = PoseStamped()
            pose = Pose()
            pose.position.x = p.x
            pose.position.y=p.y
            poseStamped.pose = pose
            poseStamped.header = header
            poses.append(poseStamped)

        path_points.poses = poses

        path_pub.publish(path_points)
        print("cekam az dojede pohyb")
        #rospy.sleep(3)
        #while not newPath and not rospy.is_shutdown():
            #rospy.sleep(0.1)



        print("pracuju")
