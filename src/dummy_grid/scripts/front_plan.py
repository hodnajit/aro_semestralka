#!/usr/bin/env python
from __future__ import absolute_import, division, print_function
import rospy
from geometry_msgs.msg import Pose2D, Pose, Point, Vector3, PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Header, ColorRGBA, Int32
from visualization_msgs.msg import Marker, MarkerArray
from exploration.srv import AnyFrontiersLeft, AnyFrontiersLeftRequest, AnyFrontiersLeftResponse, GenerateFrontier, GenerateFrontierResponse
from exploration.srv import PlanPath,  PlanPathRequest, PlanPathResponse
import numpy as np

sentPath = 0
newPath = False

def path_cb(msg):
    global newPath
    newPath = False
    rcvPath = msg.data # pocet kolik waypoints zbyva do konce
    traveledPath = sentPath - rcvPath
    change = sentPath/2
    if traveledPath > change:
        newPath = True
        #print("Preplanovavam "+str(traveledPath)+">"+str(change))


if __name__ == "__main__":
    rospy.init_node("frontier_planner")
    print("start")

    pathSubscriber = rospy.Subscriber('waypoints_ahead', Int32, path_cb)

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
        if cmd.lower() == "random":
            rospy.wait_for_service("get_random_frontier")
            caller = rospy.ServiceProxy("get_random_frontier", GenerateFrontier)

        elif cmd.lower() == "near":
            rospy.wait_for_service("get_closest_frontier")
            caller = rospy.ServiceProxy("get_closest_frontier", GenerateFrontier)

        response = caller.call()
        rospy.sleep(0.5)
        header = Header(stamp=rospy.Time.now(), frame_id="map")
        msg = Marker(header=header, pose=Pose(position=Point(response.goal_pose.x, response.goal_pose.y, 0)), id=np.random.randint(0, 1e9), type=Marker.CUBE, scale=Vector3(0.1, 0.1, 0.1), color=ColorRGBA(0.5, 1, 0, 1), lifetime=rospy.Duration(0))
        frontierMarker.publish(msg)
        rospy.loginfo(response)

        request = PlanPathRequest(Pose2D(response.goal_pose.x, response.goal_pose.y, 0.0))
        response = pathPlanner.call(request)
        rospy.loginfo(response)

        global sentPath
        sentPath = len(response.path)

        rospy.sleep(0.5)
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
        while not newPath and not rospy.is_shutdown():
            pass

        print("pracuju")
