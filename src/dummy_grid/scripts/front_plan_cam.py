#!/usr/bin/env python
from __future__ import absolute_import, division, print_function
import rospy
from geometry_msgs.msg import Pose2D, Pose, Point, Vector3, PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Header, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from exploration.srv import AnyFrontiersLeft, AnyFrontiersLeftRequest, AnyFrontiersLeftResponse, GenerateFrontier, GenerateFrontierResponse
from exploration.srv import PlanPath,  PlanPathRequest, PlanPathResponse
import numpy as np

if __name__ == "__main__":
    rospy.init_node("frontier_planner_cam")
    print("start")

    cmd = rospy.get_param("~cmd", "random")
    frontierMarker = rospy.Publisher("frontier", Marker, queue_size=10)
    pathMarkers = rospy.Publisher("path", MarkerArray, queue_size=10)
    path_pub = rospy.Publisher("path_waypoints", Path, queue_size=10)
    rospy.wait_for_service("plan_path")
    pathPlanner = rospy.ServiceProxy("plan_path", PlanPath)

    print("frontier planning started")
    any = True
    while any and not rospy.is_shutdown():
        rospy.wait_for_service("any_frontiers_left_cam")
        caller = rospy.ServiceProxy("any_frontiers_left_cam", AnyFrontiersLeft)
        #rospy.loginfo(caller.call())
        response = caller.call()
        any = response.any_frontiers_left
        print(any)
        if cmd.lower() == "random":
            rospy.wait_for_service("get_random_frontier_cam")
            caller = rospy.ServiceProxy("get_random_frontier_cam", GenerateFrontier)

        elif cmd.lower() == "near":
            rospy.wait_for_service("get_closest_frontier_cam")
            caller = rospy.ServiceProxy("get_closest_frontier_cam", GenerateFrontier)
            
        response = caller.call()
        rospy.sleep(0.5)
        header = Header(stamp=rospy.Time.now(), frame_id="map")
        msg = Marker(header=header, pose=Pose(position=Point(response.goal_pose.x, response.goal_pose.y, 0)), id=np.random.randint(0, 1e9), type=Marker.CUBE, scale=Vector3(0.1, 0.1, 0.1), color=ColorRGBA(0.5, 1, 0, 1), lifetime=rospy.Duration(0))
        frontierMarker.publish(msg)
        rospy.loginfo(response)

        request = PlanPathRequest(Pose2D(response.goal_pose.x, response.goal_pose.y, 0.0))
        response = pathPlanner.call(request)
        rospy.loginfo(response)

        rospy.sleep(0.5)
        msg = MarkerArray([Marker(header=header, pose=Pose(position=Point(p.x, p.y, 0)), id=np.random.randint(0, 1000), type=1, scale=Vector3(0.1, 0.1, 0.1), color=ColorRGBA(0.5, 0.5, 1, 1)) for p in response.path])
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
        rospy.sleep(60)
