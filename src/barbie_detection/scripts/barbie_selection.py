#!/usr/bin/env python
import rospy
from nav_msgs.msg import Path, OccupancyGrid
from std_msgs.msg import Header, ColorRGBA
from geometry_msgs.msg import PointStamped, Pose, Vector3
from visualization_msgs.msg import Marker, MarkerArray
from exploration.srv import AnyFrontiersLeft, AnyFrontiersLeftRequest, AnyFrontiersLeftResponse, GenerateFrontier, GenerateFrontierResponse
from exploration.srv import PlanPath,  PlanPathRequest, PlanPathResponse
import numpy as np
import tf.transformations as tft
import tf2_ros


detections = []
def barbie_cb(msg):

    detections.append(msg.point)

    header = Header(stamp=rospy.Time.now(), frame_id="camera_rgb_optical_frame")
    msg = Marker(header=header, pose=Pose(position=msg.point), id=np.random.randint(0, 1e9), type=Marker.SPHERE, scale=Vector3(0.05, 0.05, 0.05), color=ColorRGBA(0, 0, 1, 1), lifetime=rospy.Duration(0))
    markerAllPublisher.publish(msg)


if __name__ == "__main__":
    rospy.init_node("barbie_selection")
    print("start")

    barbSubscriber = rospy.Subscriber('barbie_point', PointStamped, barbie_cb)
    barbPublisher = rospy.Publisher('barbie_position',PointStamped, queue_size=2)
    markerAllPublisher = rospy.Publisher('barbie_marker_all',Marker, queue_size=2)
    markerPublisher = rospy.Publisher('barbie_marker',Marker, queue_size=2)

    rospy.spin()


