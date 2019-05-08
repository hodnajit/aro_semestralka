#!/usr/bin/env python
import rospy
from nav_msgs.msg import Path, OccupancyGrid
from std_msgs.msg import Header, ColorRGBA
from geometry_msgs.msg import PointStamped, Pose, Vector3, Point
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
import tf.transformations as tft
import tf2_ros
import math 

tresh = 0.2
detections = []
def getDist(pose1,pose2):
    x1 = pose1.x
    y1 = pose1.y
    x2 = pose2.x
    y2 = pose2.y
    return math.hypot(x2 - x1, y2 - y1)

def barbie_cb(msg):

    detections.append(msg.point)
    counter = []
    counter= [0] * len(detections)    

    for index,det in enumerate(detections):
        for nn in detections:
            if getDist(det,nn) < tresh :
                counter[index] +=1

    transforming = True
    try:
        trans = self.tfBuffer.lookup_transform("camera_rgb_optical_frame", "map", msg.header.stamp, rospy.Duration(0.5))
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        rospy.logwarn("Cannot get the odom position!")
        transforming = False

    if transforming:
        pose_transformed = tf2_geometry_msgs.do_transform_pose(msg.point, trans)
    else:
        pose_transformed =msg.point

    header = Header(stamp=rospy.Time.now(), frame_id="map")
    msg = Marker(header=header, pose=Pose(position=pose_transformed), id=np.random.randint(0, 1e9), type=Marker.SPHERE, scale=Vector3(0.04, 0.04, 0.04), color=ColorRGBA(1, 1, 1, 1), lifetime=rospy.Duration(0))
    markerAllPublisher.publish(msg)
    best = detections[counter.index(max(counter))]#find index of largest

    if transforming:
        pose_transformed = tf2_geometry_msgs.do_transform_pose(best, trans)
    else:
        pose_transformed =best


    msg = Marker(header=header, pose=Pose(position=pose_transformed), id=1, type=Marker.SPHERE, scale=Vector3(0.1, 0.1, 0.1), color=ColorRGBA(1, 0, 0, 1), lifetime=rospy.Duration(0))
    markerPublisher.publish(msg)



    barbPublisher.publish(pose_transformed)


if __name__ == "__main__":
    rospy.init_node("barbie_selection")
    print("start")
    tfBuffer = tf2_ros.Buffer()
    barbSubscriber = rospy.Subscriber('barbie_point', PointStamped, barbie_cb)
    barbPublisher = rospy.Publisher('barbie_position_final',Point, queue_size=2)
    markerAllPublisher = rospy.Publisher('barbie_marker_all',Marker, queue_size=2)
    markerPublisher = rospy.Publisher('barbie_marker',Marker, queue_size=2)
    tfListener = tf2_ros.TransformListener(tfBuffer)

    rospy.spin()


