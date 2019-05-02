;; Auto-generated. Do not edit!


(when (boundp 'exploration::GenerateFrontier)
  (if (not (find-package "EXPLORATION"))
    (make-package "EXPLORATION"))
  (shadow 'GenerateFrontier (find-package "EXPLORATION")))
(unless (find-package "EXPLORATION::GENERATEFRONTIER")
  (make-package "EXPLORATION::GENERATEFRONTIER"))
(unless (find-package "EXPLORATION::GENERATEFRONTIERREQUEST")
  (make-package "EXPLORATION::GENERATEFRONTIERREQUEST"))
(unless (find-package "EXPLORATION::GENERATEFRONTIERRESPONSE")
  (make-package "EXPLORATION::GENERATEFRONTIERRESPONSE"))

(in-package "ROS")



(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass exploration::GenerateFrontierRequest
  :super ros::object
  :slots ())

(defmethod exploration::GenerateFrontierRequest
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(defclass exploration::GenerateFrontierResponse
  :super ros::object
  :slots (_goal_pose ))

(defmethod exploration::GenerateFrontierResponse
  (:init
   (&key
    ((:goal_pose __goal_pose) (instance geometry_msgs::Pose2D :init))
    )
   (send-super :init)
   (setq _goal_pose __goal_pose)
   self)
  (:goal_pose
   (&rest __goal_pose)
   (if (keywordp (car __goal_pose))
       (send* _goal_pose __goal_pose)
     (progn
       (if __goal_pose (setq _goal_pose (car __goal_pose)))
       _goal_pose)))
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/Pose2D _goal_pose
    (send _goal_pose :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/Pose2D _goal_pose
       (send _goal_pose :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/Pose2D _goal_pose
     (send _goal_pose :deserialize buf ptr-) (incf ptr- (send _goal_pose :serialization-length))
   ;;
   self)
  )

(defclass exploration::GenerateFrontier
  :super ros::object
  :slots ())

(setf (get exploration::GenerateFrontier :md5sum-) "9c696f8357da6184167237a9d05c73bc")
(setf (get exploration::GenerateFrontier :datatype-) "exploration/GenerateFrontier")
(setf (get exploration::GenerateFrontier :request) exploration::GenerateFrontierRequest)
(setf (get exploration::GenerateFrontier :response) exploration::GenerateFrontierResponse)

(defmethod exploration::GenerateFrontierRequest
  (:response () (instance exploration::GenerateFrontierResponse :init)))

(setf (get exploration::GenerateFrontierRequest :md5sum-) "9c696f8357da6184167237a9d05c73bc")
(setf (get exploration::GenerateFrontierRequest :datatype-) "exploration/GenerateFrontierRequest")
(setf (get exploration::GenerateFrontierRequest :definition-)
      "---
geometry_msgs/Pose2D goal_pose

================================================================================
MSG: geometry_msgs/Pose2D
# Deprecated
# Please use the full 3D pose.

# In general our recommendation is to use a full 3D representation of everything and for 2D specific applications make the appropriate projections into the plane for their calculations but optimally will preserve the 3D information during processing.

# If we have parallel copies of 2D datatypes every UI and other pipeline will end up needing to have dual interfaces to plot everything. And you will end up with not being able to use 3D tools for 2D use cases even if they're completely valid, as you'd have to reimplement it with different inputs and outputs. It's not particularly hard to plot the 2D pose or compute the yaw error for the Pose message and there are already tools and libraries that can do this for you.


# This expresses a position and orientation on a 2D manifold.

float64 x
float64 y
float64 theta
")

(setf (get exploration::GenerateFrontierResponse :md5sum-) "9c696f8357da6184167237a9d05c73bc")
(setf (get exploration::GenerateFrontierResponse :datatype-) "exploration/GenerateFrontierResponse")
(setf (get exploration::GenerateFrontierResponse :definition-)
      "---
geometry_msgs/Pose2D goal_pose

================================================================================
MSG: geometry_msgs/Pose2D
# Deprecated
# Please use the full 3D pose.

# In general our recommendation is to use a full 3D representation of everything and for 2D specific applications make the appropriate projections into the plane for their calculations but optimally will preserve the 3D information during processing.

# If we have parallel copies of 2D datatypes every UI and other pipeline will end up needing to have dual interfaces to plot everything. And you will end up with not being able to use 3D tools for 2D use cases even if they're completely valid, as you'd have to reimplement it with different inputs and outputs. It's not particularly hard to plot the 2D pose or compute the yaw error for the Pose message and there are already tools and libraries that can do this for you.


# This expresses a position and orientation on a 2D manifold.

float64 x
float64 y
float64 theta
")



(provide :exploration/GenerateFrontier "9c696f8357da6184167237a9d05c73bc")


