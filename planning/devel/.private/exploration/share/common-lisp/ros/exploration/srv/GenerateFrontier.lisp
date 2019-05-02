; Auto-generated. Do not edit!


(cl:in-package exploration-srv)


;//! \htmlinclude GenerateFrontier-request.msg.html

(cl:defclass <GenerateFrontier-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass GenerateFrontier-request (<GenerateFrontier-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GenerateFrontier-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GenerateFrontier-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name exploration-srv:<GenerateFrontier-request> is deprecated: use exploration-srv:GenerateFrontier-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GenerateFrontier-request>) ostream)
  "Serializes a message object of type '<GenerateFrontier-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GenerateFrontier-request>) istream)
  "Deserializes a message object of type '<GenerateFrontier-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GenerateFrontier-request>)))
  "Returns string type for a service object of type '<GenerateFrontier-request>"
  "exploration/GenerateFrontierRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GenerateFrontier-request)))
  "Returns string type for a service object of type 'GenerateFrontier-request"
  "exploration/GenerateFrontierRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GenerateFrontier-request>)))
  "Returns md5sum for a message object of type '<GenerateFrontier-request>"
  "9c696f8357da6184167237a9d05c73bc")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GenerateFrontier-request)))
  "Returns md5sum for a message object of type 'GenerateFrontier-request"
  "9c696f8357da6184167237a9d05c73bc")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GenerateFrontier-request>)))
  "Returns full string definition for message of type '<GenerateFrontier-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GenerateFrontier-request)))
  "Returns full string definition for message of type 'GenerateFrontier-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GenerateFrontier-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GenerateFrontier-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GenerateFrontier-request
))
;//! \htmlinclude GenerateFrontier-response.msg.html

(cl:defclass <GenerateFrontier-response> (roslisp-msg-protocol:ros-message)
  ((goal_pose
    :reader goal_pose
    :initarg :goal_pose
    :type geometry_msgs-msg:Pose2D
    :initform (cl:make-instance 'geometry_msgs-msg:Pose2D)))
)

(cl:defclass GenerateFrontier-response (<GenerateFrontier-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GenerateFrontier-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GenerateFrontier-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name exploration-srv:<GenerateFrontier-response> is deprecated: use exploration-srv:GenerateFrontier-response instead.")))

(cl:ensure-generic-function 'goal_pose-val :lambda-list '(m))
(cl:defmethod goal_pose-val ((m <GenerateFrontier-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader exploration-srv:goal_pose-val is deprecated.  Use exploration-srv:goal_pose instead.")
  (goal_pose m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GenerateFrontier-response>) ostream)
  "Serializes a message object of type '<GenerateFrontier-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'goal_pose) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GenerateFrontier-response>) istream)
  "Deserializes a message object of type '<GenerateFrontier-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'goal_pose) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GenerateFrontier-response>)))
  "Returns string type for a service object of type '<GenerateFrontier-response>"
  "exploration/GenerateFrontierResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GenerateFrontier-response)))
  "Returns string type for a service object of type 'GenerateFrontier-response"
  "exploration/GenerateFrontierResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GenerateFrontier-response>)))
  "Returns md5sum for a message object of type '<GenerateFrontier-response>"
  "9c696f8357da6184167237a9d05c73bc")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GenerateFrontier-response)))
  "Returns md5sum for a message object of type 'GenerateFrontier-response"
  "9c696f8357da6184167237a9d05c73bc")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GenerateFrontier-response>)))
  "Returns full string definition for message of type '<GenerateFrontier-response>"
  (cl:format cl:nil "geometry_msgs/Pose2D goal_pose~%~%================================================================================~%MSG: geometry_msgs/Pose2D~%# Deprecated~%# Please use the full 3D pose.~%~%# In general our recommendation is to use a full 3D representation of everything and for 2D specific applications make the appropriate projections into the plane for their calculations but optimally will preserve the 3D information during processing.~%~%# If we have parallel copies of 2D datatypes every UI and other pipeline will end up needing to have dual interfaces to plot everything. And you will end up with not being able to use 3D tools for 2D use cases even if they're completely valid, as you'd have to reimplement it with different inputs and outputs. It's not particularly hard to plot the 2D pose or compute the yaw error for the Pose message and there are already tools and libraries that can do this for you.~%~%~%# This expresses a position and orientation on a 2D manifold.~%~%float64 x~%float64 y~%float64 theta~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GenerateFrontier-response)))
  "Returns full string definition for message of type 'GenerateFrontier-response"
  (cl:format cl:nil "geometry_msgs/Pose2D goal_pose~%~%================================================================================~%MSG: geometry_msgs/Pose2D~%# Deprecated~%# Please use the full 3D pose.~%~%# In general our recommendation is to use a full 3D representation of everything and for 2D specific applications make the appropriate projections into the plane for their calculations but optimally will preserve the 3D information during processing.~%~%# If we have parallel copies of 2D datatypes every UI and other pipeline will end up needing to have dual interfaces to plot everything. And you will end up with not being able to use 3D tools for 2D use cases even if they're completely valid, as you'd have to reimplement it with different inputs and outputs. It's not particularly hard to plot the 2D pose or compute the yaw error for the Pose message and there are already tools and libraries that can do this for you.~%~%~%# This expresses a position and orientation on a 2D manifold.~%~%float64 x~%float64 y~%float64 theta~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GenerateFrontier-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'goal_pose))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GenerateFrontier-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GenerateFrontier-response
    (cl:cons ':goal_pose (goal_pose msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GenerateFrontier)))
  'GenerateFrontier-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GenerateFrontier)))
  'GenerateFrontier-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GenerateFrontier)))
  "Returns string type for a service object of type '<GenerateFrontier>"
  "exploration/GenerateFrontier")