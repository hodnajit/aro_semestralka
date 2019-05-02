; Auto-generated. Do not edit!


(cl:in-package exploration-srv)


;//! \htmlinclude AnyFrontiersLeft-request.msg.html

(cl:defclass <AnyFrontiersLeft-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass AnyFrontiersLeft-request (<AnyFrontiersLeft-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <AnyFrontiersLeft-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'AnyFrontiersLeft-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name exploration-srv:<AnyFrontiersLeft-request> is deprecated: use exploration-srv:AnyFrontiersLeft-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <AnyFrontiersLeft-request>) ostream)
  "Serializes a message object of type '<AnyFrontiersLeft-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <AnyFrontiersLeft-request>) istream)
  "Deserializes a message object of type '<AnyFrontiersLeft-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<AnyFrontiersLeft-request>)))
  "Returns string type for a service object of type '<AnyFrontiersLeft-request>"
  "exploration/AnyFrontiersLeftRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AnyFrontiersLeft-request)))
  "Returns string type for a service object of type 'AnyFrontiersLeft-request"
  "exploration/AnyFrontiersLeftRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<AnyFrontiersLeft-request>)))
  "Returns md5sum for a message object of type '<AnyFrontiersLeft-request>"
  "9c55a8e73f02e61ed1fd7718907efa64")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'AnyFrontiersLeft-request)))
  "Returns md5sum for a message object of type 'AnyFrontiersLeft-request"
  "9c55a8e73f02e61ed1fd7718907efa64")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<AnyFrontiersLeft-request>)))
  "Returns full string definition for message of type '<AnyFrontiersLeft-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'AnyFrontiersLeft-request)))
  "Returns full string definition for message of type 'AnyFrontiersLeft-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <AnyFrontiersLeft-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <AnyFrontiersLeft-request>))
  "Converts a ROS message object to a list"
  (cl:list 'AnyFrontiersLeft-request
))
;//! \htmlinclude AnyFrontiersLeft-response.msg.html

(cl:defclass <AnyFrontiersLeft-response> (roslisp-msg-protocol:ros-message)
  ((any_frontiers_left
    :reader any_frontiers_left
    :initarg :any_frontiers_left
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass AnyFrontiersLeft-response (<AnyFrontiersLeft-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <AnyFrontiersLeft-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'AnyFrontiersLeft-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name exploration-srv:<AnyFrontiersLeft-response> is deprecated: use exploration-srv:AnyFrontiersLeft-response instead.")))

(cl:ensure-generic-function 'any_frontiers_left-val :lambda-list '(m))
(cl:defmethod any_frontiers_left-val ((m <AnyFrontiersLeft-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader exploration-srv:any_frontiers_left-val is deprecated.  Use exploration-srv:any_frontiers_left instead.")
  (any_frontiers_left m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <AnyFrontiersLeft-response>) ostream)
  "Serializes a message object of type '<AnyFrontiersLeft-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'any_frontiers_left) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <AnyFrontiersLeft-response>) istream)
  "Deserializes a message object of type '<AnyFrontiersLeft-response>"
    (cl:setf (cl:slot-value msg 'any_frontiers_left) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<AnyFrontiersLeft-response>)))
  "Returns string type for a service object of type '<AnyFrontiersLeft-response>"
  "exploration/AnyFrontiersLeftResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AnyFrontiersLeft-response)))
  "Returns string type for a service object of type 'AnyFrontiersLeft-response"
  "exploration/AnyFrontiersLeftResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<AnyFrontiersLeft-response>)))
  "Returns md5sum for a message object of type '<AnyFrontiersLeft-response>"
  "9c55a8e73f02e61ed1fd7718907efa64")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'AnyFrontiersLeft-response)))
  "Returns md5sum for a message object of type 'AnyFrontiersLeft-response"
  "9c55a8e73f02e61ed1fd7718907efa64")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<AnyFrontiersLeft-response>)))
  "Returns full string definition for message of type '<AnyFrontiersLeft-response>"
  (cl:format cl:nil "bool any_frontiers_left~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'AnyFrontiersLeft-response)))
  "Returns full string definition for message of type 'AnyFrontiersLeft-response"
  (cl:format cl:nil "bool any_frontiers_left~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <AnyFrontiersLeft-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <AnyFrontiersLeft-response>))
  "Converts a ROS message object to a list"
  (cl:list 'AnyFrontiersLeft-response
    (cl:cons ':any_frontiers_left (any_frontiers_left msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'AnyFrontiersLeft)))
  'AnyFrontiersLeft-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'AnyFrontiersLeft)))
  'AnyFrontiersLeft-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AnyFrontiersLeft)))
  "Returns string type for a service object of type '<AnyFrontiersLeft>"
  "exploration/AnyFrontiersLeft")