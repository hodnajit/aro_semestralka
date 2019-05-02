; Auto-generated. Do not edit!


(cl:in-package dummy_grid-srv)


;//! \htmlinclude DrawGrid-request.msg.html

(cl:defclass <DrawGrid-request> (roslisp-msg-protocol:ros-message)
  ((points
    :reader points
    :initarg :points
    :type (cl:vector dummy_grid-msg:ValuePoint)
   :initform (cl:make-array 0 :element-type 'dummy_grid-msg:ValuePoint :initial-element (cl:make-instance 'dummy_grid-msg:ValuePoint))))
)

(cl:defclass DrawGrid-request (<DrawGrid-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DrawGrid-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DrawGrid-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dummy_grid-srv:<DrawGrid-request> is deprecated: use dummy_grid-srv:DrawGrid-request instead.")))

(cl:ensure-generic-function 'points-val :lambda-list '(m))
(cl:defmethod points-val ((m <DrawGrid-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dummy_grid-srv:points-val is deprecated.  Use dummy_grid-srv:points instead.")
  (points m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DrawGrid-request>) ostream)
  "Serializes a message object of type '<DrawGrid-request>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'points))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'points))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DrawGrid-request>) istream)
  "Deserializes a message object of type '<DrawGrid-request>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'points) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'points)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'dummy_grid-msg:ValuePoint))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DrawGrid-request>)))
  "Returns string type for a service object of type '<DrawGrid-request>"
  "dummy_grid/DrawGridRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DrawGrid-request)))
  "Returns string type for a service object of type 'DrawGrid-request"
  "dummy_grid/DrawGridRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DrawGrid-request>)))
  "Returns md5sum for a message object of type '<DrawGrid-request>"
  "58bbaa800dd9242d5466364decd790f6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DrawGrid-request)))
  "Returns md5sum for a message object of type 'DrawGrid-request"
  "58bbaa800dd9242d5466364decd790f6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DrawGrid-request>)))
  "Returns full string definition for message of type '<DrawGrid-request>"
  (cl:format cl:nil "dummy_grid/ValuePoint[] points~%~%================================================================================~%MSG: dummy_grid/ValuePoint~%int16 x~%int16 y~%int8 value~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DrawGrid-request)))
  "Returns full string definition for message of type 'DrawGrid-request"
  (cl:format cl:nil "dummy_grid/ValuePoint[] points~%~%================================================================================~%MSG: dummy_grid/ValuePoint~%int16 x~%int16 y~%int8 value~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DrawGrid-request>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'points) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DrawGrid-request>))
  "Converts a ROS message object to a list"
  (cl:list 'DrawGrid-request
    (cl:cons ':points (points msg))
))
;//! \htmlinclude DrawGrid-response.msg.html

(cl:defclass <DrawGrid-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass DrawGrid-response (<DrawGrid-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DrawGrid-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DrawGrid-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dummy_grid-srv:<DrawGrid-response> is deprecated: use dummy_grid-srv:DrawGrid-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DrawGrid-response>) ostream)
  "Serializes a message object of type '<DrawGrid-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DrawGrid-response>) istream)
  "Deserializes a message object of type '<DrawGrid-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DrawGrid-response>)))
  "Returns string type for a service object of type '<DrawGrid-response>"
  "dummy_grid/DrawGridResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DrawGrid-response)))
  "Returns string type for a service object of type 'DrawGrid-response"
  "dummy_grid/DrawGridResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DrawGrid-response>)))
  "Returns md5sum for a message object of type '<DrawGrid-response>"
  "58bbaa800dd9242d5466364decd790f6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DrawGrid-response)))
  "Returns md5sum for a message object of type 'DrawGrid-response"
  "58bbaa800dd9242d5466364decd790f6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DrawGrid-response>)))
  "Returns full string definition for message of type '<DrawGrid-response>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DrawGrid-response)))
  "Returns full string definition for message of type 'DrawGrid-response"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DrawGrid-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DrawGrid-response>))
  "Converts a ROS message object to a list"
  (cl:list 'DrawGrid-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'DrawGrid)))
  'DrawGrid-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'DrawGrid)))
  'DrawGrid-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DrawGrid)))
  "Returns string type for a service object of type '<DrawGrid>"
  "dummy_grid/DrawGrid")