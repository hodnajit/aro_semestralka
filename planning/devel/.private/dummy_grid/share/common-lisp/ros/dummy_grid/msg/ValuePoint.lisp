; Auto-generated. Do not edit!


(cl:in-package dummy_grid-msg)


;//! \htmlinclude ValuePoint.msg.html

(cl:defclass <ValuePoint> (roslisp-msg-protocol:ros-message)
  ((x
    :reader x
    :initarg :x
    :type cl:fixnum
    :initform 0)
   (y
    :reader y
    :initarg :y
    :type cl:fixnum
    :initform 0)
   (value
    :reader value
    :initarg :value
    :type cl:fixnum
    :initform 0))
)

(cl:defclass ValuePoint (<ValuePoint>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ValuePoint>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ValuePoint)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dummy_grid-msg:<ValuePoint> is deprecated: use dummy_grid-msg:ValuePoint instead.")))

(cl:ensure-generic-function 'x-val :lambda-list '(m))
(cl:defmethod x-val ((m <ValuePoint>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dummy_grid-msg:x-val is deprecated.  Use dummy_grid-msg:x instead.")
  (x m))

(cl:ensure-generic-function 'y-val :lambda-list '(m))
(cl:defmethod y-val ((m <ValuePoint>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dummy_grid-msg:y-val is deprecated.  Use dummy_grid-msg:y instead.")
  (y m))

(cl:ensure-generic-function 'value-val :lambda-list '(m))
(cl:defmethod value-val ((m <ValuePoint>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dummy_grid-msg:value-val is deprecated.  Use dummy_grid-msg:value instead.")
  (value m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ValuePoint>) ostream)
  "Serializes a message object of type '<ValuePoint>"
  (cl:let* ((signed (cl:slot-value msg 'x)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'y)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'value)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ValuePoint>) istream)
  "Deserializes a message object of type '<ValuePoint>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'x) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'y) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'value) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ValuePoint>)))
  "Returns string type for a message object of type '<ValuePoint>"
  "dummy_grid/ValuePoint")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ValuePoint)))
  "Returns string type for a message object of type 'ValuePoint"
  "dummy_grid/ValuePoint")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ValuePoint>)))
  "Returns md5sum for a message object of type '<ValuePoint>"
  "f251dc5cdacac9c0ea3ef6f992e4d62a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ValuePoint)))
  "Returns md5sum for a message object of type 'ValuePoint"
  "f251dc5cdacac9c0ea3ef6f992e4d62a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ValuePoint>)))
  "Returns full string definition for message of type '<ValuePoint>"
  (cl:format cl:nil "int16 x~%int16 y~%int8 value~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ValuePoint)))
  "Returns full string definition for message of type 'ValuePoint"
  (cl:format cl:nil "int16 x~%int16 y~%int8 value~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ValuePoint>))
  (cl:+ 0
     2
     2
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ValuePoint>))
  "Converts a ROS message object to a list"
  (cl:list 'ValuePoint
    (cl:cons ':x (x msg))
    (cl:cons ':y (y msg))
    (cl:cons ':value (value msg))
))
