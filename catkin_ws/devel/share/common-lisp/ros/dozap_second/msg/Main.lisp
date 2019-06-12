; Auto-generated. Do not edit!


(cl:in-package dozap_second-msg)


;//! \htmlinclude Main.msg.html

(cl:defclass <Main> (roslisp-msg-protocol:ros-message)
  ((motor_right
    :reader motor_right
    :initarg :motor_right
    :type cl:float
    :initform 0.0)
   (motor_left
    :reader motor_left
    :initarg :motor_left
    :type cl:float
    :initform 0.0)
   (rotation_a_right
    :reader rotation_a_right
    :initarg :rotation_a_right
    :type cl:integer
    :initform 0)
   (rotation_a_left
    :reader rotation_a_left
    :initarg :rotation_a_left
    :type cl:integer
    :initform 0)
   (rotation_b_right
    :reader rotation_b_right
    :initarg :rotation_b_right
    :type cl:integer
    :initform 0)
   (rotation_b_left
    :reader rotation_b_left
    :initarg :rotation_b_left
    :type cl:integer
    :initform 0))
)

(cl:defclass Main (<Main>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Main>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Main)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dozap_second-msg:<Main> is deprecated: use dozap_second-msg:Main instead.")))

(cl:ensure-generic-function 'motor_right-val :lambda-list '(m))
(cl:defmethod motor_right-val ((m <Main>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dozap_second-msg:motor_right-val is deprecated.  Use dozap_second-msg:motor_right instead.")
  (motor_right m))

(cl:ensure-generic-function 'motor_left-val :lambda-list '(m))
(cl:defmethod motor_left-val ((m <Main>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dozap_second-msg:motor_left-val is deprecated.  Use dozap_second-msg:motor_left instead.")
  (motor_left m))

(cl:ensure-generic-function 'rotation_a_right-val :lambda-list '(m))
(cl:defmethod rotation_a_right-val ((m <Main>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dozap_second-msg:rotation_a_right-val is deprecated.  Use dozap_second-msg:rotation_a_right instead.")
  (rotation_a_right m))

(cl:ensure-generic-function 'rotation_a_left-val :lambda-list '(m))
(cl:defmethod rotation_a_left-val ((m <Main>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dozap_second-msg:rotation_a_left-val is deprecated.  Use dozap_second-msg:rotation_a_left instead.")
  (rotation_a_left m))

(cl:ensure-generic-function 'rotation_b_right-val :lambda-list '(m))
(cl:defmethod rotation_b_right-val ((m <Main>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dozap_second-msg:rotation_b_right-val is deprecated.  Use dozap_second-msg:rotation_b_right instead.")
  (rotation_b_right m))

(cl:ensure-generic-function 'rotation_b_left-val :lambda-list '(m))
(cl:defmethod rotation_b_left-val ((m <Main>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dozap_second-msg:rotation_b_left-val is deprecated.  Use dozap_second-msg:rotation_b_left instead.")
  (rotation_b_left m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Main>) ostream)
  "Serializes a message object of type '<Main>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'motor_right))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'motor_left))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let* ((signed (cl:slot-value msg 'rotation_a_right)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'rotation_a_left)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'rotation_b_right)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'rotation_b_left)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Main>) istream)
  "Deserializes a message object of type '<Main>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'motor_right) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'motor_left) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'rotation_a_right) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'rotation_a_left) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'rotation_b_right) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'rotation_b_left) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Main>)))
  "Returns string type for a message object of type '<Main>"
  "dozap_second/Main")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Main)))
  "Returns string type for a message object of type 'Main"
  "dozap_second/Main")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Main>)))
  "Returns md5sum for a message object of type '<Main>"
  "bf323843762cfb3e8a894aa82e16c9f6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Main)))
  "Returns md5sum for a message object of type 'Main"
  "bf323843762cfb3e8a894aa82e16c9f6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Main>)))
  "Returns full string definition for message of type '<Main>"
  (cl:format cl:nil "float64 motor_right~%float64 motor_left~%int32 rotation_a_right~%int32 rotation_a_left~%int32 rotation_b_right~%int32 rotation_b_left~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Main)))
  "Returns full string definition for message of type 'Main"
  (cl:format cl:nil "float64 motor_right~%float64 motor_left~%int32 rotation_a_right~%int32 rotation_a_left~%int32 rotation_b_right~%int32 rotation_b_left~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Main>))
  (cl:+ 0
     8
     8
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Main>))
  "Converts a ROS message object to a list"
  (cl:list 'Main
    (cl:cons ':motor_right (motor_right msg))
    (cl:cons ':motor_left (motor_left msg))
    (cl:cons ':rotation_a_right (rotation_a_right msg))
    (cl:cons ':rotation_a_left (rotation_a_left msg))
    (cl:cons ':rotation_b_right (rotation_b_right msg))
    (cl:cons ':rotation_b_left (rotation_b_left msg))
))
