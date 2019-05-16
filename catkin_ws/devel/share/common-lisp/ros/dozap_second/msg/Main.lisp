; Auto-generated. Do not edit!


(cl:in-package dozap_second-msg)


;//! \htmlinclude Main.msg.html

(cl:defclass <Main> (roslisp-msg-protocol:ros-message)
  ((motor_right
    :reader motor_right
    :initarg :motor_right
    :type cl:integer
    :initform 0)
   (motor_left
    :reader motor_left
    :initarg :motor_left
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
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Main>) ostream)
  "Serializes a message object of type '<Main>"
  (cl:let* ((signed (cl:slot-value msg 'motor_right)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'motor_left)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Main>) istream)
  "Deserializes a message object of type '<Main>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'motor_right) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'motor_left) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
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
  "799f128dce14b1811f847ac0f5950039")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Main)))
  "Returns md5sum for a message object of type 'Main"
  "799f128dce14b1811f847ac0f5950039")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Main>)))
  "Returns full string definition for message of type '<Main>"
  (cl:format cl:nil "int32 motor_right~%int32 motor_left~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Main)))
  "Returns full string definition for message of type 'Main"
  (cl:format cl:nil "int32 motor_right~%int32 motor_left~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Main>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Main>))
  "Converts a ROS message object to a list"
  (cl:list 'Main
    (cl:cons ':motor_right (motor_right msg))
    (cl:cons ':motor_left (motor_left msg))
))
