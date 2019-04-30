; Auto-generated. Do not edit!


(cl:in-package dozap_second-msg)


;//! \htmlinclude Button.msg.html

(cl:defclass <Button> (roslisp-msg-protocol:ros-message)
  ((move
    :reader move
    :initarg :move
    :type cl:integer
    :initform 0)
   (rotation_right
    :reader rotation_right
    :initarg :rotation_right
    :type cl:integer
    :initform 0)
   (rotation_left
    :reader rotation_left
    :initarg :rotation_left
    :type cl:integer
    :initform 0))
)

(cl:defclass Button (<Button>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Button>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Button)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dozap_second-msg:<Button> is deprecated: use dozap_second-msg:Button instead.")))

(cl:ensure-generic-function 'move-val :lambda-list '(m))
(cl:defmethod move-val ((m <Button>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dozap_second-msg:move-val is deprecated.  Use dozap_second-msg:move instead.")
  (move m))

(cl:ensure-generic-function 'rotation_right-val :lambda-list '(m))
(cl:defmethod rotation_right-val ((m <Button>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dozap_second-msg:rotation_right-val is deprecated.  Use dozap_second-msg:rotation_right instead.")
  (rotation_right m))

(cl:ensure-generic-function 'rotation_left-val :lambda-list '(m))
(cl:defmethod rotation_left-val ((m <Button>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dozap_second-msg:rotation_left-val is deprecated.  Use dozap_second-msg:rotation_left instead.")
  (rotation_left m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Button>) ostream)
  "Serializes a message object of type '<Button>"
  (cl:let* ((signed (cl:slot-value msg 'move)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'rotation_right)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'rotation_left)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Button>) istream)
  "Deserializes a message object of type '<Button>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'move) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'rotation_right) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'rotation_left) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Button>)))
  "Returns string type for a message object of type '<Button>"
  "dozap_second/Button")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Button)))
  "Returns string type for a message object of type 'Button"
  "dozap_second/Button")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Button>)))
  "Returns md5sum for a message object of type '<Button>"
  "df37f56eb5634ffffccd37fb25f1f170")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Button)))
  "Returns md5sum for a message object of type 'Button"
  "df37f56eb5634ffffccd37fb25f1f170")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Button>)))
  "Returns full string definition for message of type '<Button>"
  (cl:format cl:nil "int32 move~%int32 rotation_right~%int32 rotation_left~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Button)))
  "Returns full string definition for message of type 'Button"
  (cl:format cl:nil "int32 move~%int32 rotation_right~%int32 rotation_left~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Button>))
  (cl:+ 0
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Button>))
  "Converts a ROS message object to a list"
  (cl:list 'Button
    (cl:cons ':move (move msg))
    (cl:cons ':rotation_right (rotation_right msg))
    (cl:cons ':rotation_left (rotation_left msg))
))
