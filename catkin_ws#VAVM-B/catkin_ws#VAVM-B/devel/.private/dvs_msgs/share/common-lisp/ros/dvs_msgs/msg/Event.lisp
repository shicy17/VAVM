; Auto-generated. Do not edit!


(cl:in-package dvs_msgs-msg)


;//! \htmlinclude Event.msg.html

(cl:defclass <Event> (roslisp-msg-protocol:ros-message)
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
   (ts
    :reader ts
    :initarg :ts
    :type cl:real
    :initform 0)
   (polarity
    :reader polarity
    :initarg :polarity
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass Event (<Event>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Event>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Event)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dvs_msgs-msg:<Event> is deprecated: use dvs_msgs-msg:Event instead.")))

(cl:ensure-generic-function 'x-val :lambda-list '(m))
(cl:defmethod x-val ((m <Event>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dvs_msgs-msg:x-val is deprecated.  Use dvs_msgs-msg:x instead.")
  (x m))

(cl:ensure-generic-function 'y-val :lambda-list '(m))
(cl:defmethod y-val ((m <Event>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dvs_msgs-msg:y-val is deprecated.  Use dvs_msgs-msg:y instead.")
  (y m))

(cl:ensure-generic-function 'ts-val :lambda-list '(m))
(cl:defmethod ts-val ((m <Event>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dvs_msgs-msg:ts-val is deprecated.  Use dvs_msgs-msg:ts instead.")
  (ts m))

(cl:ensure-generic-function 'polarity-val :lambda-list '(m))
(cl:defmethod polarity-val ((m <Event>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dvs_msgs-msg:polarity-val is deprecated.  Use dvs_msgs-msg:polarity instead.")
  (polarity m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Event>) ostream)
  "Serializes a message object of type '<Event>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'x)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'x)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'y)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'y)) ostream)
  (cl:let ((__sec (cl:floor (cl:slot-value msg 'ts)))
        (__nsec (cl:round (cl:* 1e9 (cl:- (cl:slot-value msg 'ts) (cl:floor (cl:slot-value msg 'ts)))))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 0) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __nsec) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'polarity) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Event>) istream)
  "Deserializes a message object of type '<Event>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'x)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'x)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'y)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'y)) (cl:read-byte istream))
    (cl:let ((__sec 0) (__nsec 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 0) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __nsec) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'ts) (cl:+ (cl:coerce __sec 'cl:double-float) (cl:/ __nsec 1e9))))
    (cl:setf (cl:slot-value msg 'polarity) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Event>)))
  "Returns string type for a message object of type '<Event>"
  "dvs_msgs/Event")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Event)))
  "Returns string type for a message object of type 'Event"
  "dvs_msgs/Event")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Event>)))
  "Returns md5sum for a message object of type '<Event>"
  "40d339b261445b8ce9a6fef01267cd5d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Event)))
  "Returns md5sum for a message object of type 'Event"
  "40d339b261445b8ce9a6fef01267cd5d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Event>)))
  "Returns full string definition for message of type '<Event>"
  (cl:format cl:nil "# A DVS event~%uint16 x~%uint16 y~%time ts~%bool polarity~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Event)))
  "Returns full string definition for message of type 'Event"
  (cl:format cl:nil "# A DVS event~%uint16 x~%uint16 y~%time ts~%bool polarity~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Event>))
  (cl:+ 0
     2
     2
     8
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Event>))
  "Converts a ROS message object to a list"
  (cl:list 'Event
    (cl:cons ':x (x msg))
    (cl:cons ':y (y msg))
    (cl:cons ':ts (ts msg))
    (cl:cons ':polarity (polarity msg))
))
