; Auto-generated. Do not edit!


(cl:in-package dvs_msgs-msg)


;//! \htmlinclude EventArray.msg.html

(cl:defclass <EventArray> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (height
    :reader height
    :initarg :height
    :type cl:integer
    :initform 0)
   (width
    :reader width
    :initarg :width
    :type cl:integer
    :initform 0)
   (events
    :reader events
    :initarg :events
    :type (cl:vector dvs_msgs-msg:Event)
   :initform (cl:make-array 0 :element-type 'dvs_msgs-msg:Event :initial-element (cl:make-instance 'dvs_msgs-msg:Event))))
)

(cl:defclass EventArray (<EventArray>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <EventArray>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'EventArray)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dvs_msgs-msg:<EventArray> is deprecated: use dvs_msgs-msg:EventArray instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <EventArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dvs_msgs-msg:header-val is deprecated.  Use dvs_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'height-val :lambda-list '(m))
(cl:defmethod height-val ((m <EventArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dvs_msgs-msg:height-val is deprecated.  Use dvs_msgs-msg:height instead.")
  (height m))

(cl:ensure-generic-function 'width-val :lambda-list '(m))
(cl:defmethod width-val ((m <EventArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dvs_msgs-msg:width-val is deprecated.  Use dvs_msgs-msg:width instead.")
  (width m))

(cl:ensure-generic-function 'events-val :lambda-list '(m))
(cl:defmethod events-val ((m <EventArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dvs_msgs-msg:events-val is deprecated.  Use dvs_msgs-msg:events instead.")
  (events m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <EventArray>) ostream)
  "Serializes a message object of type '<EventArray>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'height)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'height)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'height)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'height)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'width)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'width)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'width)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'width)) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'events))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'events))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <EventArray>) istream)
  "Deserializes a message object of type '<EventArray>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'height)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'height)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'height)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'height)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'width)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'width)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'width)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'width)) (cl:read-byte istream))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'events) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'events)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'dvs_msgs-msg:Event))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<EventArray>)))
  "Returns string type for a message object of type '<EventArray>"
  "dvs_msgs/EventArray")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'EventArray)))
  "Returns string type for a message object of type 'EventArray"
  "dvs_msgs/EventArray")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<EventArray>)))
  "Returns md5sum for a message object of type '<EventArray>"
  "5e8beee5a6c107e504c2e78903c224b8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'EventArray)))
  "Returns md5sum for a message object of type 'EventArray"
  "5e8beee5a6c107e504c2e78903c224b8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<EventArray>)))
  "Returns full string definition for message of type '<EventArray>"
  (cl:format cl:nil "# This message contains an array of events~%# (0, 0) is at top-left corner of image~%#~%~%Header header~%~%uint32 height         # image height, that is, number of rows~%uint32 width          # image width, that is, number of columns~%~%# an array of events~%Event[] events~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: dvs_msgs/Event~%# A DVS event~%uint16 x~%uint16 y~%time ts~%bool polarity~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'EventArray)))
  "Returns full string definition for message of type 'EventArray"
  (cl:format cl:nil "# This message contains an array of events~%# (0, 0) is at top-left corner of image~%#~%~%Header header~%~%uint32 height         # image height, that is, number of rows~%uint32 width          # image width, that is, number of columns~%~%# an array of events~%Event[] events~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: dvs_msgs/Event~%# A DVS event~%uint16 x~%uint16 y~%time ts~%bool polarity~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <EventArray>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4
     4
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'events) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <EventArray>))
  "Converts a ROS message object to a list"
  (cl:list 'EventArray
    (cl:cons ':header (header msg))
    (cl:cons ':height (height msg))
    (cl:cons ':width (width msg))
    (cl:cons ':events (events msg))
))
