;; Auto-generated. Do not edit!


(when (boundp 'dvs_msgs::EventArray)
  (if (not (find-package "DVS_MSGS"))
    (make-package "DVS_MSGS"))
  (shadow 'EventArray (find-package "DVS_MSGS")))
(unless (find-package "DVS_MSGS::EVENTARRAY")
  (make-package "DVS_MSGS::EVENTARRAY"))

(in-package "ROS")
;;//! \htmlinclude EventArray.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass dvs_msgs::EventArray
  :super ros::object
  :slots (_header _height _width _events ))

(defmethod dvs_msgs::EventArray
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:height __height) 0)
    ((:width __width) 0)
    ((:events __events) (let (r) (dotimes (i 0) (push (instance dvs_msgs::Event :init) r)) r))
    )
   (send-super :init)
   (setq _header __header)
   (setq _height (round __height))
   (setq _width (round __width))
   (setq _events __events)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:height
   (&optional __height)
   (if __height (setq _height __height)) _height)
  (:width
   (&optional __width)
   (if __width (setq _width __width)) _width)
  (:events
   (&rest __events)
   (if (keywordp (car __events))
       (send* _events __events)
     (progn
       (if __events (setq _events (car __events)))
       _events)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uint32 _height
    4
    ;; uint32 _width
    4
    ;; dvs_msgs/Event[] _events
    (apply #'+ (send-all _events :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; uint32 _height
       (write-long _height s)
     ;; uint32 _width
       (write-long _width s)
     ;; dvs_msgs/Event[] _events
     (write-long (length _events) s)
     (dolist (elem _events)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; uint32 _height
     (setq _height (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _width
     (setq _width (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; dvs_msgs/Event[] _events
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _events (let (r) (dotimes (i n) (push (instance dvs_msgs::Event :init) r)) r))
     (dolist (elem- _events)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get dvs_msgs::EventArray :md5sum-) "5e8beee5a6c107e504c2e78903c224b8")
(setf (get dvs_msgs::EventArray :datatype-) "dvs_msgs/EventArray")
(setf (get dvs_msgs::EventArray :definition-)
      "# This message contains an array of events
# (0, 0) is at top-left corner of image
#

Header header

uint32 height         # image height, that is, number of rows
uint32 width          # image width, that is, number of columns

# an array of events
Event[] events

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: dvs_msgs/Event
# A DVS event
uint16 x
uint16 y
time ts
bool polarity

")



(provide :dvs_msgs/EventArray "5e8beee5a6c107e504c2e78903c224b8")


