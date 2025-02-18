;; Auto-generated. Do not edit!


(when (boundp 'dvs_msgs::Event)
  (if (not (find-package "DVS_MSGS"))
    (make-package "DVS_MSGS"))
  (shadow 'Event (find-package "DVS_MSGS")))
(unless (find-package "DVS_MSGS::EVENT")
  (make-package "DVS_MSGS::EVENT"))

(in-package "ROS")
;;//! \htmlinclude Event.msg.html


(defclass dvs_msgs::Event
  :super ros::object
  :slots (_x _y _ts _polarity ))

(defmethod dvs_msgs::Event
  (:init
   (&key
    ((:x __x) 0)
    ((:y __y) 0)
    ((:ts __ts) (instance ros::time :init))
    ((:polarity __polarity) nil)
    )
   (send-super :init)
   (setq _x (round __x))
   (setq _y (round __y))
   (setq _ts __ts)
   (setq _polarity __polarity)
   self)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:ts
   (&optional __ts)
   (if __ts (setq _ts __ts)) _ts)
  (:polarity
   (&optional __polarity)
   (if __polarity (setq _polarity __polarity)) _polarity)
  (:serialization-length
   ()
   (+
    ;; uint16 _x
    2
    ;; uint16 _y
    2
    ;; time _ts
    8
    ;; bool _polarity
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint16 _x
       (write-word _x s)
     ;; uint16 _y
       (write-word _y s)
     ;; time _ts
       (write-long (send _ts :sec) s) (write-long (send _ts :nsec) s)
     ;; bool _polarity
       (if _polarity (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint16 _x
     (setq _x (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint16 _y
     (setq _y (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; time _ts
     (send _ts :sec (sys::peek buf ptr- :integer)) (incf ptr- 4)  (send _ts :nsec (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; bool _polarity
     (setq _polarity (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get dvs_msgs::Event :md5sum-) "40d339b261445b8ce9a6fef01267cd5d")
(setf (get dvs_msgs::Event :datatype-) "dvs_msgs/Event")
(setf (get dvs_msgs::Event :definition-)
      "# A DVS event
uint16 x
uint16 y
time ts
bool polarity

")



(provide :dvs_msgs/Event "40d339b261445b8ce9a6fef01267cd5d")


