;; Auto-generated. Do not edit!


(when (boundp 'reference_governor::polyTraj)
  (if (not (find-package "REFERENCE_GOVERNOR"))
    (make-package "REFERENCE_GOVERNOR"))
  (shadow 'polyTraj (find-package "REFERENCE_GOVERNOR")))
(unless (find-package "REFERENCE_GOVERNOR::POLYTRAJ")
  (make-package "REFERENCE_GOVERNOR::POLYTRAJ"))

(in-package "ROS")
;;//! \htmlinclude polyTraj.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass reference_governor::polyTraj
  :super ros::object
  :slots (_header _start_planning_time _poly_coefs _poly_duration ))

(defmethod reference_governor::polyTraj
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:start_planning_time __start_planning_time) (instance std_msgs::Time :init))
    ((:poly_coefs __poly_coefs) (instance std_msgs::Float64MultiArray :init))
    ((:poly_duration __poly_duration) (instance std_msgs::Float64 :init))
    )
   (send-super :init)
   (setq _header __header)
   (setq _start_planning_time __start_planning_time)
   (setq _poly_coefs __poly_coefs)
   (setq _poly_duration __poly_duration)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:start_planning_time
   (&rest __start_planning_time)
   (if (keywordp (car __start_planning_time))
       (send* _start_planning_time __start_planning_time)
     (progn
       (if __start_planning_time (setq _start_planning_time (car __start_planning_time)))
       _start_planning_time)))
  (:poly_coefs
   (&rest __poly_coefs)
   (if (keywordp (car __poly_coefs))
       (send* _poly_coefs __poly_coefs)
     (progn
       (if __poly_coefs (setq _poly_coefs (car __poly_coefs)))
       _poly_coefs)))
  (:poly_duration
   (&rest __poly_duration)
   (if (keywordp (car __poly_duration))
       (send* _poly_duration __poly_duration)
     (progn
       (if __poly_duration (setq _poly_duration (car __poly_duration)))
       _poly_duration)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; std_msgs/Time _start_planning_time
    (send _start_planning_time :serialization-length)
    ;; std_msgs/Float64MultiArray _poly_coefs
    (send _poly_coefs :serialization-length)
    ;; std_msgs/Float64 _poly_duration
    (send _poly_duration :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; std_msgs/Time _start_planning_time
       (send _start_planning_time :serialize s)
     ;; std_msgs/Float64MultiArray _poly_coefs
       (send _poly_coefs :serialize s)
     ;; std_msgs/Float64 _poly_duration
       (send _poly_duration :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; std_msgs/Time _start_planning_time
     (send _start_planning_time :deserialize buf ptr-) (incf ptr- (send _start_planning_time :serialization-length))
   ;; std_msgs/Float64MultiArray _poly_coefs
     (send _poly_coefs :deserialize buf ptr-) (incf ptr- (send _poly_coefs :serialization-length))
   ;; std_msgs/Float64 _poly_duration
     (send _poly_duration :deserialize buf ptr-) (incf ptr- (send _poly_duration :serialization-length))
   ;;
   self)
  )

(setf (get reference_governor::polyTraj :md5sum-) "71bc53ae599621f980c4732c605466da")
(setf (get reference_governor::polyTraj :datatype-) "reference_governor/polyTraj")
(setf (get reference_governor::polyTraj :definition-)
      "std_msgs/Header header
std_msgs/Time start_planning_time
std_msgs/Float64MultiArray poly_coefs
std_msgs/Float64 poly_duration
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
MSG: std_msgs/Time
time data

================================================================================
MSG: std_msgs/Float64MultiArray
# Please look at the MultiArrayLayout message definition for
# documentation on all multiarrays.

MultiArrayLayout  layout        # specification of data layout
float64[]         data          # array of data


================================================================================
MSG: std_msgs/MultiArrayLayout
# The multiarray declares a generic multi-dimensional array of a
# particular data type.  Dimensions are ordered from outer most
# to inner most.

MultiArrayDimension[] dim # Array of dimension properties
uint32 data_offset        # padding elements at front of data

# Accessors should ALWAYS be written in terms of dimension stride
# and specified outer-most dimension first.
# 
# multiarray(i,j,k) = data[data_offset + dim_stride[1]*i + dim_stride[2]*j + k]
#
# A standard, 3-channel 640x480 image with interleaved color channels
# would be specified as:
#
# dim[0].label  = \"height\"
# dim[0].size   = 480
# dim[0].stride = 3*640*480 = 921600  (note dim[0] stride is just size of image)
# dim[1].label  = \"width\"
# dim[1].size   = 640
# dim[1].stride = 3*640 = 1920
# dim[2].label  = \"channel\"
# dim[2].size   = 3
# dim[2].stride = 3
#
# multiarray(i,j,k) refers to the ith row, jth column, and kth channel.

================================================================================
MSG: std_msgs/MultiArrayDimension
string label   # label of given dimension
uint32 size    # size of given dimension (in type units)
uint32 stride  # stride of given dimension
================================================================================
MSG: std_msgs/Float64
float64 data
")



(provide :reference_governor/polyTraj "71bc53ae599621f980c4732c605466da")


