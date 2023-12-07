; Auto-generated. Do not edit!


(cl:in-package reference_governor-msg)


;//! \htmlinclude polyTraj.msg.html

(cl:defclass <polyTraj> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (start_planning_time
    :reader start_planning_time
    :initarg :start_planning_time
    :type std_msgs-msg:Time
    :initform (cl:make-instance 'std_msgs-msg:Time))
   (poly_coefs
    :reader poly_coefs
    :initarg :poly_coefs
    :type std_msgs-msg:Float64MultiArray
    :initform (cl:make-instance 'std_msgs-msg:Float64MultiArray))
   (poly_duration
    :reader poly_duration
    :initarg :poly_duration
    :type std_msgs-msg:Float64
    :initform (cl:make-instance 'std_msgs-msg:Float64)))
)

(cl:defclass polyTraj (<polyTraj>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <polyTraj>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'polyTraj)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name reference_governor-msg:<polyTraj> is deprecated: use reference_governor-msg:polyTraj instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <polyTraj>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader reference_governor-msg:header-val is deprecated.  Use reference_governor-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'start_planning_time-val :lambda-list '(m))
(cl:defmethod start_planning_time-val ((m <polyTraj>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader reference_governor-msg:start_planning_time-val is deprecated.  Use reference_governor-msg:start_planning_time instead.")
  (start_planning_time m))

(cl:ensure-generic-function 'poly_coefs-val :lambda-list '(m))
(cl:defmethod poly_coefs-val ((m <polyTraj>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader reference_governor-msg:poly_coefs-val is deprecated.  Use reference_governor-msg:poly_coefs instead.")
  (poly_coefs m))

(cl:ensure-generic-function 'poly_duration-val :lambda-list '(m))
(cl:defmethod poly_duration-val ((m <polyTraj>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader reference_governor-msg:poly_duration-val is deprecated.  Use reference_governor-msg:poly_duration instead.")
  (poly_duration m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <polyTraj>) ostream)
  "Serializes a message object of type '<polyTraj>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'start_planning_time) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'poly_coefs) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'poly_duration) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <polyTraj>) istream)
  "Deserializes a message object of type '<polyTraj>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'start_planning_time) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'poly_coefs) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'poly_duration) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<polyTraj>)))
  "Returns string type for a message object of type '<polyTraj>"
  "reference_governor/polyTraj")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'polyTraj)))
  "Returns string type for a message object of type 'polyTraj"
  "reference_governor/polyTraj")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<polyTraj>)))
  "Returns md5sum for a message object of type '<polyTraj>"
  "71bc53ae599621f980c4732c605466da")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'polyTraj)))
  "Returns md5sum for a message object of type 'polyTraj"
  "71bc53ae599621f980c4732c605466da")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<polyTraj>)))
  "Returns full string definition for message of type '<polyTraj>"
  (cl:format cl:nil "std_msgs/Header header~%std_msgs/Time start_planning_time~%std_msgs/Float64MultiArray poly_coefs~%std_msgs/Float64 poly_duration~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: std_msgs/Time~%time data~%~%================================================================================~%MSG: std_msgs/Float64MultiArray~%# Please look at the MultiArrayLayout message definition for~%# documentation on all multiarrays.~%~%MultiArrayLayout  layout        # specification of data layout~%float64[]         data          # array of data~%~%~%================================================================================~%MSG: std_msgs/MultiArrayLayout~%# The multiarray declares a generic multi-dimensional array of a~%# particular data type.  Dimensions are ordered from outer most~%# to inner most.~%~%MultiArrayDimension[] dim # Array of dimension properties~%uint32 data_offset        # padding elements at front of data~%~%# Accessors should ALWAYS be written in terms of dimension stride~%# and specified outer-most dimension first.~%# ~%# multiarray(i,j,k) = data[data_offset + dim_stride[1]*i + dim_stride[2]*j + k]~%#~%# A standard, 3-channel 640x480 image with interleaved color channels~%# would be specified as:~%#~%# dim[0].label  = \"height\"~%# dim[0].size   = 480~%# dim[0].stride = 3*640*480 = 921600  (note dim[0] stride is just size of image)~%# dim[1].label  = \"width\"~%# dim[1].size   = 640~%# dim[1].stride = 3*640 = 1920~%# dim[2].label  = \"channel\"~%# dim[2].size   = 3~%# dim[2].stride = 3~%#~%# multiarray(i,j,k) refers to the ith row, jth column, and kth channel.~%~%================================================================================~%MSG: std_msgs/MultiArrayDimension~%string label   # label of given dimension~%uint32 size    # size of given dimension (in type units)~%uint32 stride  # stride of given dimension~%================================================================================~%MSG: std_msgs/Float64~%float64 data~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'polyTraj)))
  "Returns full string definition for message of type 'polyTraj"
  (cl:format cl:nil "std_msgs/Header header~%std_msgs/Time start_planning_time~%std_msgs/Float64MultiArray poly_coefs~%std_msgs/Float64 poly_duration~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: std_msgs/Time~%time data~%~%================================================================================~%MSG: std_msgs/Float64MultiArray~%# Please look at the MultiArrayLayout message definition for~%# documentation on all multiarrays.~%~%MultiArrayLayout  layout        # specification of data layout~%float64[]         data          # array of data~%~%~%================================================================================~%MSG: std_msgs/MultiArrayLayout~%# The multiarray declares a generic multi-dimensional array of a~%# particular data type.  Dimensions are ordered from outer most~%# to inner most.~%~%MultiArrayDimension[] dim # Array of dimension properties~%uint32 data_offset        # padding elements at front of data~%~%# Accessors should ALWAYS be written in terms of dimension stride~%# and specified outer-most dimension first.~%# ~%# multiarray(i,j,k) = data[data_offset + dim_stride[1]*i + dim_stride[2]*j + k]~%#~%# A standard, 3-channel 640x480 image with interleaved color channels~%# would be specified as:~%#~%# dim[0].label  = \"height\"~%# dim[0].size   = 480~%# dim[0].stride = 3*640*480 = 921600  (note dim[0] stride is just size of image)~%# dim[1].label  = \"width\"~%# dim[1].size   = 640~%# dim[1].stride = 3*640 = 1920~%# dim[2].label  = \"channel\"~%# dim[2].size   = 3~%# dim[2].stride = 3~%#~%# multiarray(i,j,k) refers to the ith row, jth column, and kth channel.~%~%================================================================================~%MSG: std_msgs/MultiArrayDimension~%string label   # label of given dimension~%uint32 size    # size of given dimension (in type units)~%uint32 stride  # stride of given dimension~%================================================================================~%MSG: std_msgs/Float64~%float64 data~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <polyTraj>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'start_planning_time))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'poly_coefs))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'poly_duration))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <polyTraj>))
  "Converts a ROS message object to a list"
  (cl:list 'polyTraj
    (cl:cons ':header (header msg))
    (cl:cons ':start_planning_time (start_planning_time msg))
    (cl:cons ':poly_coefs (poly_coefs msg))
    (cl:cons ':poly_duration (poly_duration msg))
))
