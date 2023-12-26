; Auto-generated. Do not edit!


(cl:in-package disturbance_sources-srv)


;//! \htmlinclude DisturbRatio-request.msg.html

(cl:defclass <DisturbRatio-request> (roslisp-msg-protocol:ros-message)
  ((pos_x
    :reader pos_x
    :initarg :pos_x
    :type cl:float
    :initform 0.0)
   (pos_y
    :reader pos_y
    :initarg :pos_y
    :type cl:float
    :initform 0.0)
   (pos_z
    :reader pos_z
    :initarg :pos_z
    :type cl:float
    :initform 0.0))
)

(cl:defclass DisturbRatio-request (<DisturbRatio-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DisturbRatio-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DisturbRatio-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name disturbance_sources-srv:<DisturbRatio-request> is deprecated: use disturbance_sources-srv:DisturbRatio-request instead.")))

(cl:ensure-generic-function 'pos_x-val :lambda-list '(m))
(cl:defmethod pos_x-val ((m <DisturbRatio-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader disturbance_sources-srv:pos_x-val is deprecated.  Use disturbance_sources-srv:pos_x instead.")
  (pos_x m))

(cl:ensure-generic-function 'pos_y-val :lambda-list '(m))
(cl:defmethod pos_y-val ((m <DisturbRatio-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader disturbance_sources-srv:pos_y-val is deprecated.  Use disturbance_sources-srv:pos_y instead.")
  (pos_y m))

(cl:ensure-generic-function 'pos_z-val :lambda-list '(m))
(cl:defmethod pos_z-val ((m <DisturbRatio-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader disturbance_sources-srv:pos_z-val is deprecated.  Use disturbance_sources-srv:pos_z instead.")
  (pos_z m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DisturbRatio-request>) ostream)
  "Serializes a message object of type '<DisturbRatio-request>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'pos_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'pos_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'pos_z))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DisturbRatio-request>) istream)
  "Deserializes a message object of type '<DisturbRatio-request>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'pos_x) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'pos_y) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'pos_z) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DisturbRatio-request>)))
  "Returns string type for a service object of type '<DisturbRatio-request>"
  "disturbance_sources/DisturbRatioRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DisturbRatio-request)))
  "Returns string type for a service object of type 'DisturbRatio-request"
  "disturbance_sources/DisturbRatioRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DisturbRatio-request>)))
  "Returns md5sum for a message object of type '<DisturbRatio-request>"
  "357bf3ef38c1e016d0352525c776ffa2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DisturbRatio-request)))
  "Returns md5sum for a message object of type 'DisturbRatio-request"
  "357bf3ef38c1e016d0352525c776ffa2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DisturbRatio-request>)))
  "Returns full string definition for message of type '<DisturbRatio-request>"
  (cl:format cl:nil "float64 pos_x~%float64 pos_y~%float64 pos_z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DisturbRatio-request)))
  "Returns full string definition for message of type 'DisturbRatio-request"
  (cl:format cl:nil "float64 pos_x~%float64 pos_y~%float64 pos_z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DisturbRatio-request>))
  (cl:+ 0
     8
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DisturbRatio-request>))
  "Converts a ROS message object to a list"
  (cl:list 'DisturbRatio-request
    (cl:cons ':pos_x (pos_x msg))
    (cl:cons ':pos_y (pos_y msg))
    (cl:cons ':pos_z (pos_z msg))
))
;//! \htmlinclude DisturbRatio-response.msg.html

(cl:defclass <DisturbRatio-response> (roslisp-msg-protocol:ros-message)
  ((ratio
    :reader ratio
    :initarg :ratio
    :type cl:float
    :initform 0.0))
)

(cl:defclass DisturbRatio-response (<DisturbRatio-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DisturbRatio-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DisturbRatio-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name disturbance_sources-srv:<DisturbRatio-response> is deprecated: use disturbance_sources-srv:DisturbRatio-response instead.")))

(cl:ensure-generic-function 'ratio-val :lambda-list '(m))
(cl:defmethod ratio-val ((m <DisturbRatio-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader disturbance_sources-srv:ratio-val is deprecated.  Use disturbance_sources-srv:ratio instead.")
  (ratio m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DisturbRatio-response>) ostream)
  "Serializes a message object of type '<DisturbRatio-response>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'ratio))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DisturbRatio-response>) istream)
  "Deserializes a message object of type '<DisturbRatio-response>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'ratio) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DisturbRatio-response>)))
  "Returns string type for a service object of type '<DisturbRatio-response>"
  "disturbance_sources/DisturbRatioResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DisturbRatio-response)))
  "Returns string type for a service object of type 'DisturbRatio-response"
  "disturbance_sources/DisturbRatioResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DisturbRatio-response>)))
  "Returns md5sum for a message object of type '<DisturbRatio-response>"
  "357bf3ef38c1e016d0352525c776ffa2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DisturbRatio-response)))
  "Returns md5sum for a message object of type 'DisturbRatio-response"
  "357bf3ef38c1e016d0352525c776ffa2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DisturbRatio-response>)))
  "Returns full string definition for message of type '<DisturbRatio-response>"
  (cl:format cl:nil "float64 ratio~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DisturbRatio-response)))
  "Returns full string definition for message of type 'DisturbRatio-response"
  (cl:format cl:nil "float64 ratio~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DisturbRatio-response>))
  (cl:+ 0
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DisturbRatio-response>))
  "Converts a ROS message object to a list"
  (cl:list 'DisturbRatio-response
    (cl:cons ':ratio (ratio msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'DisturbRatio)))
  'DisturbRatio-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'DisturbRatio)))
  'DisturbRatio-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DisturbRatio)))
  "Returns string type for a service object of type '<DisturbRatio>"
  "disturbance_sources/DisturbRatio")