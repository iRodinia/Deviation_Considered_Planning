;; Auto-generated. Do not edit!


(when (boundp 'disturbance_sources::DisturbRatio)
  (if (not (find-package "DISTURBANCE_SOURCES"))
    (make-package "DISTURBANCE_SOURCES"))
  (shadow 'DisturbRatio (find-package "DISTURBANCE_SOURCES")))
(unless (find-package "DISTURBANCE_SOURCES::DISTURBRATIO")
  (make-package "DISTURBANCE_SOURCES::DISTURBRATIO"))
(unless (find-package "DISTURBANCE_SOURCES::DISTURBRATIOREQUEST")
  (make-package "DISTURBANCE_SOURCES::DISTURBRATIOREQUEST"))
(unless (find-package "DISTURBANCE_SOURCES::DISTURBRATIORESPONSE")
  (make-package "DISTURBANCE_SOURCES::DISTURBRATIORESPONSE"))

(in-package "ROS")





(defclass disturbance_sources::DisturbRatioRequest
  :super ros::object
  :slots (_pos_x _pos_y _pos_z ))

(defmethod disturbance_sources::DisturbRatioRequest
  (:init
   (&key
    ((:pos_x __pos_x) 0.0)
    ((:pos_y __pos_y) 0.0)
    ((:pos_z __pos_z) 0.0)
    )
   (send-super :init)
   (setq _pos_x (float __pos_x))
   (setq _pos_y (float __pos_y))
   (setq _pos_z (float __pos_z))
   self)
  (:pos_x
   (&optional __pos_x)
   (if __pos_x (setq _pos_x __pos_x)) _pos_x)
  (:pos_y
   (&optional __pos_y)
   (if __pos_y (setq _pos_y __pos_y)) _pos_y)
  (:pos_z
   (&optional __pos_z)
   (if __pos_z (setq _pos_z __pos_z)) _pos_z)
  (:serialization-length
   ()
   (+
    ;; float64 _pos_x
    8
    ;; float64 _pos_y
    8
    ;; float64 _pos_z
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _pos_x
       (sys::poke _pos_x (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _pos_y
       (sys::poke _pos_y (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _pos_z
       (sys::poke _pos_z (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _pos_x
     (setq _pos_x (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _pos_y
     (setq _pos_y (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _pos_z
     (setq _pos_z (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(defclass disturbance_sources::DisturbRatioResponse
  :super ros::object
  :slots (_ratio ))

(defmethod disturbance_sources::DisturbRatioResponse
  (:init
   (&key
    ((:ratio __ratio) 0.0)
    )
   (send-super :init)
   (setq _ratio (float __ratio))
   self)
  (:ratio
   (&optional __ratio)
   (if __ratio (setq _ratio __ratio)) _ratio)
  (:serialization-length
   ()
   (+
    ;; float64 _ratio
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _ratio
       (sys::poke _ratio (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _ratio
     (setq _ratio (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(defclass disturbance_sources::DisturbRatio
  :super ros::object
  :slots ())

(setf (get disturbance_sources::DisturbRatio :md5sum-) "357bf3ef38c1e016d0352525c776ffa2")
(setf (get disturbance_sources::DisturbRatio :datatype-) "disturbance_sources/DisturbRatio")
(setf (get disturbance_sources::DisturbRatio :request) disturbance_sources::DisturbRatioRequest)
(setf (get disturbance_sources::DisturbRatio :response) disturbance_sources::DisturbRatioResponse)

(defmethod disturbance_sources::DisturbRatioRequest
  (:response () (instance disturbance_sources::DisturbRatioResponse :init)))

(setf (get disturbance_sources::DisturbRatioRequest :md5sum-) "357bf3ef38c1e016d0352525c776ffa2")
(setf (get disturbance_sources::DisturbRatioRequest :datatype-) "disturbance_sources/DisturbRatioRequest")
(setf (get disturbance_sources::DisturbRatioRequest :definition-)
      "float64 pos_x
float64 pos_y
float64 pos_z
---
float64 ratio
")

(setf (get disturbance_sources::DisturbRatioResponse :md5sum-) "357bf3ef38c1e016d0352525c776ffa2")
(setf (get disturbance_sources::DisturbRatioResponse :datatype-) "disturbance_sources/DisturbRatioResponse")
(setf (get disturbance_sources::DisturbRatioResponse :definition-)
      "float64 pos_x
float64 pos_y
float64 pos_z
---
float64 ratio
")



(provide :disturbance_sources/DisturbRatio "357bf3ef38c1e016d0352525c776ffa2")


