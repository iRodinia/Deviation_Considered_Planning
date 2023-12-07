
(cl:in-package :asdf)

(defsystem "reference_governor-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "polyTraj" :depends-on ("_package_polyTraj"))
    (:file "_package_polyTraj" :depends-on ("_package"))
  ))