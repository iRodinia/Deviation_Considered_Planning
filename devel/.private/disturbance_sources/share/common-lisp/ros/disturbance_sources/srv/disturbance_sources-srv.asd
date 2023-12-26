
(cl:in-package :asdf)

(defsystem "disturbance_sources-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "DisturbRatio" :depends-on ("_package_DisturbRatio"))
    (:file "_package_DisturbRatio" :depends-on ("_package"))
  ))