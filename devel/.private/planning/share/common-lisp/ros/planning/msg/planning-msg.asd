
(cl:in-package :asdf)

(defsystem "planning-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
)
  :components ((:file "_package")
    (:file "PoseWithUncertainties" :depends-on ("_package_PoseWithUncertainties"))
    (:file "_package_PoseWithUncertainties" :depends-on ("_package"))
  ))