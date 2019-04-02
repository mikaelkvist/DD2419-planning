;; Auto-generated. Do not edit!


(when (boundp 'planning::PoseWithUncertainties)
  (if (not (find-package "PLANNING"))
    (make-package "PLANNING"))
  (shadow 'PoseWithUncertainties (find-package "PLANNING")))
(unless (find-package "PLANNING::POSEWITHUNCERTAINTIES")
  (make-package "PLANNING::POSEWITHUNCERTAINTIES"))

(in-package "ROS")
;;//! \htmlinclude PoseWithUncertainties.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass planning::PoseWithUncertainties
  :super ros::object
  :slots (_position _uncertainties ))

(defmethod planning::PoseWithUncertainties
  (:init
   (&key
    ((:position __position) (instance geometry_msgs::Quaternion :init))
    ((:uncertainties __uncertainties) (instance geometry_msgs::Quaternion :init))
    )
   (send-super :init)
   (setq _position __position)
   (setq _uncertainties __uncertainties)
   self)
  (:position
   (&rest __position)
   (if (keywordp (car __position))
       (send* _position __position)
     (progn
       (if __position (setq _position (car __position)))
       _position)))
  (:uncertainties
   (&rest __uncertainties)
   (if (keywordp (car __uncertainties))
       (send* _uncertainties __uncertainties)
     (progn
       (if __uncertainties (setq _uncertainties (car __uncertainties)))
       _uncertainties)))
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/Quaternion _position
    (send _position :serialization-length)
    ;; geometry_msgs/Quaternion _uncertainties
    (send _uncertainties :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/Quaternion _position
       (send _position :serialize s)
     ;; geometry_msgs/Quaternion _uncertainties
       (send _uncertainties :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/Quaternion _position
     (send _position :deserialize buf ptr-) (incf ptr- (send _position :serialization-length))
   ;; geometry_msgs/Quaternion _uncertainties
     (send _uncertainties :deserialize buf ptr-) (incf ptr- (send _uncertainties :serialization-length))
   ;;
   self)
  )

(setf (get planning::PoseWithUncertainties :md5sum-) "143b13c3f79c2e2058bebcc4baee982a")
(setf (get planning::PoseWithUncertainties :datatype-) "planning/PoseWithUncertainties")
(setf (get planning::PoseWithUncertainties :definition-)
      "geometry_msgs/Quaternion position
geometry_msgs/Quaternion uncertainties

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

")



(provide :planning/PoseWithUncertainties "143b13c3f79c2e2058bebcc4baee982a")


