; Auto-generated. Do not edit!


(cl:in-package planning-msg)


;//! \htmlinclude PoseWithUncertainties.msg.html

(cl:defclass <PoseWithUncertainties> (roslisp-msg-protocol:ros-message)
  ((position
    :reader position
    :initarg :position
    :type geometry_msgs-msg:Quaternion
    :initform (cl:make-instance 'geometry_msgs-msg:Quaternion))
   (uncertainties
    :reader uncertainties
    :initarg :uncertainties
    :type geometry_msgs-msg:Quaternion
    :initform (cl:make-instance 'geometry_msgs-msg:Quaternion)))
)

(cl:defclass PoseWithUncertainties (<PoseWithUncertainties>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PoseWithUncertainties>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PoseWithUncertainties)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name planning-msg:<PoseWithUncertainties> is deprecated: use planning-msg:PoseWithUncertainties instead.")))

(cl:ensure-generic-function 'position-val :lambda-list '(m))
(cl:defmethod position-val ((m <PoseWithUncertainties>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader planning-msg:position-val is deprecated.  Use planning-msg:position instead.")
  (position m))

(cl:ensure-generic-function 'uncertainties-val :lambda-list '(m))
(cl:defmethod uncertainties-val ((m <PoseWithUncertainties>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader planning-msg:uncertainties-val is deprecated.  Use planning-msg:uncertainties instead.")
  (uncertainties m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PoseWithUncertainties>) ostream)
  "Serializes a message object of type '<PoseWithUncertainties>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'position) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'uncertainties) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PoseWithUncertainties>) istream)
  "Deserializes a message object of type '<PoseWithUncertainties>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'position) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'uncertainties) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PoseWithUncertainties>)))
  "Returns string type for a message object of type '<PoseWithUncertainties>"
  "planning/PoseWithUncertainties")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PoseWithUncertainties)))
  "Returns string type for a message object of type 'PoseWithUncertainties"
  "planning/PoseWithUncertainties")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PoseWithUncertainties>)))
  "Returns md5sum for a message object of type '<PoseWithUncertainties>"
  "143b13c3f79c2e2058bebcc4baee982a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PoseWithUncertainties)))
  "Returns md5sum for a message object of type 'PoseWithUncertainties"
  "143b13c3f79c2e2058bebcc4baee982a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PoseWithUncertainties>)))
  "Returns full string definition for message of type '<PoseWithUncertainties>"
  (cl:format cl:nil "geometry_msgs/Quaternion position~%geometry_msgs/Quaternion uncertainties~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PoseWithUncertainties)))
  "Returns full string definition for message of type 'PoseWithUncertainties"
  (cl:format cl:nil "geometry_msgs/Quaternion position~%geometry_msgs/Quaternion uncertainties~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PoseWithUncertainties>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'position))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'uncertainties))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PoseWithUncertainties>))
  "Converts a ROS message object to a list"
  (cl:list 'PoseWithUncertainties
    (cl:cons ':position (position msg))
    (cl:cons ':uncertainties (uncertainties msg))
))
