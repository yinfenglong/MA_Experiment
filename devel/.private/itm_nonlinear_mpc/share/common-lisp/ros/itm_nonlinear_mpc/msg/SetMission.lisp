; Auto-generated. Do not edit!


(cl:in-package itm_nonlinear_mpc-msg)


;//! \htmlinclude SetMission.msg.html

(cl:defclass <SetMission> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (command_idx
    :reader command_idx
    :initarg :command_idx
    :type cl:integer
    :initform 0)
   (mission_mode
    :reader mission_mode
    :initarg :mission_mode
    :type cl:fixnum
    :initform 0))
)

(cl:defclass SetMission (<SetMission>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetMission>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetMission)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name itm_nonlinear_mpc-msg:<SetMission> is deprecated: use itm_nonlinear_mpc-msg:SetMission instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <SetMission>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader itm_nonlinear_mpc-msg:header-val is deprecated.  Use itm_nonlinear_mpc-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'command_idx-val :lambda-list '(m))
(cl:defmethod command_idx-val ((m <SetMission>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader itm_nonlinear_mpc-msg:command_idx-val is deprecated.  Use itm_nonlinear_mpc-msg:command_idx instead.")
  (command_idx m))

(cl:ensure-generic-function 'mission_mode-val :lambda-list '(m))
(cl:defmethod mission_mode-val ((m <SetMission>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader itm_nonlinear_mpc-msg:mission_mode-val is deprecated.  Use itm_nonlinear_mpc-msg:mission_mode instead.")
  (mission_mode m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetMission>) ostream)
  "Serializes a message object of type '<SetMission>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'command_idx)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'command_idx)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'command_idx)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'command_idx)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'mission_mode)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetMission>) istream)
  "Deserializes a message object of type '<SetMission>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'command_idx)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'command_idx)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'command_idx)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'command_idx)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'mission_mode)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetMission>)))
  "Returns string type for a message object of type '<SetMission>"
  "itm_nonlinear_mpc/SetMission")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetMission)))
  "Returns string type for a message object of type 'SetMission"
  "itm_nonlinear_mpc/SetMission")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetMission>)))
  "Returns md5sum for a message object of type '<SetMission>"
  "1631998827fd12d678dc74adb693f0b9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetMission)))
  "Returns md5sum for a message object of type 'SetMission"
  "1631998827fd12d678dc74adb693f0b9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetMission>)))
  "Returns full string definition for message of type '<SetMission>"
  (cl:format cl:nil "std_msgs/Header header~%uint32 command_idx~%uint8 mission_mode~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetMission)))
  "Returns full string definition for message of type 'SetMission"
  (cl:format cl:nil "std_msgs/Header header~%uint32 command_idx~%uint8 mission_mode~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetMission>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetMission>))
  "Converts a ROS message object to a list"
  (cl:list 'SetMission
    (cl:cons ':header (header msg))
    (cl:cons ':command_idx (command_idx msg))
    (cl:cons ':mission_mode (mission_mode msg))
))
