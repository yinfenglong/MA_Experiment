; Auto-generated. Do not edit!


(cl:in-package itm_nonlinear_mpc-msg)


;//! \htmlinclude mpc_command_rpyt.msg.html

(cl:defclass <mpc_command_rpyt> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (roll_ref
    :reader roll_ref
    :initarg :roll_ref
    :type cl:float
    :initform 0.0)
   (pitch_ref
    :reader pitch_ref
    :initarg :pitch_ref
    :type cl:float
    :initform 0.0)
   (yaw_rate_cmd
    :reader yaw_rate_cmd
    :initarg :yaw_rate_cmd
    :type cl:float
    :initform 0.0)
   (thrust_ref
    :reader thrust_ref
    :initarg :thrust_ref
    :type cl:float
    :initform 0.0))
)

(cl:defclass mpc_command_rpyt (<mpc_command_rpyt>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <mpc_command_rpyt>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'mpc_command_rpyt)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name itm_nonlinear_mpc-msg:<mpc_command_rpyt> is deprecated: use itm_nonlinear_mpc-msg:mpc_command_rpyt instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <mpc_command_rpyt>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader itm_nonlinear_mpc-msg:header-val is deprecated.  Use itm_nonlinear_mpc-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'roll_ref-val :lambda-list '(m))
(cl:defmethod roll_ref-val ((m <mpc_command_rpyt>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader itm_nonlinear_mpc-msg:roll_ref-val is deprecated.  Use itm_nonlinear_mpc-msg:roll_ref instead.")
  (roll_ref m))

(cl:ensure-generic-function 'pitch_ref-val :lambda-list '(m))
(cl:defmethod pitch_ref-val ((m <mpc_command_rpyt>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader itm_nonlinear_mpc-msg:pitch_ref-val is deprecated.  Use itm_nonlinear_mpc-msg:pitch_ref instead.")
  (pitch_ref m))

(cl:ensure-generic-function 'yaw_rate_cmd-val :lambda-list '(m))
(cl:defmethod yaw_rate_cmd-val ((m <mpc_command_rpyt>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader itm_nonlinear_mpc-msg:yaw_rate_cmd-val is deprecated.  Use itm_nonlinear_mpc-msg:yaw_rate_cmd instead.")
  (yaw_rate_cmd m))

(cl:ensure-generic-function 'thrust_ref-val :lambda-list '(m))
(cl:defmethod thrust_ref-val ((m <mpc_command_rpyt>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader itm_nonlinear_mpc-msg:thrust_ref-val is deprecated.  Use itm_nonlinear_mpc-msg:thrust_ref instead.")
  (thrust_ref m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <mpc_command_rpyt>) ostream)
  "Serializes a message object of type '<mpc_command_rpyt>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'roll_ref))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'pitch_ref))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'yaw_rate_cmd))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'thrust_ref))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <mpc_command_rpyt>) istream)
  "Deserializes a message object of type '<mpc_command_rpyt>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'roll_ref) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'pitch_ref) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'yaw_rate_cmd) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'thrust_ref) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<mpc_command_rpyt>)))
  "Returns string type for a message object of type '<mpc_command_rpyt>"
  "itm_nonlinear_mpc/mpc_command_rpyt")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'mpc_command_rpyt)))
  "Returns string type for a message object of type 'mpc_command_rpyt"
  "itm_nonlinear_mpc/mpc_command_rpyt")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<mpc_command_rpyt>)))
  "Returns md5sum for a message object of type '<mpc_command_rpyt>"
  "3a1aaed29b0fec0f986f12a3290ec8b8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'mpc_command_rpyt)))
  "Returns md5sum for a message object of type 'mpc_command_rpyt"
  "3a1aaed29b0fec0f986f12a3290ec8b8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<mpc_command_rpyt>)))
  "Returns full string definition for message of type '<mpc_command_rpyt>"
  (cl:format cl:nil "std_msgs/Header header~%float64 roll_ref~%float64 pitch_ref~%float64 yaw_rate_cmd~%float64 thrust_ref~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'mpc_command_rpyt)))
  "Returns full string definition for message of type 'mpc_command_rpyt"
  (cl:format cl:nil "std_msgs/Header header~%float64 roll_ref~%float64 pitch_ref~%float64 yaw_rate_cmd~%float64 thrust_ref~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <mpc_command_rpyt>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     8
     8
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <mpc_command_rpyt>))
  "Converts a ROS message object to a list"
  (cl:list 'mpc_command_rpyt
    (cl:cons ':header (header msg))
    (cl:cons ':roll_ref (roll_ref msg))
    (cl:cons ':pitch_ref (pitch_ref msg))
    (cl:cons ':yaw_rate_cmd (yaw_rate_cmd msg))
    (cl:cons ':thrust_ref (thrust_ref msg))
))
