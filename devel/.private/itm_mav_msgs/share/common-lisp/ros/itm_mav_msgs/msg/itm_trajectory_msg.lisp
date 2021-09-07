; Auto-generated. Do not edit!


(cl:in-package itm_mav_msgs-msg)


;//! \htmlinclude itm_trajectory_msg.msg.html

(cl:defclass <itm_trajectory_msg> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (size
    :reader size
    :initarg :size
    :type cl:integer
    :initform 0)
   (traj
    :reader traj
    :initarg :traj
    :type (cl:vector itm_mav_msgs-msg:itm_trajectory_point)
   :initform (cl:make-array 0 :element-type 'itm_mav_msgs-msg:itm_trajectory_point :initial-element (cl:make-instance 'itm_mav_msgs-msg:itm_trajectory_point)))
   (data
    :reader data
    :initarg :data
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass itm_trajectory_msg (<itm_trajectory_msg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <itm_trajectory_msg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'itm_trajectory_msg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name itm_mav_msgs-msg:<itm_trajectory_msg> is deprecated: use itm_mav_msgs-msg:itm_trajectory_msg instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <itm_trajectory_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader itm_mav_msgs-msg:header-val is deprecated.  Use itm_mav_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'size-val :lambda-list '(m))
(cl:defmethod size-val ((m <itm_trajectory_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader itm_mav_msgs-msg:size-val is deprecated.  Use itm_mav_msgs-msg:size instead.")
  (size m))

(cl:ensure-generic-function 'traj-val :lambda-list '(m))
(cl:defmethod traj-val ((m <itm_trajectory_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader itm_mav_msgs-msg:traj-val is deprecated.  Use itm_mav_msgs-msg:traj instead.")
  (traj m))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <itm_trajectory_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader itm_mav_msgs-msg:data-val is deprecated.  Use itm_mav_msgs-msg:data instead.")
  (data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <itm_trajectory_msg>) ostream)
  "Serializes a message object of type '<itm_trajectory_msg>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'size)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'size)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'size)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'size)) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'traj))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'traj))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'data))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'data))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <itm_trajectory_msg>) istream)
  "Deserializes a message object of type '<itm_trajectory_msg>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'size)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'size)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'size)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'size)) (cl:read-byte istream))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'traj) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'traj)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'itm_mav_msgs-msg:itm_trajectory_point))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'data) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'data)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<itm_trajectory_msg>)))
  "Returns string type for a message object of type '<itm_trajectory_msg>"
  "itm_mav_msgs/itm_trajectory_msg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'itm_trajectory_msg)))
  "Returns string type for a message object of type 'itm_trajectory_msg"
  "itm_mav_msgs/itm_trajectory_msg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<itm_trajectory_msg>)))
  "Returns md5sum for a message object of type '<itm_trajectory_msg>"
  "d9bcd1f60582328c13383ece94b6ed5c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'itm_trajectory_msg)))
  "Returns md5sum for a message object of type 'itm_trajectory_msg"
  "d9bcd1f60582328c13383ece94b6ed5c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<itm_trajectory_msg>)))
  "Returns full string definition for message of type '<itm_trajectory_msg>"
  (cl:format cl:nil "Header header~%uint32 size~%itm_mav_msgs/itm_trajectory_point[] traj~%float64[] data~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: itm_mav_msgs/itm_trajectory_point~%float64 x~%float64 y~%float64 z~%float64 vx~%float64 vy~%float64 vz~%float64 roll~%float64 pitch~%float64 yaw~%float64 roll_des~%float64 pitch_des~%float64 yaw_des~%float64 roll_rate_des~%float64 pitch_rate_des~%float64 yaw_rate_des~%float64 thrust_des~%bool input_given~%float64[4] q~%float64[2] cube_x~%float64[2] cube_y~%float64[2] cube_z~%float64[2] cube_yaw~%bool fixed~%bool time_known~%int8 derivative~%int8 segment_index~%float64 time_stamp~%bool quaternion_given~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'itm_trajectory_msg)))
  "Returns full string definition for message of type 'itm_trajectory_msg"
  (cl:format cl:nil "Header header~%uint32 size~%itm_mav_msgs/itm_trajectory_point[] traj~%float64[] data~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: itm_mav_msgs/itm_trajectory_point~%float64 x~%float64 y~%float64 z~%float64 vx~%float64 vy~%float64 vz~%float64 roll~%float64 pitch~%float64 yaw~%float64 roll_des~%float64 pitch_des~%float64 yaw_des~%float64 roll_rate_des~%float64 pitch_rate_des~%float64 yaw_rate_des~%float64 thrust_des~%bool input_given~%float64[4] q~%float64[2] cube_x~%float64[2] cube_y~%float64[2] cube_z~%float64[2] cube_yaw~%bool fixed~%bool time_known~%int8 derivative~%int8 segment_index~%float64 time_stamp~%bool quaternion_given~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <itm_trajectory_msg>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'traj) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'data) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <itm_trajectory_msg>))
  "Converts a ROS message object to a list"
  (cl:list 'itm_trajectory_msg
    (cl:cons ':header (header msg))
    (cl:cons ':size (size msg))
    (cl:cons ':traj (traj msg))
    (cl:cons ':data (data msg))
))
