; Auto-generated. Do not edit!


(cl:in-package itm_mav_srvs-srv)


;//! \htmlinclude itm_trajectory_srv-request.msg.html

(cl:defclass <itm_trajectory_srv-request> (roslisp-msg-protocol:ros-message)
  ((index
    :reader index
    :initarg :index
    :type cl:integer
    :initform 0)
   (traj
    :reader traj
    :initarg :traj
    :type (cl:vector itm_mav_msgs-msg:itm_trajectory_point)
   :initform (cl:make-array 0 :element-type 'itm_mav_msgs-msg:itm_trajectory_point :initial-element (cl:make-instance 'itm_mav_msgs-msg:itm_trajectory_point))))
)

(cl:defclass itm_trajectory_srv-request (<itm_trajectory_srv-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <itm_trajectory_srv-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'itm_trajectory_srv-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name itm_mav_srvs-srv:<itm_trajectory_srv-request> is deprecated: use itm_mav_srvs-srv:itm_trajectory_srv-request instead.")))

(cl:ensure-generic-function 'index-val :lambda-list '(m))
(cl:defmethod index-val ((m <itm_trajectory_srv-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader itm_mav_srvs-srv:index-val is deprecated.  Use itm_mav_srvs-srv:index instead.")
  (index m))

(cl:ensure-generic-function 'traj-val :lambda-list '(m))
(cl:defmethod traj-val ((m <itm_trajectory_srv-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader itm_mav_srvs-srv:traj-val is deprecated.  Use itm_mav_srvs-srv:traj instead.")
  (traj m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <itm_trajectory_srv-request>) ostream)
  "Serializes a message object of type '<itm_trajectory_srv-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'index)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'index)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'index)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'index)) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'traj))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'traj))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <itm_trajectory_srv-request>) istream)
  "Deserializes a message object of type '<itm_trajectory_srv-request>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'index)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'index)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'index)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'index)) (cl:read-byte istream))
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
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<itm_trajectory_srv-request>)))
  "Returns string type for a service object of type '<itm_trajectory_srv-request>"
  "itm_mav_srvs/itm_trajectory_srvRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'itm_trajectory_srv-request)))
  "Returns string type for a service object of type 'itm_trajectory_srv-request"
  "itm_mav_srvs/itm_trajectory_srvRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<itm_trajectory_srv-request>)))
  "Returns md5sum for a message object of type '<itm_trajectory_srv-request>"
  "bf0f076326f268117df02baf1b905ccb")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'itm_trajectory_srv-request)))
  "Returns md5sum for a message object of type 'itm_trajectory_srv-request"
  "bf0f076326f268117df02baf1b905ccb")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<itm_trajectory_srv-request>)))
  "Returns full string definition for message of type '<itm_trajectory_srv-request>"
  (cl:format cl:nil "uint32 index~%itm_mav_msgs/itm_trajectory_point[] traj~%~%================================================================================~%MSG: itm_mav_msgs/itm_trajectory_point~%float64 x~%float64 y~%float64 z~%float64 vx~%float64 vy~%float64 vz~%float64 roll~%float64 pitch~%float64 yaw~%float64 roll_des~%float64 pitch_des~%float64 yaw_des~%float64 roll_rate_des~%float64 pitch_rate_des~%float64 yaw_rate_des~%float64 thrust_des~%bool input_given~%float64[4] q~%float64[2] cube_x~%float64[2] cube_y~%float64[2] cube_z~%float64[2] cube_yaw~%bool fixed~%bool time_known~%int8 derivative~%int8 segment_index~%float64 time_stamp~%bool quaternion_given~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'itm_trajectory_srv-request)))
  "Returns full string definition for message of type 'itm_trajectory_srv-request"
  (cl:format cl:nil "uint32 index~%itm_mav_msgs/itm_trajectory_point[] traj~%~%================================================================================~%MSG: itm_mav_msgs/itm_trajectory_point~%float64 x~%float64 y~%float64 z~%float64 vx~%float64 vy~%float64 vz~%float64 roll~%float64 pitch~%float64 yaw~%float64 roll_des~%float64 pitch_des~%float64 yaw_des~%float64 roll_rate_des~%float64 pitch_rate_des~%float64 yaw_rate_des~%float64 thrust_des~%bool input_given~%float64[4] q~%float64[2] cube_x~%float64[2] cube_y~%float64[2] cube_z~%float64[2] cube_yaw~%bool fixed~%bool time_known~%int8 derivative~%int8 segment_index~%float64 time_stamp~%bool quaternion_given~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <itm_trajectory_srv-request>))
  (cl:+ 0
     4
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'traj) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <itm_trajectory_srv-request>))
  "Converts a ROS message object to a list"
  (cl:list 'itm_trajectory_srv-request
    (cl:cons ':index (index msg))
    (cl:cons ':traj (traj msg))
))
;//! \htmlinclude itm_trajectory_srv-response.msg.html

(cl:defclass <itm_trajectory_srv-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass itm_trajectory_srv-response (<itm_trajectory_srv-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <itm_trajectory_srv-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'itm_trajectory_srv-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name itm_mav_srvs-srv:<itm_trajectory_srv-response> is deprecated: use itm_mav_srvs-srv:itm_trajectory_srv-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <itm_trajectory_srv-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader itm_mav_srvs-srv:success-val is deprecated.  Use itm_mav_srvs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <itm_trajectory_srv-response>) ostream)
  "Serializes a message object of type '<itm_trajectory_srv-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <itm_trajectory_srv-response>) istream)
  "Deserializes a message object of type '<itm_trajectory_srv-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<itm_trajectory_srv-response>)))
  "Returns string type for a service object of type '<itm_trajectory_srv-response>"
  "itm_mav_srvs/itm_trajectory_srvResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'itm_trajectory_srv-response)))
  "Returns string type for a service object of type 'itm_trajectory_srv-response"
  "itm_mav_srvs/itm_trajectory_srvResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<itm_trajectory_srv-response>)))
  "Returns md5sum for a message object of type '<itm_trajectory_srv-response>"
  "bf0f076326f268117df02baf1b905ccb")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'itm_trajectory_srv-response)))
  "Returns md5sum for a message object of type 'itm_trajectory_srv-response"
  "bf0f076326f268117df02baf1b905ccb")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<itm_trajectory_srv-response>)))
  "Returns full string definition for message of type '<itm_trajectory_srv-response>"
  (cl:format cl:nil "bool success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'itm_trajectory_srv-response)))
  "Returns full string definition for message of type 'itm_trajectory_srv-response"
  (cl:format cl:nil "bool success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <itm_trajectory_srv-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <itm_trajectory_srv-response>))
  "Converts a ROS message object to a list"
  (cl:list 'itm_trajectory_srv-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'itm_trajectory_srv)))
  'itm_trajectory_srv-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'itm_trajectory_srv)))
  'itm_trajectory_srv-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'itm_trajectory_srv)))
  "Returns string type for a service object of type '<itm_trajectory_srv>"
  "itm_mav_srvs/itm_trajectory_srv")