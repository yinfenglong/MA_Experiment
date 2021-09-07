; Auto-generated. Do not edit!


(cl:in-package itm_mav_srvs-srv)


;//! \htmlinclude mpc_set_point_pos-request.msg.html

(cl:defclass <mpc_set_point_pos-request> (roslisp-msg-protocol:ros-message)
  ((x
    :reader x
    :initarg :x
    :type cl:float
    :initform 0.0)
   (y
    :reader y
    :initarg :y
    :type cl:float
    :initform 0.0)
   (z
    :reader z
    :initarg :z
    :type cl:float
    :initform 0.0)
   (frame_id
    :reader frame_id
    :initarg :frame_id
    :type cl:string
    :initform "")
   (auto_arm
    :reader auto_arm
    :initarg :auto_arm
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass mpc_set_point_pos-request (<mpc_set_point_pos-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <mpc_set_point_pos-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'mpc_set_point_pos-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name itm_mav_srvs-srv:<mpc_set_point_pos-request> is deprecated: use itm_mav_srvs-srv:mpc_set_point_pos-request instead.")))

(cl:ensure-generic-function 'x-val :lambda-list '(m))
(cl:defmethod x-val ((m <mpc_set_point_pos-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader itm_mav_srvs-srv:x-val is deprecated.  Use itm_mav_srvs-srv:x instead.")
  (x m))

(cl:ensure-generic-function 'y-val :lambda-list '(m))
(cl:defmethod y-val ((m <mpc_set_point_pos-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader itm_mav_srvs-srv:y-val is deprecated.  Use itm_mav_srvs-srv:y instead.")
  (y m))

(cl:ensure-generic-function 'z-val :lambda-list '(m))
(cl:defmethod z-val ((m <mpc_set_point_pos-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader itm_mav_srvs-srv:z-val is deprecated.  Use itm_mav_srvs-srv:z instead.")
  (z m))

(cl:ensure-generic-function 'frame_id-val :lambda-list '(m))
(cl:defmethod frame_id-val ((m <mpc_set_point_pos-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader itm_mav_srvs-srv:frame_id-val is deprecated.  Use itm_mav_srvs-srv:frame_id instead.")
  (frame_id m))

(cl:ensure-generic-function 'auto_arm-val :lambda-list '(m))
(cl:defmethod auto_arm-val ((m <mpc_set_point_pos-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader itm_mav_srvs-srv:auto_arm-val is deprecated.  Use itm_mav_srvs-srv:auto_arm instead.")
  (auto_arm m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <mpc_set_point_pos-request>) ostream)
  "Serializes a message object of type '<mpc_set_point_pos-request>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'z))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'frame_id))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'frame_id))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'auto_arm) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <mpc_set_point_pos-request>) istream)
  "Deserializes a message object of type '<mpc_set_point_pos-request>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'z) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'frame_id) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'frame_id) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:setf (cl:slot-value msg 'auto_arm) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<mpc_set_point_pos-request>)))
  "Returns string type for a service object of type '<mpc_set_point_pos-request>"
  "itm_mav_srvs/mpc_set_point_posRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'mpc_set_point_pos-request)))
  "Returns string type for a service object of type 'mpc_set_point_pos-request"
  "itm_mav_srvs/mpc_set_point_posRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<mpc_set_point_pos-request>)))
  "Returns md5sum for a message object of type '<mpc_set_point_pos-request>"
  "5c072df7d2362d0e0013d773b5d5c303")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'mpc_set_point_pos-request)))
  "Returns md5sum for a message object of type 'mpc_set_point_pos-request"
  "5c072df7d2362d0e0013d773b5d5c303")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<mpc_set_point_pos-request>)))
  "Returns full string definition for message of type '<mpc_set_point_pos-request>"
  (cl:format cl:nil "float32 x~%float32 y~%float32 z~%string frame_id~%bool auto_arm~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'mpc_set_point_pos-request)))
  "Returns full string definition for message of type 'mpc_set_point_pos-request"
  (cl:format cl:nil "float32 x~%float32 y~%float32 z~%string frame_id~%bool auto_arm~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <mpc_set_point_pos-request>))
  (cl:+ 0
     4
     4
     4
     4 (cl:length (cl:slot-value msg 'frame_id))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <mpc_set_point_pos-request>))
  "Converts a ROS message object to a list"
  (cl:list 'mpc_set_point_pos-request
    (cl:cons ':x (x msg))
    (cl:cons ':y (y msg))
    (cl:cons ':z (z msg))
    (cl:cons ':frame_id (frame_id msg))
    (cl:cons ':auto_arm (auto_arm msg))
))
;//! \htmlinclude mpc_set_point_pos-response.msg.html

(cl:defclass <mpc_set_point_pos-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil)
   (message
    :reader message
    :initarg :message
    :type cl:string
    :initform ""))
)

(cl:defclass mpc_set_point_pos-response (<mpc_set_point_pos-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <mpc_set_point_pos-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'mpc_set_point_pos-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name itm_mav_srvs-srv:<mpc_set_point_pos-response> is deprecated: use itm_mav_srvs-srv:mpc_set_point_pos-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <mpc_set_point_pos-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader itm_mav_srvs-srv:success-val is deprecated.  Use itm_mav_srvs-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'message-val :lambda-list '(m))
(cl:defmethod message-val ((m <mpc_set_point_pos-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader itm_mav_srvs-srv:message-val is deprecated.  Use itm_mav_srvs-srv:message instead.")
  (message m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <mpc_set_point_pos-response>) ostream)
  "Serializes a message object of type '<mpc_set_point_pos-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'message))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'message))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <mpc_set_point_pos-response>) istream)
  "Deserializes a message object of type '<mpc_set_point_pos-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'message) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'message) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<mpc_set_point_pos-response>)))
  "Returns string type for a service object of type '<mpc_set_point_pos-response>"
  "itm_mav_srvs/mpc_set_point_posResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'mpc_set_point_pos-response)))
  "Returns string type for a service object of type 'mpc_set_point_pos-response"
  "itm_mav_srvs/mpc_set_point_posResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<mpc_set_point_pos-response>)))
  "Returns md5sum for a message object of type '<mpc_set_point_pos-response>"
  "5c072df7d2362d0e0013d773b5d5c303")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'mpc_set_point_pos-response)))
  "Returns md5sum for a message object of type 'mpc_set_point_pos-response"
  "5c072df7d2362d0e0013d773b5d5c303")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<mpc_set_point_pos-response>)))
  "Returns full string definition for message of type '<mpc_set_point_pos-response>"
  (cl:format cl:nil "bool success~%string message~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'mpc_set_point_pos-response)))
  "Returns full string definition for message of type 'mpc_set_point_pos-response"
  (cl:format cl:nil "bool success~%string message~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <mpc_set_point_pos-response>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'message))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <mpc_set_point_pos-response>))
  "Converts a ROS message object to a list"
  (cl:list 'mpc_set_point_pos-response
    (cl:cons ':success (success msg))
    (cl:cons ':message (message msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'mpc_set_point_pos)))
  'mpc_set_point_pos-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'mpc_set_point_pos)))
  'mpc_set_point_pos-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'mpc_set_point_pos)))
  "Returns string type for a service object of type '<mpc_set_point_pos>"
  "itm_mav_srvs/mpc_set_point_pos")