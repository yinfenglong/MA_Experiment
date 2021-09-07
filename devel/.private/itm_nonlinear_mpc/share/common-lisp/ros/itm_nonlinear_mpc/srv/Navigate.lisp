; Auto-generated. Do not edit!


(cl:in-package itm_nonlinear_mpc-srv)


;//! \htmlinclude Navigate-request.msg.html

(cl:defclass <Navigate-request> (roslisp-msg-protocol:ros-message)
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
   (yaw
    :reader yaw
    :initarg :yaw
    :type cl:float
    :initform 0.0)
   (yaw_rate
    :reader yaw_rate
    :initarg :yaw_rate
    :type cl:float
    :initform 0.0)
   (speed
    :reader speed
    :initarg :speed
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

(cl:defclass Navigate-request (<Navigate-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Navigate-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Navigate-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name itm_nonlinear_mpc-srv:<Navigate-request> is deprecated: use itm_nonlinear_mpc-srv:Navigate-request instead.")))

(cl:ensure-generic-function 'x-val :lambda-list '(m))
(cl:defmethod x-val ((m <Navigate-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader itm_nonlinear_mpc-srv:x-val is deprecated.  Use itm_nonlinear_mpc-srv:x instead.")
  (x m))

(cl:ensure-generic-function 'y-val :lambda-list '(m))
(cl:defmethod y-val ((m <Navigate-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader itm_nonlinear_mpc-srv:y-val is deprecated.  Use itm_nonlinear_mpc-srv:y instead.")
  (y m))

(cl:ensure-generic-function 'z-val :lambda-list '(m))
(cl:defmethod z-val ((m <Navigate-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader itm_nonlinear_mpc-srv:z-val is deprecated.  Use itm_nonlinear_mpc-srv:z instead.")
  (z m))

(cl:ensure-generic-function 'yaw-val :lambda-list '(m))
(cl:defmethod yaw-val ((m <Navigate-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader itm_nonlinear_mpc-srv:yaw-val is deprecated.  Use itm_nonlinear_mpc-srv:yaw instead.")
  (yaw m))

(cl:ensure-generic-function 'yaw_rate-val :lambda-list '(m))
(cl:defmethod yaw_rate-val ((m <Navigate-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader itm_nonlinear_mpc-srv:yaw_rate-val is deprecated.  Use itm_nonlinear_mpc-srv:yaw_rate instead.")
  (yaw_rate m))

(cl:ensure-generic-function 'speed-val :lambda-list '(m))
(cl:defmethod speed-val ((m <Navigate-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader itm_nonlinear_mpc-srv:speed-val is deprecated.  Use itm_nonlinear_mpc-srv:speed instead.")
  (speed m))

(cl:ensure-generic-function 'frame_id-val :lambda-list '(m))
(cl:defmethod frame_id-val ((m <Navigate-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader itm_nonlinear_mpc-srv:frame_id-val is deprecated.  Use itm_nonlinear_mpc-srv:frame_id instead.")
  (frame_id m))

(cl:ensure-generic-function 'auto_arm-val :lambda-list '(m))
(cl:defmethod auto_arm-val ((m <Navigate-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader itm_nonlinear_mpc-srv:auto_arm-val is deprecated.  Use itm_nonlinear_mpc-srv:auto_arm instead.")
  (auto_arm m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Navigate-request>) ostream)
  "Serializes a message object of type '<Navigate-request>"
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
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'yaw))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'yaw_rate))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'speed))))
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Navigate-request>) istream)
  "Deserializes a message object of type '<Navigate-request>"
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
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'yaw) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'yaw_rate) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'speed) (roslisp-utils:decode-single-float-bits bits)))
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Navigate-request>)))
  "Returns string type for a service object of type '<Navigate-request>"
  "itm_nonlinear_mpc/NavigateRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Navigate-request)))
  "Returns string type for a service object of type 'Navigate-request"
  "itm_nonlinear_mpc/NavigateRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Navigate-request>)))
  "Returns md5sum for a message object of type '<Navigate-request>"
  "d27cf8a87020c28b3d14e06881a86c21")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Navigate-request)))
  "Returns md5sum for a message object of type 'Navigate-request"
  "d27cf8a87020c28b3d14e06881a86c21")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Navigate-request>)))
  "Returns full string definition for message of type '<Navigate-request>"
  (cl:format cl:nil "float32 x~%float32 y~%float32 z~%float32 yaw~%float32 yaw_rate~%float32 speed~%string frame_id~%bool auto_arm~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Navigate-request)))
  "Returns full string definition for message of type 'Navigate-request"
  (cl:format cl:nil "float32 x~%float32 y~%float32 z~%float32 yaw~%float32 yaw_rate~%float32 speed~%string frame_id~%bool auto_arm~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Navigate-request>))
  (cl:+ 0
     4
     4
     4
     4
     4
     4
     4 (cl:length (cl:slot-value msg 'frame_id))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Navigate-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Navigate-request
    (cl:cons ':x (x msg))
    (cl:cons ':y (y msg))
    (cl:cons ':z (z msg))
    (cl:cons ':yaw (yaw msg))
    (cl:cons ':yaw_rate (yaw_rate msg))
    (cl:cons ':speed (speed msg))
    (cl:cons ':frame_id (frame_id msg))
    (cl:cons ':auto_arm (auto_arm msg))
))
;//! \htmlinclude Navigate-response.msg.html

(cl:defclass <Navigate-response> (roslisp-msg-protocol:ros-message)
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

(cl:defclass Navigate-response (<Navigate-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Navigate-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Navigate-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name itm_nonlinear_mpc-srv:<Navigate-response> is deprecated: use itm_nonlinear_mpc-srv:Navigate-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <Navigate-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader itm_nonlinear_mpc-srv:success-val is deprecated.  Use itm_nonlinear_mpc-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'message-val :lambda-list '(m))
(cl:defmethod message-val ((m <Navigate-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader itm_nonlinear_mpc-srv:message-val is deprecated.  Use itm_nonlinear_mpc-srv:message instead.")
  (message m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Navigate-response>) ostream)
  "Serializes a message object of type '<Navigate-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'message))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'message))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Navigate-response>) istream)
  "Deserializes a message object of type '<Navigate-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Navigate-response>)))
  "Returns string type for a service object of type '<Navigate-response>"
  "itm_nonlinear_mpc/NavigateResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Navigate-response)))
  "Returns string type for a service object of type 'Navigate-response"
  "itm_nonlinear_mpc/NavigateResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Navigate-response>)))
  "Returns md5sum for a message object of type '<Navigate-response>"
  "d27cf8a87020c28b3d14e06881a86c21")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Navigate-response)))
  "Returns md5sum for a message object of type 'Navigate-response"
  "d27cf8a87020c28b3d14e06881a86c21")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Navigate-response>)))
  "Returns full string definition for message of type '<Navigate-response>"
  (cl:format cl:nil "bool success~%string message~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Navigate-response)))
  "Returns full string definition for message of type 'Navigate-response"
  (cl:format cl:nil "bool success~%string message~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Navigate-response>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'message))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Navigate-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Navigate-response
    (cl:cons ':success (success msg))
    (cl:cons ':message (message msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Navigate)))
  'Navigate-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Navigate)))
  'Navigate-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Navigate)))
  "Returns string type for a service object of type '<Navigate>"
  "itm_nonlinear_mpc/Navigate")