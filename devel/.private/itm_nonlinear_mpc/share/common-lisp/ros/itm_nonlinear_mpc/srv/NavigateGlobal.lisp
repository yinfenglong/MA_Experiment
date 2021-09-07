; Auto-generated. Do not edit!


(cl:in-package itm_nonlinear_mpc-srv)


;//! \htmlinclude NavigateGlobal-request.msg.html

(cl:defclass <NavigateGlobal-request> (roslisp-msg-protocol:ros-message)
  ((lat
    :reader lat
    :initarg :lat
    :type cl:float
    :initform 0.0)
   (lon
    :reader lon
    :initarg :lon
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

(cl:defclass NavigateGlobal-request (<NavigateGlobal-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <NavigateGlobal-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'NavigateGlobal-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name itm_nonlinear_mpc-srv:<NavigateGlobal-request> is deprecated: use itm_nonlinear_mpc-srv:NavigateGlobal-request instead.")))

(cl:ensure-generic-function 'lat-val :lambda-list '(m))
(cl:defmethod lat-val ((m <NavigateGlobal-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader itm_nonlinear_mpc-srv:lat-val is deprecated.  Use itm_nonlinear_mpc-srv:lat instead.")
  (lat m))

(cl:ensure-generic-function 'lon-val :lambda-list '(m))
(cl:defmethod lon-val ((m <NavigateGlobal-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader itm_nonlinear_mpc-srv:lon-val is deprecated.  Use itm_nonlinear_mpc-srv:lon instead.")
  (lon m))

(cl:ensure-generic-function 'z-val :lambda-list '(m))
(cl:defmethod z-val ((m <NavigateGlobal-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader itm_nonlinear_mpc-srv:z-val is deprecated.  Use itm_nonlinear_mpc-srv:z instead.")
  (z m))

(cl:ensure-generic-function 'yaw-val :lambda-list '(m))
(cl:defmethod yaw-val ((m <NavigateGlobal-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader itm_nonlinear_mpc-srv:yaw-val is deprecated.  Use itm_nonlinear_mpc-srv:yaw instead.")
  (yaw m))

(cl:ensure-generic-function 'yaw_rate-val :lambda-list '(m))
(cl:defmethod yaw_rate-val ((m <NavigateGlobal-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader itm_nonlinear_mpc-srv:yaw_rate-val is deprecated.  Use itm_nonlinear_mpc-srv:yaw_rate instead.")
  (yaw_rate m))

(cl:ensure-generic-function 'speed-val :lambda-list '(m))
(cl:defmethod speed-val ((m <NavigateGlobal-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader itm_nonlinear_mpc-srv:speed-val is deprecated.  Use itm_nonlinear_mpc-srv:speed instead.")
  (speed m))

(cl:ensure-generic-function 'frame_id-val :lambda-list '(m))
(cl:defmethod frame_id-val ((m <NavigateGlobal-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader itm_nonlinear_mpc-srv:frame_id-val is deprecated.  Use itm_nonlinear_mpc-srv:frame_id instead.")
  (frame_id m))

(cl:ensure-generic-function 'auto_arm-val :lambda-list '(m))
(cl:defmethod auto_arm-val ((m <NavigateGlobal-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader itm_nonlinear_mpc-srv:auto_arm-val is deprecated.  Use itm_nonlinear_mpc-srv:auto_arm instead.")
  (auto_arm m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <NavigateGlobal-request>) ostream)
  "Serializes a message object of type '<NavigateGlobal-request>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'lat))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'lon))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <NavigateGlobal-request>) istream)
  "Deserializes a message object of type '<NavigateGlobal-request>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'lat) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'lon) (roslisp-utils:decode-double-float-bits bits)))
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<NavigateGlobal-request>)))
  "Returns string type for a service object of type '<NavigateGlobal-request>"
  "itm_nonlinear_mpc/NavigateGlobalRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'NavigateGlobal-request)))
  "Returns string type for a service object of type 'NavigateGlobal-request"
  "itm_nonlinear_mpc/NavigateGlobalRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<NavigateGlobal-request>)))
  "Returns md5sum for a message object of type '<NavigateGlobal-request>"
  "fd9375a2b183ad5a1c80cf671893464b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'NavigateGlobal-request)))
  "Returns md5sum for a message object of type 'NavigateGlobal-request"
  "fd9375a2b183ad5a1c80cf671893464b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<NavigateGlobal-request>)))
  "Returns full string definition for message of type '<NavigateGlobal-request>"
  (cl:format cl:nil "float64 lat~%float64 lon~%float32 z~%float32 yaw~%float32 yaw_rate~%float32 speed~%string frame_id~%bool auto_arm~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'NavigateGlobal-request)))
  "Returns full string definition for message of type 'NavigateGlobal-request"
  (cl:format cl:nil "float64 lat~%float64 lon~%float32 z~%float32 yaw~%float32 yaw_rate~%float32 speed~%string frame_id~%bool auto_arm~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <NavigateGlobal-request>))
  (cl:+ 0
     8
     8
     4
     4
     4
     4
     4 (cl:length (cl:slot-value msg 'frame_id))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <NavigateGlobal-request>))
  "Converts a ROS message object to a list"
  (cl:list 'NavigateGlobal-request
    (cl:cons ':lat (lat msg))
    (cl:cons ':lon (lon msg))
    (cl:cons ':z (z msg))
    (cl:cons ':yaw (yaw msg))
    (cl:cons ':yaw_rate (yaw_rate msg))
    (cl:cons ':speed (speed msg))
    (cl:cons ':frame_id (frame_id msg))
    (cl:cons ':auto_arm (auto_arm msg))
))
;//! \htmlinclude NavigateGlobal-response.msg.html

(cl:defclass <NavigateGlobal-response> (roslisp-msg-protocol:ros-message)
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

(cl:defclass NavigateGlobal-response (<NavigateGlobal-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <NavigateGlobal-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'NavigateGlobal-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name itm_nonlinear_mpc-srv:<NavigateGlobal-response> is deprecated: use itm_nonlinear_mpc-srv:NavigateGlobal-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <NavigateGlobal-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader itm_nonlinear_mpc-srv:success-val is deprecated.  Use itm_nonlinear_mpc-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'message-val :lambda-list '(m))
(cl:defmethod message-val ((m <NavigateGlobal-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader itm_nonlinear_mpc-srv:message-val is deprecated.  Use itm_nonlinear_mpc-srv:message instead.")
  (message m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <NavigateGlobal-response>) ostream)
  "Serializes a message object of type '<NavigateGlobal-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'message))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'message))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <NavigateGlobal-response>) istream)
  "Deserializes a message object of type '<NavigateGlobal-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<NavigateGlobal-response>)))
  "Returns string type for a service object of type '<NavigateGlobal-response>"
  "itm_nonlinear_mpc/NavigateGlobalResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'NavigateGlobal-response)))
  "Returns string type for a service object of type 'NavigateGlobal-response"
  "itm_nonlinear_mpc/NavigateGlobalResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<NavigateGlobal-response>)))
  "Returns md5sum for a message object of type '<NavigateGlobal-response>"
  "fd9375a2b183ad5a1c80cf671893464b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'NavigateGlobal-response)))
  "Returns md5sum for a message object of type 'NavigateGlobal-response"
  "fd9375a2b183ad5a1c80cf671893464b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<NavigateGlobal-response>)))
  "Returns full string definition for message of type '<NavigateGlobal-response>"
  (cl:format cl:nil "bool success~%string message~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'NavigateGlobal-response)))
  "Returns full string definition for message of type 'NavigateGlobal-response"
  (cl:format cl:nil "bool success~%string message~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <NavigateGlobal-response>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'message))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <NavigateGlobal-response>))
  "Converts a ROS message object to a list"
  (cl:list 'NavigateGlobal-response
    (cl:cons ':success (success msg))
    (cl:cons ':message (message msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'NavigateGlobal)))
  'NavigateGlobal-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'NavigateGlobal)))
  'NavigateGlobal-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'NavigateGlobal)))
  "Returns string type for a service object of type '<NavigateGlobal>"
  "itm_nonlinear_mpc/NavigateGlobal")