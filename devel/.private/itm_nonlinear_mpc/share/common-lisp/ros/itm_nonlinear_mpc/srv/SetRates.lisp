; Auto-generated. Do not edit!


(cl:in-package itm_nonlinear_mpc-srv)


;//! \htmlinclude SetRates-request.msg.html

(cl:defclass <SetRates-request> (roslisp-msg-protocol:ros-message)
  ((pitch_rate
    :reader pitch_rate
    :initarg :pitch_rate
    :type cl:float
    :initform 0.0)
   (roll_rate
    :reader roll_rate
    :initarg :roll_rate
    :type cl:float
    :initform 0.0)
   (yaw_rate
    :reader yaw_rate
    :initarg :yaw_rate
    :type cl:float
    :initform 0.0)
   (thrust
    :reader thrust
    :initarg :thrust
    :type cl:float
    :initform 0.0)
   (auto_arm
    :reader auto_arm
    :initarg :auto_arm
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass SetRates-request (<SetRates-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetRates-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetRates-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name itm_nonlinear_mpc-srv:<SetRates-request> is deprecated: use itm_nonlinear_mpc-srv:SetRates-request instead.")))

(cl:ensure-generic-function 'pitch_rate-val :lambda-list '(m))
(cl:defmethod pitch_rate-val ((m <SetRates-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader itm_nonlinear_mpc-srv:pitch_rate-val is deprecated.  Use itm_nonlinear_mpc-srv:pitch_rate instead.")
  (pitch_rate m))

(cl:ensure-generic-function 'roll_rate-val :lambda-list '(m))
(cl:defmethod roll_rate-val ((m <SetRates-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader itm_nonlinear_mpc-srv:roll_rate-val is deprecated.  Use itm_nonlinear_mpc-srv:roll_rate instead.")
  (roll_rate m))

(cl:ensure-generic-function 'yaw_rate-val :lambda-list '(m))
(cl:defmethod yaw_rate-val ((m <SetRates-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader itm_nonlinear_mpc-srv:yaw_rate-val is deprecated.  Use itm_nonlinear_mpc-srv:yaw_rate instead.")
  (yaw_rate m))

(cl:ensure-generic-function 'thrust-val :lambda-list '(m))
(cl:defmethod thrust-val ((m <SetRates-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader itm_nonlinear_mpc-srv:thrust-val is deprecated.  Use itm_nonlinear_mpc-srv:thrust instead.")
  (thrust m))

(cl:ensure-generic-function 'auto_arm-val :lambda-list '(m))
(cl:defmethod auto_arm-val ((m <SetRates-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader itm_nonlinear_mpc-srv:auto_arm-val is deprecated.  Use itm_nonlinear_mpc-srv:auto_arm instead.")
  (auto_arm m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetRates-request>) ostream)
  "Serializes a message object of type '<SetRates-request>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'pitch_rate))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'roll_rate))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'yaw_rate))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'thrust))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'auto_arm) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetRates-request>) istream)
  "Deserializes a message object of type '<SetRates-request>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'pitch_rate) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'roll_rate) (roslisp-utils:decode-single-float-bits bits)))
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
    (cl:setf (cl:slot-value msg 'thrust) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:slot-value msg 'auto_arm) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetRates-request>)))
  "Returns string type for a service object of type '<SetRates-request>"
  "itm_nonlinear_mpc/SetRatesRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetRates-request)))
  "Returns string type for a service object of type 'SetRates-request"
  "itm_nonlinear_mpc/SetRatesRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetRates-request>)))
  "Returns md5sum for a message object of type '<SetRates-request>"
  "a9cc2408dc007c6dd1f503c73d267539")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetRates-request)))
  "Returns md5sum for a message object of type 'SetRates-request"
  "a9cc2408dc007c6dd1f503c73d267539")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetRates-request>)))
  "Returns full string definition for message of type '<SetRates-request>"
  (cl:format cl:nil "float32 pitch_rate~%float32 roll_rate~%float32 yaw_rate~%float32 thrust~%bool auto_arm~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetRates-request)))
  "Returns full string definition for message of type 'SetRates-request"
  (cl:format cl:nil "float32 pitch_rate~%float32 roll_rate~%float32 yaw_rate~%float32 thrust~%bool auto_arm~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetRates-request>))
  (cl:+ 0
     4
     4
     4
     4
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetRates-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetRates-request
    (cl:cons ':pitch_rate (pitch_rate msg))
    (cl:cons ':roll_rate (roll_rate msg))
    (cl:cons ':yaw_rate (yaw_rate msg))
    (cl:cons ':thrust (thrust msg))
    (cl:cons ':auto_arm (auto_arm msg))
))
;//! \htmlinclude SetRates-response.msg.html

(cl:defclass <SetRates-response> (roslisp-msg-protocol:ros-message)
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

(cl:defclass SetRates-response (<SetRates-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetRates-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetRates-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name itm_nonlinear_mpc-srv:<SetRates-response> is deprecated: use itm_nonlinear_mpc-srv:SetRates-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <SetRates-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader itm_nonlinear_mpc-srv:success-val is deprecated.  Use itm_nonlinear_mpc-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'message-val :lambda-list '(m))
(cl:defmethod message-val ((m <SetRates-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader itm_nonlinear_mpc-srv:message-val is deprecated.  Use itm_nonlinear_mpc-srv:message instead.")
  (message m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetRates-response>) ostream)
  "Serializes a message object of type '<SetRates-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'message))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'message))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetRates-response>) istream)
  "Deserializes a message object of type '<SetRates-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetRates-response>)))
  "Returns string type for a service object of type '<SetRates-response>"
  "itm_nonlinear_mpc/SetRatesResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetRates-response)))
  "Returns string type for a service object of type 'SetRates-response"
  "itm_nonlinear_mpc/SetRatesResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetRates-response>)))
  "Returns md5sum for a message object of type '<SetRates-response>"
  "a9cc2408dc007c6dd1f503c73d267539")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetRates-response)))
  "Returns md5sum for a message object of type 'SetRates-response"
  "a9cc2408dc007c6dd1f503c73d267539")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetRates-response>)))
  "Returns full string definition for message of type '<SetRates-response>"
  (cl:format cl:nil "bool success~%string message~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetRates-response)))
  "Returns full string definition for message of type 'SetRates-response"
  (cl:format cl:nil "bool success~%string message~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetRates-response>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'message))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetRates-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetRates-response
    (cl:cons ':success (success msg))
    (cl:cons ':message (message msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetRates)))
  'SetRates-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetRates)))
  'SetRates-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetRates)))
  "Returns string type for a service object of type '<SetRates>"
  "itm_nonlinear_mpc/SetRates")