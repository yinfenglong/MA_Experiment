; Auto-generated. Do not edit!


(cl:in-package itm_nonlinear_mpc-srv)


;//! \htmlinclude SetMode-request.msg.html

(cl:defclass <SetMode-request> (roslisp-msg-protocol:ros-message)
  ((mode
    :reader mode
    :initarg :mode
    :type cl:fixnum
    :initform 0))
)

(cl:defclass SetMode-request (<SetMode-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetMode-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetMode-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name itm_nonlinear_mpc-srv:<SetMode-request> is deprecated: use itm_nonlinear_mpc-srv:SetMode-request instead.")))

(cl:ensure-generic-function 'mode-val :lambda-list '(m))
(cl:defmethod mode-val ((m <SetMode-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader itm_nonlinear_mpc-srv:mode-val is deprecated.  Use itm_nonlinear_mpc-srv:mode instead.")
  (mode m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetMode-request>) ostream)
  "Serializes a message object of type '<SetMode-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'mode)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetMode-request>) istream)
  "Deserializes a message object of type '<SetMode-request>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'mode)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetMode-request>)))
  "Returns string type for a service object of type '<SetMode-request>"
  "itm_nonlinear_mpc/SetModeRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetMode-request)))
  "Returns string type for a service object of type 'SetMode-request"
  "itm_nonlinear_mpc/SetModeRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetMode-request>)))
  "Returns md5sum for a message object of type '<SetMode-request>"
  "718da2351c63fdc303e9567a9bc6772c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetMode-request)))
  "Returns md5sum for a message object of type 'SetMode-request"
  "718da2351c63fdc303e9567a9bc6772c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetMode-request>)))
  "Returns full string definition for message of type '<SetMode-request>"
  (cl:format cl:nil "uint8 mode~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetMode-request)))
  "Returns full string definition for message of type 'SetMode-request"
  (cl:format cl:nil "uint8 mode~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetMode-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetMode-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetMode-request
    (cl:cons ':mode (mode msg))
))
;//! \htmlinclude SetMode-response.msg.html

(cl:defclass <SetMode-response> (roslisp-msg-protocol:ros-message)
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

(cl:defclass SetMode-response (<SetMode-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetMode-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetMode-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name itm_nonlinear_mpc-srv:<SetMode-response> is deprecated: use itm_nonlinear_mpc-srv:SetMode-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <SetMode-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader itm_nonlinear_mpc-srv:success-val is deprecated.  Use itm_nonlinear_mpc-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'message-val :lambda-list '(m))
(cl:defmethod message-val ((m <SetMode-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader itm_nonlinear_mpc-srv:message-val is deprecated.  Use itm_nonlinear_mpc-srv:message instead.")
  (message m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetMode-response>) ostream)
  "Serializes a message object of type '<SetMode-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'message))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'message))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetMode-response>) istream)
  "Deserializes a message object of type '<SetMode-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetMode-response>)))
  "Returns string type for a service object of type '<SetMode-response>"
  "itm_nonlinear_mpc/SetModeResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetMode-response)))
  "Returns string type for a service object of type 'SetMode-response"
  "itm_nonlinear_mpc/SetModeResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetMode-response>)))
  "Returns md5sum for a message object of type '<SetMode-response>"
  "718da2351c63fdc303e9567a9bc6772c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetMode-response)))
  "Returns md5sum for a message object of type 'SetMode-response"
  "718da2351c63fdc303e9567a9bc6772c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetMode-response>)))
  "Returns full string definition for message of type '<SetMode-response>"
  (cl:format cl:nil "bool success~%string message~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetMode-response)))
  "Returns full string definition for message of type 'SetMode-response"
  (cl:format cl:nil "bool success~%string message~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetMode-response>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'message))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetMode-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetMode-response
    (cl:cons ':success (success msg))
    (cl:cons ':message (message msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetMode)))
  'SetMode-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetMode)))
  'SetMode-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetMode)))
  "Returns string type for a service object of type '<SetMode>"
  "itm_nonlinear_mpc/SetMode")