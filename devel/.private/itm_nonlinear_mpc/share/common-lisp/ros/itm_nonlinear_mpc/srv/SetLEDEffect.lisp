; Auto-generated. Do not edit!


(cl:in-package itm_nonlinear_mpc-srv)


;//! \htmlinclude SetLEDEffect-request.msg.html

(cl:defclass <SetLEDEffect-request> (roslisp-msg-protocol:ros-message)
  ((effect
    :reader effect
    :initarg :effect
    :type cl:string
    :initform "")
   (r
    :reader r
    :initarg :r
    :type cl:fixnum
    :initform 0)
   (g
    :reader g
    :initarg :g
    :type cl:fixnum
    :initform 0)
   (b
    :reader b
    :initarg :b
    :type cl:fixnum
    :initform 0))
)

(cl:defclass SetLEDEffect-request (<SetLEDEffect-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetLEDEffect-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetLEDEffect-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name itm_nonlinear_mpc-srv:<SetLEDEffect-request> is deprecated: use itm_nonlinear_mpc-srv:SetLEDEffect-request instead.")))

(cl:ensure-generic-function 'effect-val :lambda-list '(m))
(cl:defmethod effect-val ((m <SetLEDEffect-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader itm_nonlinear_mpc-srv:effect-val is deprecated.  Use itm_nonlinear_mpc-srv:effect instead.")
  (effect m))

(cl:ensure-generic-function 'r-val :lambda-list '(m))
(cl:defmethod r-val ((m <SetLEDEffect-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader itm_nonlinear_mpc-srv:r-val is deprecated.  Use itm_nonlinear_mpc-srv:r instead.")
  (r m))

(cl:ensure-generic-function 'g-val :lambda-list '(m))
(cl:defmethod g-val ((m <SetLEDEffect-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader itm_nonlinear_mpc-srv:g-val is deprecated.  Use itm_nonlinear_mpc-srv:g instead.")
  (g m))

(cl:ensure-generic-function 'b-val :lambda-list '(m))
(cl:defmethod b-val ((m <SetLEDEffect-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader itm_nonlinear_mpc-srv:b-val is deprecated.  Use itm_nonlinear_mpc-srv:b instead.")
  (b m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetLEDEffect-request>) ostream)
  "Serializes a message object of type '<SetLEDEffect-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'effect))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'effect))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'r)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'g)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'b)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetLEDEffect-request>) istream)
  "Deserializes a message object of type '<SetLEDEffect-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'effect) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'effect) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'r)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'g)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'b)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetLEDEffect-request>)))
  "Returns string type for a service object of type '<SetLEDEffect-request>"
  "itm_nonlinear_mpc/SetLEDEffectRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetLEDEffect-request)))
  "Returns string type for a service object of type 'SetLEDEffect-request"
  "itm_nonlinear_mpc/SetLEDEffectRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetLEDEffect-request>)))
  "Returns md5sum for a message object of type '<SetLEDEffect-request>"
  "044f75c927403b22bd59e8dbf871eabd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetLEDEffect-request)))
  "Returns md5sum for a message object of type 'SetLEDEffect-request"
  "044f75c927403b22bd59e8dbf871eabd")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetLEDEffect-request>)))
  "Returns full string definition for message of type '<SetLEDEffect-request>"
  (cl:format cl:nil "string effect~%uint8 r~%uint8 g~%uint8 b~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetLEDEffect-request)))
  "Returns full string definition for message of type 'SetLEDEffect-request"
  (cl:format cl:nil "string effect~%uint8 r~%uint8 g~%uint8 b~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetLEDEffect-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'effect))
     1
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetLEDEffect-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetLEDEffect-request
    (cl:cons ':effect (effect msg))
    (cl:cons ':r (r msg))
    (cl:cons ':g (g msg))
    (cl:cons ':b (b msg))
))
;//! \htmlinclude SetLEDEffect-response.msg.html

(cl:defclass <SetLEDEffect-response> (roslisp-msg-protocol:ros-message)
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

(cl:defclass SetLEDEffect-response (<SetLEDEffect-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetLEDEffect-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetLEDEffect-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name itm_nonlinear_mpc-srv:<SetLEDEffect-response> is deprecated: use itm_nonlinear_mpc-srv:SetLEDEffect-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <SetLEDEffect-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader itm_nonlinear_mpc-srv:success-val is deprecated.  Use itm_nonlinear_mpc-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'message-val :lambda-list '(m))
(cl:defmethod message-val ((m <SetLEDEffect-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader itm_nonlinear_mpc-srv:message-val is deprecated.  Use itm_nonlinear_mpc-srv:message instead.")
  (message m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetLEDEffect-response>) ostream)
  "Serializes a message object of type '<SetLEDEffect-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'message))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'message))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetLEDEffect-response>) istream)
  "Deserializes a message object of type '<SetLEDEffect-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetLEDEffect-response>)))
  "Returns string type for a service object of type '<SetLEDEffect-response>"
  "itm_nonlinear_mpc/SetLEDEffectResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetLEDEffect-response)))
  "Returns string type for a service object of type 'SetLEDEffect-response"
  "itm_nonlinear_mpc/SetLEDEffectResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetLEDEffect-response>)))
  "Returns md5sum for a message object of type '<SetLEDEffect-response>"
  "044f75c927403b22bd59e8dbf871eabd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetLEDEffect-response)))
  "Returns md5sum for a message object of type 'SetLEDEffect-response"
  "044f75c927403b22bd59e8dbf871eabd")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetLEDEffect-response>)))
  "Returns full string definition for message of type '<SetLEDEffect-response>"
  (cl:format cl:nil "bool success~%string message~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetLEDEffect-response)))
  "Returns full string definition for message of type 'SetLEDEffect-response"
  (cl:format cl:nil "bool success~%string message~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetLEDEffect-response>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'message))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetLEDEffect-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetLEDEffect-response
    (cl:cons ':success (success msg))
    (cl:cons ':message (message msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetLEDEffect)))
  'SetLEDEffect-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetLEDEffect)))
  'SetLEDEffect-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetLEDEffect)))
  "Returns string type for a service object of type '<SetLEDEffect>"
  "itm_nonlinear_mpc/SetLEDEffect")