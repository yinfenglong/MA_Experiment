; Auto-generated. Do not edit!


(cl:in-package itm_mav_srvs-srv)


;//! \htmlinclude GetControllerState-request.msg.html

(cl:defclass <GetControllerState-request> (roslisp-msg-protocol:ros-message)
  ((robot_name
    :reader robot_name
    :initarg :robot_name
    :type cl:string
    :initform "")
   (command_id
    :reader command_id
    :initarg :command_id
    :type cl:fixnum
    :initform 0))
)

(cl:defclass GetControllerState-request (<GetControllerState-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetControllerState-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetControllerState-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name itm_mav_srvs-srv:<GetControllerState-request> is deprecated: use itm_mav_srvs-srv:GetControllerState-request instead.")))

(cl:ensure-generic-function 'robot_name-val :lambda-list '(m))
(cl:defmethod robot_name-val ((m <GetControllerState-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader itm_mav_srvs-srv:robot_name-val is deprecated.  Use itm_mav_srvs-srv:robot_name instead.")
  (robot_name m))

(cl:ensure-generic-function 'command_id-val :lambda-list '(m))
(cl:defmethod command_id-val ((m <GetControllerState-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader itm_mav_srvs-srv:command_id-val is deprecated.  Use itm_mav_srvs-srv:command_id instead.")
  (command_id m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetControllerState-request>) ostream)
  "Serializes a message object of type '<GetControllerState-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'robot_name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'robot_name))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'command_id)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetControllerState-request>) istream)
  "Deserializes a message object of type '<GetControllerState-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'robot_name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'robot_name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'command_id)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetControllerState-request>)))
  "Returns string type for a service object of type '<GetControllerState-request>"
  "itm_mav_srvs/GetControllerStateRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetControllerState-request)))
  "Returns string type for a service object of type 'GetControllerState-request"
  "itm_mav_srvs/GetControllerStateRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetControllerState-request>)))
  "Returns md5sum for a message object of type '<GetControllerState-request>"
  "5ded73ff38c3ac0ab83d4d33c5cfb2fd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetControllerState-request)))
  "Returns md5sum for a message object of type 'GetControllerState-request"
  "5ded73ff38c3ac0ab83d4d33c5cfb2fd")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetControllerState-request>)))
  "Returns full string definition for message of type '<GetControllerState-request>"
  (cl:format cl:nil "string robot_name~%uint8 command_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetControllerState-request)))
  "Returns full string definition for message of type 'GetControllerState-request"
  (cl:format cl:nil "string robot_name~%uint8 command_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetControllerState-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'robot_name))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetControllerState-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetControllerState-request
    (cl:cons ':robot_name (robot_name msg))
    (cl:cons ':command_id (command_id msg))
))
;//! \htmlinclude GetControllerState-response.msg.html

(cl:defclass <GetControllerState-response> (roslisp-msg-protocol:ros-message)
  ((connected
    :reader connected
    :initarg :connected
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass GetControllerState-response (<GetControllerState-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetControllerState-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetControllerState-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name itm_mav_srvs-srv:<GetControllerState-response> is deprecated: use itm_mav_srvs-srv:GetControllerState-response instead.")))

(cl:ensure-generic-function 'connected-val :lambda-list '(m))
(cl:defmethod connected-val ((m <GetControllerState-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader itm_mav_srvs-srv:connected-val is deprecated.  Use itm_mav_srvs-srv:connected instead.")
  (connected m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetControllerState-response>) ostream)
  "Serializes a message object of type '<GetControllerState-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'connected) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetControllerState-response>) istream)
  "Deserializes a message object of type '<GetControllerState-response>"
    (cl:setf (cl:slot-value msg 'connected) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetControllerState-response>)))
  "Returns string type for a service object of type '<GetControllerState-response>"
  "itm_mav_srvs/GetControllerStateResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetControllerState-response)))
  "Returns string type for a service object of type 'GetControllerState-response"
  "itm_mav_srvs/GetControllerStateResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetControllerState-response>)))
  "Returns md5sum for a message object of type '<GetControllerState-response>"
  "5ded73ff38c3ac0ab83d4d33c5cfb2fd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetControllerState-response)))
  "Returns md5sum for a message object of type 'GetControllerState-response"
  "5ded73ff38c3ac0ab83d4d33c5cfb2fd")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetControllerState-response>)))
  "Returns full string definition for message of type '<GetControllerState-response>"
  (cl:format cl:nil "bool connected~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetControllerState-response)))
  "Returns full string definition for message of type 'GetControllerState-response"
  (cl:format cl:nil "bool connected~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetControllerState-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetControllerState-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetControllerState-response
    (cl:cons ':connected (connected msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetControllerState)))
  'GetControllerState-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetControllerState)))
  'GetControllerState-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetControllerState)))
  "Returns string type for a service object of type '<GetControllerState>"
  "itm_mav_srvs/GetControllerState")