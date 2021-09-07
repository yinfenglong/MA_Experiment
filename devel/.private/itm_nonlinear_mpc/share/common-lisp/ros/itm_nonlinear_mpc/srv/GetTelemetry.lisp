; Auto-generated. Do not edit!


(cl:in-package itm_nonlinear_mpc-srv)


;//! \htmlinclude GetTelemetry-request.msg.html

(cl:defclass <GetTelemetry-request> (roslisp-msg-protocol:ros-message)
  ((frame_id
    :reader frame_id
    :initarg :frame_id
    :type cl:string
    :initform ""))
)

(cl:defclass GetTelemetry-request (<GetTelemetry-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetTelemetry-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetTelemetry-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name itm_nonlinear_mpc-srv:<GetTelemetry-request> is deprecated: use itm_nonlinear_mpc-srv:GetTelemetry-request instead.")))

(cl:ensure-generic-function 'frame_id-val :lambda-list '(m))
(cl:defmethod frame_id-val ((m <GetTelemetry-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader itm_nonlinear_mpc-srv:frame_id-val is deprecated.  Use itm_nonlinear_mpc-srv:frame_id instead.")
  (frame_id m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetTelemetry-request>) ostream)
  "Serializes a message object of type '<GetTelemetry-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'frame_id))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'frame_id))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetTelemetry-request>) istream)
  "Deserializes a message object of type '<GetTelemetry-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'frame_id) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'frame_id) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetTelemetry-request>)))
  "Returns string type for a service object of type '<GetTelemetry-request>"
  "itm_nonlinear_mpc/GetTelemetryRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetTelemetry-request)))
  "Returns string type for a service object of type 'GetTelemetry-request"
  "itm_nonlinear_mpc/GetTelemetryRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetTelemetry-request>)))
  "Returns md5sum for a message object of type '<GetTelemetry-request>"
  "3f1c3a9f95e1194fe8ebc671ce98ef91")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetTelemetry-request)))
  "Returns md5sum for a message object of type 'GetTelemetry-request"
  "3f1c3a9f95e1194fe8ebc671ce98ef91")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetTelemetry-request>)))
  "Returns full string definition for message of type '<GetTelemetry-request>"
  (cl:format cl:nil "string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetTelemetry-request)))
  "Returns full string definition for message of type 'GetTelemetry-request"
  (cl:format cl:nil "string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetTelemetry-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'frame_id))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetTelemetry-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetTelemetry-request
    (cl:cons ':frame_id (frame_id msg))
))
;//! \htmlinclude GetTelemetry-response.msg.html

(cl:defclass <GetTelemetry-response> (roslisp-msg-protocol:ros-message)
  ((frame_id
    :reader frame_id
    :initarg :frame_id
    :type cl:string
    :initform "")
   (connected
    :reader connected
    :initarg :connected
    :type cl:boolean
    :initform cl:nil)
   (armed
    :reader armed
    :initarg :armed
    :type cl:boolean
    :initform cl:nil)
   (mode
    :reader mode
    :initarg :mode
    :type cl:string
    :initform "")
   (x
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
   (lat
    :reader lat
    :initarg :lat
    :type cl:float
    :initform 0.0)
   (lon
    :reader lon
    :initarg :lon
    :type cl:float
    :initform 0.0)
   (alt
    :reader alt
    :initarg :alt
    :type cl:float
    :initform 0.0)
   (vx
    :reader vx
    :initarg :vx
    :type cl:float
    :initform 0.0)
   (vy
    :reader vy
    :initarg :vy
    :type cl:float
    :initform 0.0)
   (vz
    :reader vz
    :initarg :vz
    :type cl:float
    :initform 0.0)
   (pitch
    :reader pitch
    :initarg :pitch
    :type cl:float
    :initform 0.0)
   (roll
    :reader roll
    :initarg :roll
    :type cl:float
    :initform 0.0)
   (yaw
    :reader yaw
    :initarg :yaw
    :type cl:float
    :initform 0.0)
   (pitch_rate
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
   (voltage
    :reader voltage
    :initarg :voltage
    :type cl:float
    :initform 0.0)
   (cell_voltage
    :reader cell_voltage
    :initarg :cell_voltage
    :type cl:float
    :initform 0.0))
)

(cl:defclass GetTelemetry-response (<GetTelemetry-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetTelemetry-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetTelemetry-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name itm_nonlinear_mpc-srv:<GetTelemetry-response> is deprecated: use itm_nonlinear_mpc-srv:GetTelemetry-response instead.")))

(cl:ensure-generic-function 'frame_id-val :lambda-list '(m))
(cl:defmethod frame_id-val ((m <GetTelemetry-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader itm_nonlinear_mpc-srv:frame_id-val is deprecated.  Use itm_nonlinear_mpc-srv:frame_id instead.")
  (frame_id m))

(cl:ensure-generic-function 'connected-val :lambda-list '(m))
(cl:defmethod connected-val ((m <GetTelemetry-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader itm_nonlinear_mpc-srv:connected-val is deprecated.  Use itm_nonlinear_mpc-srv:connected instead.")
  (connected m))

(cl:ensure-generic-function 'armed-val :lambda-list '(m))
(cl:defmethod armed-val ((m <GetTelemetry-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader itm_nonlinear_mpc-srv:armed-val is deprecated.  Use itm_nonlinear_mpc-srv:armed instead.")
  (armed m))

(cl:ensure-generic-function 'mode-val :lambda-list '(m))
(cl:defmethod mode-val ((m <GetTelemetry-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader itm_nonlinear_mpc-srv:mode-val is deprecated.  Use itm_nonlinear_mpc-srv:mode instead.")
  (mode m))

(cl:ensure-generic-function 'x-val :lambda-list '(m))
(cl:defmethod x-val ((m <GetTelemetry-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader itm_nonlinear_mpc-srv:x-val is deprecated.  Use itm_nonlinear_mpc-srv:x instead.")
  (x m))

(cl:ensure-generic-function 'y-val :lambda-list '(m))
(cl:defmethod y-val ((m <GetTelemetry-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader itm_nonlinear_mpc-srv:y-val is deprecated.  Use itm_nonlinear_mpc-srv:y instead.")
  (y m))

(cl:ensure-generic-function 'z-val :lambda-list '(m))
(cl:defmethod z-val ((m <GetTelemetry-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader itm_nonlinear_mpc-srv:z-val is deprecated.  Use itm_nonlinear_mpc-srv:z instead.")
  (z m))

(cl:ensure-generic-function 'lat-val :lambda-list '(m))
(cl:defmethod lat-val ((m <GetTelemetry-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader itm_nonlinear_mpc-srv:lat-val is deprecated.  Use itm_nonlinear_mpc-srv:lat instead.")
  (lat m))

(cl:ensure-generic-function 'lon-val :lambda-list '(m))
(cl:defmethod lon-val ((m <GetTelemetry-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader itm_nonlinear_mpc-srv:lon-val is deprecated.  Use itm_nonlinear_mpc-srv:lon instead.")
  (lon m))

(cl:ensure-generic-function 'alt-val :lambda-list '(m))
(cl:defmethod alt-val ((m <GetTelemetry-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader itm_nonlinear_mpc-srv:alt-val is deprecated.  Use itm_nonlinear_mpc-srv:alt instead.")
  (alt m))

(cl:ensure-generic-function 'vx-val :lambda-list '(m))
(cl:defmethod vx-val ((m <GetTelemetry-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader itm_nonlinear_mpc-srv:vx-val is deprecated.  Use itm_nonlinear_mpc-srv:vx instead.")
  (vx m))

(cl:ensure-generic-function 'vy-val :lambda-list '(m))
(cl:defmethod vy-val ((m <GetTelemetry-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader itm_nonlinear_mpc-srv:vy-val is deprecated.  Use itm_nonlinear_mpc-srv:vy instead.")
  (vy m))

(cl:ensure-generic-function 'vz-val :lambda-list '(m))
(cl:defmethod vz-val ((m <GetTelemetry-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader itm_nonlinear_mpc-srv:vz-val is deprecated.  Use itm_nonlinear_mpc-srv:vz instead.")
  (vz m))

(cl:ensure-generic-function 'pitch-val :lambda-list '(m))
(cl:defmethod pitch-val ((m <GetTelemetry-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader itm_nonlinear_mpc-srv:pitch-val is deprecated.  Use itm_nonlinear_mpc-srv:pitch instead.")
  (pitch m))

(cl:ensure-generic-function 'roll-val :lambda-list '(m))
(cl:defmethod roll-val ((m <GetTelemetry-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader itm_nonlinear_mpc-srv:roll-val is deprecated.  Use itm_nonlinear_mpc-srv:roll instead.")
  (roll m))

(cl:ensure-generic-function 'yaw-val :lambda-list '(m))
(cl:defmethod yaw-val ((m <GetTelemetry-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader itm_nonlinear_mpc-srv:yaw-val is deprecated.  Use itm_nonlinear_mpc-srv:yaw instead.")
  (yaw m))

(cl:ensure-generic-function 'pitch_rate-val :lambda-list '(m))
(cl:defmethod pitch_rate-val ((m <GetTelemetry-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader itm_nonlinear_mpc-srv:pitch_rate-val is deprecated.  Use itm_nonlinear_mpc-srv:pitch_rate instead.")
  (pitch_rate m))

(cl:ensure-generic-function 'roll_rate-val :lambda-list '(m))
(cl:defmethod roll_rate-val ((m <GetTelemetry-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader itm_nonlinear_mpc-srv:roll_rate-val is deprecated.  Use itm_nonlinear_mpc-srv:roll_rate instead.")
  (roll_rate m))

(cl:ensure-generic-function 'yaw_rate-val :lambda-list '(m))
(cl:defmethod yaw_rate-val ((m <GetTelemetry-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader itm_nonlinear_mpc-srv:yaw_rate-val is deprecated.  Use itm_nonlinear_mpc-srv:yaw_rate instead.")
  (yaw_rate m))

(cl:ensure-generic-function 'voltage-val :lambda-list '(m))
(cl:defmethod voltage-val ((m <GetTelemetry-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader itm_nonlinear_mpc-srv:voltage-val is deprecated.  Use itm_nonlinear_mpc-srv:voltage instead.")
  (voltage m))

(cl:ensure-generic-function 'cell_voltage-val :lambda-list '(m))
(cl:defmethod cell_voltage-val ((m <GetTelemetry-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader itm_nonlinear_mpc-srv:cell_voltage-val is deprecated.  Use itm_nonlinear_mpc-srv:cell_voltage instead.")
  (cell_voltage m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetTelemetry-response>) ostream)
  "Serializes a message object of type '<GetTelemetry-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'frame_id))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'frame_id))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'connected) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'armed) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'mode))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'mode))
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
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'alt))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'vx))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'vy))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'vz))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'pitch))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'roll))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'yaw))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
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
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'voltage))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'cell_voltage))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetTelemetry-response>) istream)
  "Deserializes a message object of type '<GetTelemetry-response>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'frame_id) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'frame_id) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:setf (cl:slot-value msg 'connected) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'armed) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'mode) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'mode) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
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
    (cl:setf (cl:slot-value msg 'alt) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'vx) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'vy) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'vz) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'pitch) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'roll) (roslisp-utils:decode-single-float-bits bits)))
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
    (cl:setf (cl:slot-value msg 'voltage) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'cell_voltage) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetTelemetry-response>)))
  "Returns string type for a service object of type '<GetTelemetry-response>"
  "itm_nonlinear_mpc/GetTelemetryResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetTelemetry-response)))
  "Returns string type for a service object of type 'GetTelemetry-response"
  "itm_nonlinear_mpc/GetTelemetryResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetTelemetry-response>)))
  "Returns md5sum for a message object of type '<GetTelemetry-response>"
  "3f1c3a9f95e1194fe8ebc671ce98ef91")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetTelemetry-response)))
  "Returns md5sum for a message object of type 'GetTelemetry-response"
  "3f1c3a9f95e1194fe8ebc671ce98ef91")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetTelemetry-response>)))
  "Returns full string definition for message of type '<GetTelemetry-response>"
  (cl:format cl:nil "string frame_id~%bool connected~%bool armed~%string mode~%float32 x~%float32 y~%float32 z~%float64 lat~%float64 lon~%float32 alt~%float32 vx~%float32 vy~%float32 vz~%float32 pitch~%float32 roll~%float32 yaw~%float32 pitch_rate~%float32 roll_rate~%float32 yaw_rate~%float32 voltage~%float32 cell_voltage~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetTelemetry-response)))
  "Returns full string definition for message of type 'GetTelemetry-response"
  (cl:format cl:nil "string frame_id~%bool connected~%bool armed~%string mode~%float32 x~%float32 y~%float32 z~%float64 lat~%float64 lon~%float32 alt~%float32 vx~%float32 vy~%float32 vz~%float32 pitch~%float32 roll~%float32 yaw~%float32 pitch_rate~%float32 roll_rate~%float32 yaw_rate~%float32 voltage~%float32 cell_voltage~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetTelemetry-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'frame_id))
     1
     1
     4 (cl:length (cl:slot-value msg 'mode))
     4
     4
     4
     8
     8
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetTelemetry-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetTelemetry-response
    (cl:cons ':frame_id (frame_id msg))
    (cl:cons ':connected (connected msg))
    (cl:cons ':armed (armed msg))
    (cl:cons ':mode (mode msg))
    (cl:cons ':x (x msg))
    (cl:cons ':y (y msg))
    (cl:cons ':z (z msg))
    (cl:cons ':lat (lat msg))
    (cl:cons ':lon (lon msg))
    (cl:cons ':alt (alt msg))
    (cl:cons ':vx (vx msg))
    (cl:cons ':vy (vy msg))
    (cl:cons ':vz (vz msg))
    (cl:cons ':pitch (pitch msg))
    (cl:cons ':roll (roll msg))
    (cl:cons ':yaw (yaw msg))
    (cl:cons ':pitch_rate (pitch_rate msg))
    (cl:cons ':roll_rate (roll_rate msg))
    (cl:cons ':yaw_rate (yaw_rate msg))
    (cl:cons ':voltage (voltage msg))
    (cl:cons ':cell_voltage (cell_voltage msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetTelemetry)))
  'GetTelemetry-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetTelemetry)))
  'GetTelemetry-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetTelemetry)))
  "Returns string type for a service object of type '<GetTelemetry>"
  "itm_nonlinear_mpc/GetTelemetry")