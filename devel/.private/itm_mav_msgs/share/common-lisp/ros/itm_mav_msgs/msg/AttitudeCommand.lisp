; Auto-generated. Do not edit!


(cl:in-package itm_mav_msgs-msg)


;//! \htmlinclude AttitudeCommand.msg.html

(cl:defclass <AttitudeCommand> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (controller
    :reader controller
    :initarg :controller
    :type cl:string
    :initform "")
   (ramping_up
    :reader ramping_up
    :initarg :ramping_up
    :type cl:boolean
    :initform cl:nil)
   (attitude
    :reader attitude
    :initarg :attitude
    :type geometry_msgs-msg:Quaternion
    :initform (cl:make-instance 'geometry_msgs-msg:Quaternion))
   (attitude_rate
    :reader attitude_rate
    :initarg :attitude_rate
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point))
   (desired_acceleration
    :reader desired_acceleration
    :initarg :desired_acceleration
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point))
   (thrust
    :reader thrust
    :initarg :thrust
    :type cl:float
    :initform 0.0)
   (mass_difference
    :reader mass_difference
    :initarg :mass_difference
    :type cl:float
    :initform 0.0)
   (total_mass
    :reader total_mass
    :initarg :total_mass
    :type cl:float
    :initform 0.0)
   (disturbance_wx_w
    :reader disturbance_wx_w
    :initarg :disturbance_wx_w
    :type cl:float
    :initform 0.0)
   (disturbance_wy_w
    :reader disturbance_wy_w
    :initarg :disturbance_wy_w
    :type cl:float
    :initform 0.0)
   (disturbance_bx_w
    :reader disturbance_bx_w
    :initarg :disturbance_bx_w
    :type cl:float
    :initform 0.0)
   (disturbance_by_w
    :reader disturbance_by_w
    :initarg :disturbance_by_w
    :type cl:float
    :initform 0.0)
   (disturbance_bx_b
    :reader disturbance_bx_b
    :initarg :disturbance_bx_b
    :type cl:float
    :initform 0.0)
   (disturbance_by_b
    :reader disturbance_by_b
    :initarg :disturbance_by_b
    :type cl:float
    :initform 0.0)
   (controller_enforcing_constraints
    :reader controller_enforcing_constraints
    :initarg :controller_enforcing_constraints
    :type cl:boolean
    :initform cl:nil)
   (horizontal_speed_constraint
    :reader horizontal_speed_constraint
    :initarg :horizontal_speed_constraint
    :type cl:float
    :initform 0.0)
   (horizontal_acc_constraint
    :reader horizontal_acc_constraint
    :initarg :horizontal_acc_constraint
    :type cl:float
    :initform 0.0)
   (vertical_asc_speed_constraint
    :reader vertical_asc_speed_constraint
    :initarg :vertical_asc_speed_constraint
    :type cl:float
    :initform 0.0)
   (vertical_asc_acc_constraint
    :reader vertical_asc_acc_constraint
    :initarg :vertical_asc_acc_constraint
    :type cl:float
    :initform 0.0)
   (vertical_desc_speed_constraint
    :reader vertical_desc_speed_constraint
    :initarg :vertical_desc_speed_constraint
    :type cl:float
    :initform 0.0)
   (vertical_desc_acc_constraint
    :reader vertical_desc_acc_constraint
    :initarg :vertical_desc_acc_constraint
    :type cl:float
    :initform 0.0)
   (mode_mask
    :reader mode_mask
    :initarg :mode_mask
    :type cl:fixnum
    :initform 0))
)

(cl:defclass AttitudeCommand (<AttitudeCommand>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <AttitudeCommand>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'AttitudeCommand)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name itm_mav_msgs-msg:<AttitudeCommand> is deprecated: use itm_mav_msgs-msg:AttitudeCommand instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <AttitudeCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader itm_mav_msgs-msg:header-val is deprecated.  Use itm_mav_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'controller-val :lambda-list '(m))
(cl:defmethod controller-val ((m <AttitudeCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader itm_mav_msgs-msg:controller-val is deprecated.  Use itm_mav_msgs-msg:controller instead.")
  (controller m))

(cl:ensure-generic-function 'ramping_up-val :lambda-list '(m))
(cl:defmethod ramping_up-val ((m <AttitudeCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader itm_mav_msgs-msg:ramping_up-val is deprecated.  Use itm_mav_msgs-msg:ramping_up instead.")
  (ramping_up m))

(cl:ensure-generic-function 'attitude-val :lambda-list '(m))
(cl:defmethod attitude-val ((m <AttitudeCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader itm_mav_msgs-msg:attitude-val is deprecated.  Use itm_mav_msgs-msg:attitude instead.")
  (attitude m))

(cl:ensure-generic-function 'attitude_rate-val :lambda-list '(m))
(cl:defmethod attitude_rate-val ((m <AttitudeCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader itm_mav_msgs-msg:attitude_rate-val is deprecated.  Use itm_mav_msgs-msg:attitude_rate instead.")
  (attitude_rate m))

(cl:ensure-generic-function 'desired_acceleration-val :lambda-list '(m))
(cl:defmethod desired_acceleration-val ((m <AttitudeCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader itm_mav_msgs-msg:desired_acceleration-val is deprecated.  Use itm_mav_msgs-msg:desired_acceleration instead.")
  (desired_acceleration m))

(cl:ensure-generic-function 'thrust-val :lambda-list '(m))
(cl:defmethod thrust-val ((m <AttitudeCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader itm_mav_msgs-msg:thrust-val is deprecated.  Use itm_mav_msgs-msg:thrust instead.")
  (thrust m))

(cl:ensure-generic-function 'mass_difference-val :lambda-list '(m))
(cl:defmethod mass_difference-val ((m <AttitudeCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader itm_mav_msgs-msg:mass_difference-val is deprecated.  Use itm_mav_msgs-msg:mass_difference instead.")
  (mass_difference m))

(cl:ensure-generic-function 'total_mass-val :lambda-list '(m))
(cl:defmethod total_mass-val ((m <AttitudeCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader itm_mav_msgs-msg:total_mass-val is deprecated.  Use itm_mav_msgs-msg:total_mass instead.")
  (total_mass m))

(cl:ensure-generic-function 'disturbance_wx_w-val :lambda-list '(m))
(cl:defmethod disturbance_wx_w-val ((m <AttitudeCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader itm_mav_msgs-msg:disturbance_wx_w-val is deprecated.  Use itm_mav_msgs-msg:disturbance_wx_w instead.")
  (disturbance_wx_w m))

(cl:ensure-generic-function 'disturbance_wy_w-val :lambda-list '(m))
(cl:defmethod disturbance_wy_w-val ((m <AttitudeCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader itm_mav_msgs-msg:disturbance_wy_w-val is deprecated.  Use itm_mav_msgs-msg:disturbance_wy_w instead.")
  (disturbance_wy_w m))

(cl:ensure-generic-function 'disturbance_bx_w-val :lambda-list '(m))
(cl:defmethod disturbance_bx_w-val ((m <AttitudeCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader itm_mav_msgs-msg:disturbance_bx_w-val is deprecated.  Use itm_mav_msgs-msg:disturbance_bx_w instead.")
  (disturbance_bx_w m))

(cl:ensure-generic-function 'disturbance_by_w-val :lambda-list '(m))
(cl:defmethod disturbance_by_w-val ((m <AttitudeCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader itm_mav_msgs-msg:disturbance_by_w-val is deprecated.  Use itm_mav_msgs-msg:disturbance_by_w instead.")
  (disturbance_by_w m))

(cl:ensure-generic-function 'disturbance_bx_b-val :lambda-list '(m))
(cl:defmethod disturbance_bx_b-val ((m <AttitudeCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader itm_mav_msgs-msg:disturbance_bx_b-val is deprecated.  Use itm_mav_msgs-msg:disturbance_bx_b instead.")
  (disturbance_bx_b m))

(cl:ensure-generic-function 'disturbance_by_b-val :lambda-list '(m))
(cl:defmethod disturbance_by_b-val ((m <AttitudeCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader itm_mav_msgs-msg:disturbance_by_b-val is deprecated.  Use itm_mav_msgs-msg:disturbance_by_b instead.")
  (disturbance_by_b m))

(cl:ensure-generic-function 'controller_enforcing_constraints-val :lambda-list '(m))
(cl:defmethod controller_enforcing_constraints-val ((m <AttitudeCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader itm_mav_msgs-msg:controller_enforcing_constraints-val is deprecated.  Use itm_mav_msgs-msg:controller_enforcing_constraints instead.")
  (controller_enforcing_constraints m))

(cl:ensure-generic-function 'horizontal_speed_constraint-val :lambda-list '(m))
(cl:defmethod horizontal_speed_constraint-val ((m <AttitudeCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader itm_mav_msgs-msg:horizontal_speed_constraint-val is deprecated.  Use itm_mav_msgs-msg:horizontal_speed_constraint instead.")
  (horizontal_speed_constraint m))

(cl:ensure-generic-function 'horizontal_acc_constraint-val :lambda-list '(m))
(cl:defmethod horizontal_acc_constraint-val ((m <AttitudeCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader itm_mav_msgs-msg:horizontal_acc_constraint-val is deprecated.  Use itm_mav_msgs-msg:horizontal_acc_constraint instead.")
  (horizontal_acc_constraint m))

(cl:ensure-generic-function 'vertical_asc_speed_constraint-val :lambda-list '(m))
(cl:defmethod vertical_asc_speed_constraint-val ((m <AttitudeCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader itm_mav_msgs-msg:vertical_asc_speed_constraint-val is deprecated.  Use itm_mav_msgs-msg:vertical_asc_speed_constraint instead.")
  (vertical_asc_speed_constraint m))

(cl:ensure-generic-function 'vertical_asc_acc_constraint-val :lambda-list '(m))
(cl:defmethod vertical_asc_acc_constraint-val ((m <AttitudeCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader itm_mav_msgs-msg:vertical_asc_acc_constraint-val is deprecated.  Use itm_mav_msgs-msg:vertical_asc_acc_constraint instead.")
  (vertical_asc_acc_constraint m))

(cl:ensure-generic-function 'vertical_desc_speed_constraint-val :lambda-list '(m))
(cl:defmethod vertical_desc_speed_constraint-val ((m <AttitudeCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader itm_mav_msgs-msg:vertical_desc_speed_constraint-val is deprecated.  Use itm_mav_msgs-msg:vertical_desc_speed_constraint instead.")
  (vertical_desc_speed_constraint m))

(cl:ensure-generic-function 'vertical_desc_acc_constraint-val :lambda-list '(m))
(cl:defmethod vertical_desc_acc_constraint-val ((m <AttitudeCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader itm_mav_msgs-msg:vertical_desc_acc_constraint-val is deprecated.  Use itm_mav_msgs-msg:vertical_desc_acc_constraint instead.")
  (vertical_desc_acc_constraint m))

(cl:ensure-generic-function 'mode_mask-val :lambda-list '(m))
(cl:defmethod mode_mask-val ((m <AttitudeCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader itm_mav_msgs-msg:mode_mask-val is deprecated.  Use itm_mav_msgs-msg:mode_mask instead.")
  (mode_mask m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<AttitudeCommand>)))
    "Constants for message type '<AttitudeCommand>"
  '((:MODE_ATTITUDE . 1)
    (:MODE_ATTITUDE_RATE . 2))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'AttitudeCommand)))
    "Constants for message type 'AttitudeCommand"
  '((:MODE_ATTITUDE . 1)
    (:MODE_ATTITUDE_RATE . 2))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <AttitudeCommand>) ostream)
  "Serializes a message object of type '<AttitudeCommand>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'controller))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'controller))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'ramping_up) 1 0)) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'attitude) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'attitude_rate) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'desired_acceleration) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'thrust))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'mass_difference))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'total_mass))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'disturbance_wx_w))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'disturbance_wy_w))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'disturbance_bx_w))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'disturbance_by_w))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'disturbance_bx_b))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'disturbance_by_b))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'controller_enforcing_constraints) 1 0)) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'horizontal_speed_constraint))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'horizontal_acc_constraint))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'vertical_asc_speed_constraint))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'vertical_asc_acc_constraint))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'vertical_desc_speed_constraint))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'vertical_desc_acc_constraint))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'mode_mask)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <AttitudeCommand>) istream)
  "Deserializes a message object of type '<AttitudeCommand>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'controller) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'controller) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:setf (cl:slot-value msg 'ramping_up) (cl:not (cl:zerop (cl:read-byte istream))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'attitude) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'attitude_rate) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'desired_acceleration) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'thrust) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'mass_difference) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'total_mass) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'disturbance_wx_w) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'disturbance_wy_w) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'disturbance_bx_w) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'disturbance_by_w) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'disturbance_bx_b) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'disturbance_by_b) (roslisp-utils:decode-double-float-bits bits)))
    (cl:setf (cl:slot-value msg 'controller_enforcing_constraints) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'horizontal_speed_constraint) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'horizontal_acc_constraint) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'vertical_asc_speed_constraint) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'vertical_asc_acc_constraint) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'vertical_desc_speed_constraint) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'vertical_desc_acc_constraint) (roslisp-utils:decode-double-float-bits bits)))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'mode_mask)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<AttitudeCommand>)))
  "Returns string type for a message object of type '<AttitudeCommand>"
  "itm_mav_msgs/AttitudeCommand")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AttitudeCommand)))
  "Returns string type for a message object of type 'AttitudeCommand"
  "itm_mav_msgs/AttitudeCommand")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<AttitudeCommand>)))
  "Returns md5sum for a message object of type '<AttitudeCommand>"
  "ba99a1fcefbbc4c8eb8328bcdd1d674c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'AttitudeCommand)))
  "Returns md5sum for a message object of type 'AttitudeCommand"
  "ba99a1fcefbbc4c8eb8328bcdd1d674c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<AttitudeCommand>)))
  "Returns full string definition for message of type '<AttitudeCommand>"
  (cl:format cl:nil "# This represents an output of a UAV feedback controller (mrs_uav_manager::Controller).~%# This message is returned from a controller by means of the update() function, called by the mrs_uav_manager::ControlManager.~%~%# fork from CTU work~%~%std_msgs/Header header~%~%# The name of the controller (implementation-wise).~%# Beware, multiple instances of the same controller can be running.~%# The ControlManagerDiagnostics message contains the name given them~%# by the ControlManager.~%string controller~%~%# True if the controller is in the initial rampup phase (just after activation).~%bool ramping_up~%~%# The desired orientation produced by the controller.~%# This field should be filled every time.~%geometry_msgs/Quaternion attitude~%~%# The desired attitude rate produced by the controller.~%# This field is optional.~%geometry_msgs/Point attitude_rate~%~%# Desired acceleration produced by the controller.~%# This field is mandatory if flying with \"mrs_odometry\".~%# The desired acceleration should be without calculate without~%# compensation of external forces and disturbances, e.g., without~%# the compensation for the gravity vector, wind and internal UAV biases.~%geometry_msgs/Point desired_acceleration~%~%# The desired thrust, [0, 1].~%float64 thrust~%~%# The estimated mass difference from the nominal mass.~%float64 mass_difference~%~%# Total estimated UAV mass.~%float64 total_mass~%~%# World-frame disturbances expressed in the world frame.~%float64 disturbance_wx_w~%float64 disturbance_wy_w~%~%# Body-frame (fcu_untilted) disturbances expressed in the world frame.~%float64 disturbance_bx_w~%float64 disturbance_by_w~%~%# Body-frame (fcu_untilted) disturbances expressed in the body frame (fcu_untilted).~%float64 disturbance_bx_b~%float64 disturbance_by_b~%~%# The controller can enforce dynamics constraints over the trackers.~%# This can be used when flying with a controller that is limited to~%# some maximum speed and acceleration.~%bool controller_enforcing_constraints~%float64 horizontal_speed_constraint~%float64 horizontal_acc_constraint~%float64 vertical_asc_speed_constraint~%float64 vertical_asc_acc_constraint~%float64 vertical_desc_speed_constraint~%float64 vertical_desc_acc_constraint~%~%# Defines what output should be used, whether the attitude~%# or the attitude rate.~%uint8 mode_mask~%uint8 MODE_ATTITUDE=1~%uint8 MODE_ATTITUDE_RATE=2~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'AttitudeCommand)))
  "Returns full string definition for message of type 'AttitudeCommand"
  (cl:format cl:nil "# This represents an output of a UAV feedback controller (mrs_uav_manager::Controller).~%# This message is returned from a controller by means of the update() function, called by the mrs_uav_manager::ControlManager.~%~%# fork from CTU work~%~%std_msgs/Header header~%~%# The name of the controller (implementation-wise).~%# Beware, multiple instances of the same controller can be running.~%# The ControlManagerDiagnostics message contains the name given them~%# by the ControlManager.~%string controller~%~%# True if the controller is in the initial rampup phase (just after activation).~%bool ramping_up~%~%# The desired orientation produced by the controller.~%# This field should be filled every time.~%geometry_msgs/Quaternion attitude~%~%# The desired attitude rate produced by the controller.~%# This field is optional.~%geometry_msgs/Point attitude_rate~%~%# Desired acceleration produced by the controller.~%# This field is mandatory if flying with \"mrs_odometry\".~%# The desired acceleration should be without calculate without~%# compensation of external forces and disturbances, e.g., without~%# the compensation for the gravity vector, wind and internal UAV biases.~%geometry_msgs/Point desired_acceleration~%~%# The desired thrust, [0, 1].~%float64 thrust~%~%# The estimated mass difference from the nominal mass.~%float64 mass_difference~%~%# Total estimated UAV mass.~%float64 total_mass~%~%# World-frame disturbances expressed in the world frame.~%float64 disturbance_wx_w~%float64 disturbance_wy_w~%~%# Body-frame (fcu_untilted) disturbances expressed in the world frame.~%float64 disturbance_bx_w~%float64 disturbance_by_w~%~%# Body-frame (fcu_untilted) disturbances expressed in the body frame (fcu_untilted).~%float64 disturbance_bx_b~%float64 disturbance_by_b~%~%# The controller can enforce dynamics constraints over the trackers.~%# This can be used when flying with a controller that is limited to~%# some maximum speed and acceleration.~%bool controller_enforcing_constraints~%float64 horizontal_speed_constraint~%float64 horizontal_acc_constraint~%float64 vertical_asc_speed_constraint~%float64 vertical_asc_acc_constraint~%float64 vertical_desc_speed_constraint~%float64 vertical_desc_acc_constraint~%~%# Defines what output should be used, whether the attitude~%# or the attitude rate.~%uint8 mode_mask~%uint8 MODE_ATTITUDE=1~%uint8 MODE_ATTITUDE_RATE=2~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <AttitudeCommand>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:length (cl:slot-value msg 'controller))
     1
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'attitude))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'attitude_rate))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'desired_acceleration))
     8
     8
     8
     8
     8
     8
     8
     8
     8
     1
     8
     8
     8
     8
     8
     8
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <AttitudeCommand>))
  "Converts a ROS message object to a list"
  (cl:list 'AttitudeCommand
    (cl:cons ':header (header msg))
    (cl:cons ':controller (controller msg))
    (cl:cons ':ramping_up (ramping_up msg))
    (cl:cons ':attitude (attitude msg))
    (cl:cons ':attitude_rate (attitude_rate msg))
    (cl:cons ':desired_acceleration (desired_acceleration msg))
    (cl:cons ':thrust (thrust msg))
    (cl:cons ':mass_difference (mass_difference msg))
    (cl:cons ':total_mass (total_mass msg))
    (cl:cons ':disturbance_wx_w (disturbance_wx_w msg))
    (cl:cons ':disturbance_wy_w (disturbance_wy_w msg))
    (cl:cons ':disturbance_bx_w (disturbance_bx_w msg))
    (cl:cons ':disturbance_by_w (disturbance_by_w msg))
    (cl:cons ':disturbance_bx_b (disturbance_bx_b msg))
    (cl:cons ':disturbance_by_b (disturbance_by_b msg))
    (cl:cons ':controller_enforcing_constraints (controller_enforcing_constraints msg))
    (cl:cons ':horizontal_speed_constraint (horizontal_speed_constraint msg))
    (cl:cons ':horizontal_acc_constraint (horizontal_acc_constraint msg))
    (cl:cons ':vertical_asc_speed_constraint (vertical_asc_speed_constraint msg))
    (cl:cons ':vertical_asc_acc_constraint (vertical_asc_acc_constraint msg))
    (cl:cons ':vertical_desc_speed_constraint (vertical_desc_speed_constraint msg))
    (cl:cons ':vertical_desc_acc_constraint (vertical_desc_acc_constraint msg))
    (cl:cons ':mode_mask (mode_mask msg))
))
