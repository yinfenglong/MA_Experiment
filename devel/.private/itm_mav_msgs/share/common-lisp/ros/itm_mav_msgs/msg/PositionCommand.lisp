; Auto-generated. Do not edit!


(cl:in-package itm_mav_msgs-msg)


;//! \htmlinclude PositionCommand.msg.html

(cl:defclass <PositionCommand> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (position
    :reader position
    :initarg :position
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point))
   (velocity
    :reader velocity
    :initarg :velocity
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (acceleration
    :reader acceleration
    :initarg :acceleration
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (jerk
    :reader jerk
    :initarg :jerk
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (snap
    :reader snap
    :initarg :snap
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (orientation
    :reader orientation
    :initarg :orientation
    :type geometry_msgs-msg:Quaternion
    :initform (cl:make-instance 'geometry_msgs-msg:Quaternion))
   (attitude_rate
    :reader attitude_rate
    :initarg :attitude_rate
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point))
   (thrust
    :reader thrust
    :initarg :thrust
    :type cl:float
    :initform 0.0)
   (heading
    :reader heading
    :initarg :heading
    :type cl:float
    :initform 0.0)
   (heading_rate
    :reader heading_rate
    :initarg :heading_rate
    :type cl:float
    :initform 0.0)
   (heading_acceleration
    :reader heading_acceleration
    :initarg :heading_acceleration
    :type cl:float
    :initform 0.0)
   (heading_jerk
    :reader heading_jerk
    :initarg :heading_jerk
    :type cl:float
    :initform 0.0)
   (disable_position_gains
    :reader disable_position_gains
    :initarg :disable_position_gains
    :type cl:boolean
    :initform cl:nil)
   (disable_antiwindups
    :reader disable_antiwindups
    :initarg :disable_antiwindups
    :type cl:boolean
    :initform cl:nil)
   (use_position_horizontal
    :reader use_position_horizontal
    :initarg :use_position_horizontal
    :type cl:fixnum
    :initform 0)
   (use_position_vertical
    :reader use_position_vertical
    :initarg :use_position_vertical
    :type cl:fixnum
    :initform 0)
   (use_velocity_horizontal
    :reader use_velocity_horizontal
    :initarg :use_velocity_horizontal
    :type cl:fixnum
    :initform 0)
   (use_velocity_vertical
    :reader use_velocity_vertical
    :initarg :use_velocity_vertical
    :type cl:fixnum
    :initform 0)
   (use_acceleration
    :reader use_acceleration
    :initarg :use_acceleration
    :type cl:fixnum
    :initform 0)
   (use_jerk
    :reader use_jerk
    :initarg :use_jerk
    :type cl:fixnum
    :initform 0)
   (use_snap
    :reader use_snap
    :initarg :use_snap
    :type cl:fixnum
    :initform 0)
   (use_attitude_rate
    :reader use_attitude_rate
    :initarg :use_attitude_rate
    :type cl:fixnum
    :initform 0)
   (use_heading
    :reader use_heading
    :initarg :use_heading
    :type cl:fixnum
    :initform 0)
   (use_heading_rate
    :reader use_heading_rate
    :initarg :use_heading_rate
    :type cl:fixnum
    :initform 0)
   (use_heading_acceleration
    :reader use_heading_acceleration
    :initarg :use_heading_acceleration
    :type cl:fixnum
    :initform 0)
   (use_heading_jerk
    :reader use_heading_jerk
    :initarg :use_heading_jerk
    :type cl:fixnum
    :initform 0)
   (use_orientation
    :reader use_orientation
    :initarg :use_orientation
    :type cl:fixnum
    :initform 0)
   (use_thrust
    :reader use_thrust
    :initarg :use_thrust
    :type cl:fixnum
    :initform 0))
)

(cl:defclass PositionCommand (<PositionCommand>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PositionCommand>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PositionCommand)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name itm_mav_msgs-msg:<PositionCommand> is deprecated: use itm_mav_msgs-msg:PositionCommand instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <PositionCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader itm_mav_msgs-msg:header-val is deprecated.  Use itm_mav_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'position-val :lambda-list '(m))
(cl:defmethod position-val ((m <PositionCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader itm_mav_msgs-msg:position-val is deprecated.  Use itm_mav_msgs-msg:position instead.")
  (position m))

(cl:ensure-generic-function 'velocity-val :lambda-list '(m))
(cl:defmethod velocity-val ((m <PositionCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader itm_mav_msgs-msg:velocity-val is deprecated.  Use itm_mav_msgs-msg:velocity instead.")
  (velocity m))

(cl:ensure-generic-function 'acceleration-val :lambda-list '(m))
(cl:defmethod acceleration-val ((m <PositionCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader itm_mav_msgs-msg:acceleration-val is deprecated.  Use itm_mav_msgs-msg:acceleration instead.")
  (acceleration m))

(cl:ensure-generic-function 'jerk-val :lambda-list '(m))
(cl:defmethod jerk-val ((m <PositionCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader itm_mav_msgs-msg:jerk-val is deprecated.  Use itm_mav_msgs-msg:jerk instead.")
  (jerk m))

(cl:ensure-generic-function 'snap-val :lambda-list '(m))
(cl:defmethod snap-val ((m <PositionCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader itm_mav_msgs-msg:snap-val is deprecated.  Use itm_mav_msgs-msg:snap instead.")
  (snap m))

(cl:ensure-generic-function 'orientation-val :lambda-list '(m))
(cl:defmethod orientation-val ((m <PositionCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader itm_mav_msgs-msg:orientation-val is deprecated.  Use itm_mav_msgs-msg:orientation instead.")
  (orientation m))

(cl:ensure-generic-function 'attitude_rate-val :lambda-list '(m))
(cl:defmethod attitude_rate-val ((m <PositionCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader itm_mav_msgs-msg:attitude_rate-val is deprecated.  Use itm_mav_msgs-msg:attitude_rate instead.")
  (attitude_rate m))

(cl:ensure-generic-function 'thrust-val :lambda-list '(m))
(cl:defmethod thrust-val ((m <PositionCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader itm_mav_msgs-msg:thrust-val is deprecated.  Use itm_mav_msgs-msg:thrust instead.")
  (thrust m))

(cl:ensure-generic-function 'heading-val :lambda-list '(m))
(cl:defmethod heading-val ((m <PositionCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader itm_mav_msgs-msg:heading-val is deprecated.  Use itm_mav_msgs-msg:heading instead.")
  (heading m))

(cl:ensure-generic-function 'heading_rate-val :lambda-list '(m))
(cl:defmethod heading_rate-val ((m <PositionCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader itm_mav_msgs-msg:heading_rate-val is deprecated.  Use itm_mav_msgs-msg:heading_rate instead.")
  (heading_rate m))

(cl:ensure-generic-function 'heading_acceleration-val :lambda-list '(m))
(cl:defmethod heading_acceleration-val ((m <PositionCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader itm_mav_msgs-msg:heading_acceleration-val is deprecated.  Use itm_mav_msgs-msg:heading_acceleration instead.")
  (heading_acceleration m))

(cl:ensure-generic-function 'heading_jerk-val :lambda-list '(m))
(cl:defmethod heading_jerk-val ((m <PositionCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader itm_mav_msgs-msg:heading_jerk-val is deprecated.  Use itm_mav_msgs-msg:heading_jerk instead.")
  (heading_jerk m))

(cl:ensure-generic-function 'disable_position_gains-val :lambda-list '(m))
(cl:defmethod disable_position_gains-val ((m <PositionCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader itm_mav_msgs-msg:disable_position_gains-val is deprecated.  Use itm_mav_msgs-msg:disable_position_gains instead.")
  (disable_position_gains m))

(cl:ensure-generic-function 'disable_antiwindups-val :lambda-list '(m))
(cl:defmethod disable_antiwindups-val ((m <PositionCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader itm_mav_msgs-msg:disable_antiwindups-val is deprecated.  Use itm_mav_msgs-msg:disable_antiwindups instead.")
  (disable_antiwindups m))

(cl:ensure-generic-function 'use_position_horizontal-val :lambda-list '(m))
(cl:defmethod use_position_horizontal-val ((m <PositionCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader itm_mav_msgs-msg:use_position_horizontal-val is deprecated.  Use itm_mav_msgs-msg:use_position_horizontal instead.")
  (use_position_horizontal m))

(cl:ensure-generic-function 'use_position_vertical-val :lambda-list '(m))
(cl:defmethod use_position_vertical-val ((m <PositionCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader itm_mav_msgs-msg:use_position_vertical-val is deprecated.  Use itm_mav_msgs-msg:use_position_vertical instead.")
  (use_position_vertical m))

(cl:ensure-generic-function 'use_velocity_horizontal-val :lambda-list '(m))
(cl:defmethod use_velocity_horizontal-val ((m <PositionCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader itm_mav_msgs-msg:use_velocity_horizontal-val is deprecated.  Use itm_mav_msgs-msg:use_velocity_horizontal instead.")
  (use_velocity_horizontal m))

(cl:ensure-generic-function 'use_velocity_vertical-val :lambda-list '(m))
(cl:defmethod use_velocity_vertical-val ((m <PositionCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader itm_mav_msgs-msg:use_velocity_vertical-val is deprecated.  Use itm_mav_msgs-msg:use_velocity_vertical instead.")
  (use_velocity_vertical m))

(cl:ensure-generic-function 'use_acceleration-val :lambda-list '(m))
(cl:defmethod use_acceleration-val ((m <PositionCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader itm_mav_msgs-msg:use_acceleration-val is deprecated.  Use itm_mav_msgs-msg:use_acceleration instead.")
  (use_acceleration m))

(cl:ensure-generic-function 'use_jerk-val :lambda-list '(m))
(cl:defmethod use_jerk-val ((m <PositionCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader itm_mav_msgs-msg:use_jerk-val is deprecated.  Use itm_mav_msgs-msg:use_jerk instead.")
  (use_jerk m))

(cl:ensure-generic-function 'use_snap-val :lambda-list '(m))
(cl:defmethod use_snap-val ((m <PositionCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader itm_mav_msgs-msg:use_snap-val is deprecated.  Use itm_mav_msgs-msg:use_snap instead.")
  (use_snap m))

(cl:ensure-generic-function 'use_attitude_rate-val :lambda-list '(m))
(cl:defmethod use_attitude_rate-val ((m <PositionCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader itm_mav_msgs-msg:use_attitude_rate-val is deprecated.  Use itm_mav_msgs-msg:use_attitude_rate instead.")
  (use_attitude_rate m))

(cl:ensure-generic-function 'use_heading-val :lambda-list '(m))
(cl:defmethod use_heading-val ((m <PositionCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader itm_mav_msgs-msg:use_heading-val is deprecated.  Use itm_mav_msgs-msg:use_heading instead.")
  (use_heading m))

(cl:ensure-generic-function 'use_heading_rate-val :lambda-list '(m))
(cl:defmethod use_heading_rate-val ((m <PositionCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader itm_mav_msgs-msg:use_heading_rate-val is deprecated.  Use itm_mav_msgs-msg:use_heading_rate instead.")
  (use_heading_rate m))

(cl:ensure-generic-function 'use_heading_acceleration-val :lambda-list '(m))
(cl:defmethod use_heading_acceleration-val ((m <PositionCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader itm_mav_msgs-msg:use_heading_acceleration-val is deprecated.  Use itm_mav_msgs-msg:use_heading_acceleration instead.")
  (use_heading_acceleration m))

(cl:ensure-generic-function 'use_heading_jerk-val :lambda-list '(m))
(cl:defmethod use_heading_jerk-val ((m <PositionCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader itm_mav_msgs-msg:use_heading_jerk-val is deprecated.  Use itm_mav_msgs-msg:use_heading_jerk instead.")
  (use_heading_jerk m))

(cl:ensure-generic-function 'use_orientation-val :lambda-list '(m))
(cl:defmethod use_orientation-val ((m <PositionCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader itm_mav_msgs-msg:use_orientation-val is deprecated.  Use itm_mav_msgs-msg:use_orientation instead.")
  (use_orientation m))

(cl:ensure-generic-function 'use_thrust-val :lambda-list '(m))
(cl:defmethod use_thrust-val ((m <PositionCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader itm_mav_msgs-msg:use_thrust-val is deprecated.  Use itm_mav_msgs-msg:use_thrust instead.")
  (use_thrust m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PositionCommand>) ostream)
  "Serializes a message object of type '<PositionCommand>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'position) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'velocity) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'acceleration) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'jerk) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'snap) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'orientation) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'attitude_rate) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'thrust))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'heading))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'heading_rate))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'heading_acceleration))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'heading_jerk))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'disable_position_gains) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'disable_antiwindups) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'use_position_horizontal)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'use_position_vertical)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'use_velocity_horizontal)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'use_velocity_vertical)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'use_acceleration)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'use_jerk)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'use_snap)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'use_attitude_rate)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'use_heading)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'use_heading_rate)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'use_heading_acceleration)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'use_heading_jerk)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'use_orientation)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'use_thrust)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PositionCommand>) istream)
  "Deserializes a message object of type '<PositionCommand>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'position) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'velocity) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'acceleration) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'jerk) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'snap) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'orientation) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'attitude_rate) istream)
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
    (cl:setf (cl:slot-value msg 'heading) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'heading_rate) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'heading_acceleration) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'heading_jerk) (roslisp-utils:decode-double-float-bits bits)))
    (cl:setf (cl:slot-value msg 'disable_position_gains) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'disable_antiwindups) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'use_position_horizontal)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'use_position_vertical)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'use_velocity_horizontal)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'use_velocity_vertical)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'use_acceleration)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'use_jerk)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'use_snap)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'use_attitude_rate)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'use_heading)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'use_heading_rate)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'use_heading_acceleration)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'use_heading_jerk)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'use_orientation)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'use_thrust)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PositionCommand>)))
  "Returns string type for a message object of type '<PositionCommand>"
  "itm_mav_msgs/PositionCommand")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PositionCommand)))
  "Returns string type for a message object of type 'PositionCommand"
  "itm_mav_msgs/PositionCommand")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PositionCommand>)))
  "Returns md5sum for a message object of type '<PositionCommand>"
  "4d8e95e3ee792c1a5ce3afe2d9f2396a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PositionCommand)))
  "Returns md5sum for a message object of type 'PositionCommand"
  "4d8e95e3ee792c1a5ce3afe2d9f2396a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PositionCommand>)))
  "Returns full string definition for message of type '<PositionCommand>"
  (cl:format cl:nil "# This represents the output of the currently active Tracker (mrs_uav_manager::Tracker).~%# This message is returned from a tracker by means of the update() function, called by the mrs_uav_manager::ControlManager.~%~%std_msgs/Header header~%~%# The desired position.~%geometry_msgs/Point position~%~%# The desired velocity.~%geometry_msgs/Vector3 velocity~%~%# The desired acceleration.~%geometry_msgs/Vector3 acceleration~%~%# The desired jerk.~%geometry_msgs/Vector3 jerk~%~%# The desired snap.~%geometry_msgs/Vector3 snap~%~%# The desired orientation and attitude rate.~%# It is mutually exclusive to heading+heading_rate.~%geometry_msgs/Quaternion orientation~%geometry_msgs/Point attitude_rate~%~%# when used, it will override the thrust output of the controller~%float64 thrust~%~%# The desired heading and heading rate.~%# It is mutually exclusive to orientation+attitude_rate.~%float64 heading~%float64 heading_rate~%float64 heading_acceleration~%float64 heading_jerk~%~%# Whether the controller should mute its position feedback.~%# Useful, e.g., during takeoff or in situations where sudden control~%# error is expected but not immediately required to be fixed.~%bool disable_position_gains~%~%# Whether the controller should disable its antiwindups~%bool disable_antiwindups~%~%# Flags defining whether the XY or Z position reference was filled in.~%# If not, the controllers should pay no attention to it and to the corresponding control error.~%uint8 use_position_horizontal~%uint8 use_position_vertical~%~%# Flags defining whether the XY or Z velocity reference was filled in.~%# If not, the controllers should pay no attention to it and to the corresponding control error.~%uint8 use_velocity_horizontal~%uint8 use_velocity_vertical~%~%# Flags defining which references were filled in.~%# If not, the controllers should pay no attention to them and to the corresponding control errors.~%uint8 use_acceleration~%uint8 use_jerk~%uint8 use_snap~%uint8 use_attitude_rate~%uint8 use_heading~%uint8 use_heading_rate~%uint8 use_heading_acceleration~%uint8 use_heading_jerk~%uint8 use_orientation~%uint8 use_thrust~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PositionCommand)))
  "Returns full string definition for message of type 'PositionCommand"
  (cl:format cl:nil "# This represents the output of the currently active Tracker (mrs_uav_manager::Tracker).~%# This message is returned from a tracker by means of the update() function, called by the mrs_uav_manager::ControlManager.~%~%std_msgs/Header header~%~%# The desired position.~%geometry_msgs/Point position~%~%# The desired velocity.~%geometry_msgs/Vector3 velocity~%~%# The desired acceleration.~%geometry_msgs/Vector3 acceleration~%~%# The desired jerk.~%geometry_msgs/Vector3 jerk~%~%# The desired snap.~%geometry_msgs/Vector3 snap~%~%# The desired orientation and attitude rate.~%# It is mutually exclusive to heading+heading_rate.~%geometry_msgs/Quaternion orientation~%geometry_msgs/Point attitude_rate~%~%# when used, it will override the thrust output of the controller~%float64 thrust~%~%# The desired heading and heading rate.~%# It is mutually exclusive to orientation+attitude_rate.~%float64 heading~%float64 heading_rate~%float64 heading_acceleration~%float64 heading_jerk~%~%# Whether the controller should mute its position feedback.~%# Useful, e.g., during takeoff or in situations where sudden control~%# error is expected but not immediately required to be fixed.~%bool disable_position_gains~%~%# Whether the controller should disable its antiwindups~%bool disable_antiwindups~%~%# Flags defining whether the XY or Z position reference was filled in.~%# If not, the controllers should pay no attention to it and to the corresponding control error.~%uint8 use_position_horizontal~%uint8 use_position_vertical~%~%# Flags defining whether the XY or Z velocity reference was filled in.~%# If not, the controllers should pay no attention to it and to the corresponding control error.~%uint8 use_velocity_horizontal~%uint8 use_velocity_vertical~%~%# Flags defining which references were filled in.~%# If not, the controllers should pay no attention to them and to the corresponding control errors.~%uint8 use_acceleration~%uint8 use_jerk~%uint8 use_snap~%uint8 use_attitude_rate~%uint8 use_heading~%uint8 use_heading_rate~%uint8 use_heading_acceleration~%uint8 use_heading_jerk~%uint8 use_orientation~%uint8 use_thrust~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PositionCommand>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'position))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'velocity))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'acceleration))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'jerk))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'snap))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'orientation))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'attitude_rate))
     8
     8
     8
     8
     8
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PositionCommand>))
  "Converts a ROS message object to a list"
  (cl:list 'PositionCommand
    (cl:cons ':header (header msg))
    (cl:cons ':position (position msg))
    (cl:cons ':velocity (velocity msg))
    (cl:cons ':acceleration (acceleration msg))
    (cl:cons ':jerk (jerk msg))
    (cl:cons ':snap (snap msg))
    (cl:cons ':orientation (orientation msg))
    (cl:cons ':attitude_rate (attitude_rate msg))
    (cl:cons ':thrust (thrust msg))
    (cl:cons ':heading (heading msg))
    (cl:cons ':heading_rate (heading_rate msg))
    (cl:cons ':heading_acceleration (heading_acceleration msg))
    (cl:cons ':heading_jerk (heading_jerk msg))
    (cl:cons ':disable_position_gains (disable_position_gains msg))
    (cl:cons ':disable_antiwindups (disable_antiwindups msg))
    (cl:cons ':use_position_horizontal (use_position_horizontal msg))
    (cl:cons ':use_position_vertical (use_position_vertical msg))
    (cl:cons ':use_velocity_horizontal (use_velocity_horizontal msg))
    (cl:cons ':use_velocity_vertical (use_velocity_vertical msg))
    (cl:cons ':use_acceleration (use_acceleration msg))
    (cl:cons ':use_jerk (use_jerk msg))
    (cl:cons ':use_snap (use_snap msg))
    (cl:cons ':use_attitude_rate (use_attitude_rate msg))
    (cl:cons ':use_heading (use_heading msg))
    (cl:cons ':use_heading_rate (use_heading_rate msg))
    (cl:cons ':use_heading_acceleration (use_heading_acceleration msg))
    (cl:cons ':use_heading_jerk (use_heading_jerk msg))
    (cl:cons ':use_orientation (use_orientation msg))
    (cl:cons ':use_thrust (use_thrust msg))
))
