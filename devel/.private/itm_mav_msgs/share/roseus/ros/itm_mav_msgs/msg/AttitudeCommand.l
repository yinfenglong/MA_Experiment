;; Auto-generated. Do not edit!


(when (boundp 'itm_mav_msgs::AttitudeCommand)
  (if (not (find-package "ITM_MAV_MSGS"))
    (make-package "ITM_MAV_MSGS"))
  (shadow 'AttitudeCommand (find-package "ITM_MAV_MSGS")))
(unless (find-package "ITM_MAV_MSGS::ATTITUDECOMMAND")
  (make-package "ITM_MAV_MSGS::ATTITUDECOMMAND"))

(in-package "ROS")
;;//! \htmlinclude AttitudeCommand.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(intern "*MODE_ATTITUDE*" (find-package "ITM_MAV_MSGS::ATTITUDECOMMAND"))
(shadow '*MODE_ATTITUDE* (find-package "ITM_MAV_MSGS::ATTITUDECOMMAND"))
(defconstant itm_mav_msgs::AttitudeCommand::*MODE_ATTITUDE* 1)
(intern "*MODE_ATTITUDE_RATE*" (find-package "ITM_MAV_MSGS::ATTITUDECOMMAND"))
(shadow '*MODE_ATTITUDE_RATE* (find-package "ITM_MAV_MSGS::ATTITUDECOMMAND"))
(defconstant itm_mav_msgs::AttitudeCommand::*MODE_ATTITUDE_RATE* 2)
(defclass itm_mav_msgs::AttitudeCommand
  :super ros::object
  :slots (_header _controller _ramping_up _attitude _attitude_rate _desired_acceleration _thrust _mass_difference _total_mass _disturbance_wx_w _disturbance_wy_w _disturbance_bx_w _disturbance_by_w _disturbance_bx_b _disturbance_by_b _controller_enforcing_constraints _horizontal_speed_constraint _horizontal_acc_constraint _vertical_asc_speed_constraint _vertical_asc_acc_constraint _vertical_desc_speed_constraint _vertical_desc_acc_constraint _mode_mask ))

(defmethod itm_mav_msgs::AttitudeCommand
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:controller __controller) "")
    ((:ramping_up __ramping_up) nil)
    ((:attitude __attitude) (instance geometry_msgs::Quaternion :init))
    ((:attitude_rate __attitude_rate) (instance geometry_msgs::Point :init))
    ((:desired_acceleration __desired_acceleration) (instance geometry_msgs::Point :init))
    ((:thrust __thrust) 0.0)
    ((:mass_difference __mass_difference) 0.0)
    ((:total_mass __total_mass) 0.0)
    ((:disturbance_wx_w __disturbance_wx_w) 0.0)
    ((:disturbance_wy_w __disturbance_wy_w) 0.0)
    ((:disturbance_bx_w __disturbance_bx_w) 0.0)
    ((:disturbance_by_w __disturbance_by_w) 0.0)
    ((:disturbance_bx_b __disturbance_bx_b) 0.0)
    ((:disturbance_by_b __disturbance_by_b) 0.0)
    ((:controller_enforcing_constraints __controller_enforcing_constraints) nil)
    ((:horizontal_speed_constraint __horizontal_speed_constraint) 0.0)
    ((:horizontal_acc_constraint __horizontal_acc_constraint) 0.0)
    ((:vertical_asc_speed_constraint __vertical_asc_speed_constraint) 0.0)
    ((:vertical_asc_acc_constraint __vertical_asc_acc_constraint) 0.0)
    ((:vertical_desc_speed_constraint __vertical_desc_speed_constraint) 0.0)
    ((:vertical_desc_acc_constraint __vertical_desc_acc_constraint) 0.0)
    ((:mode_mask __mode_mask) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _controller (string __controller))
   (setq _ramping_up __ramping_up)
   (setq _attitude __attitude)
   (setq _attitude_rate __attitude_rate)
   (setq _desired_acceleration __desired_acceleration)
   (setq _thrust (float __thrust))
   (setq _mass_difference (float __mass_difference))
   (setq _total_mass (float __total_mass))
   (setq _disturbance_wx_w (float __disturbance_wx_w))
   (setq _disturbance_wy_w (float __disturbance_wy_w))
   (setq _disturbance_bx_w (float __disturbance_bx_w))
   (setq _disturbance_by_w (float __disturbance_by_w))
   (setq _disturbance_bx_b (float __disturbance_bx_b))
   (setq _disturbance_by_b (float __disturbance_by_b))
   (setq _controller_enforcing_constraints __controller_enforcing_constraints)
   (setq _horizontal_speed_constraint (float __horizontal_speed_constraint))
   (setq _horizontal_acc_constraint (float __horizontal_acc_constraint))
   (setq _vertical_asc_speed_constraint (float __vertical_asc_speed_constraint))
   (setq _vertical_asc_acc_constraint (float __vertical_asc_acc_constraint))
   (setq _vertical_desc_speed_constraint (float __vertical_desc_speed_constraint))
   (setq _vertical_desc_acc_constraint (float __vertical_desc_acc_constraint))
   (setq _mode_mask (round __mode_mask))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:controller
   (&optional __controller)
   (if __controller (setq _controller __controller)) _controller)
  (:ramping_up
   (&optional __ramping_up)
   (if __ramping_up (setq _ramping_up __ramping_up)) _ramping_up)
  (:attitude
   (&rest __attitude)
   (if (keywordp (car __attitude))
       (send* _attitude __attitude)
     (progn
       (if __attitude (setq _attitude (car __attitude)))
       _attitude)))
  (:attitude_rate
   (&rest __attitude_rate)
   (if (keywordp (car __attitude_rate))
       (send* _attitude_rate __attitude_rate)
     (progn
       (if __attitude_rate (setq _attitude_rate (car __attitude_rate)))
       _attitude_rate)))
  (:desired_acceleration
   (&rest __desired_acceleration)
   (if (keywordp (car __desired_acceleration))
       (send* _desired_acceleration __desired_acceleration)
     (progn
       (if __desired_acceleration (setq _desired_acceleration (car __desired_acceleration)))
       _desired_acceleration)))
  (:thrust
   (&optional __thrust)
   (if __thrust (setq _thrust __thrust)) _thrust)
  (:mass_difference
   (&optional __mass_difference)
   (if __mass_difference (setq _mass_difference __mass_difference)) _mass_difference)
  (:total_mass
   (&optional __total_mass)
   (if __total_mass (setq _total_mass __total_mass)) _total_mass)
  (:disturbance_wx_w
   (&optional __disturbance_wx_w)
   (if __disturbance_wx_w (setq _disturbance_wx_w __disturbance_wx_w)) _disturbance_wx_w)
  (:disturbance_wy_w
   (&optional __disturbance_wy_w)
   (if __disturbance_wy_w (setq _disturbance_wy_w __disturbance_wy_w)) _disturbance_wy_w)
  (:disturbance_bx_w
   (&optional __disturbance_bx_w)
   (if __disturbance_bx_w (setq _disturbance_bx_w __disturbance_bx_w)) _disturbance_bx_w)
  (:disturbance_by_w
   (&optional __disturbance_by_w)
   (if __disturbance_by_w (setq _disturbance_by_w __disturbance_by_w)) _disturbance_by_w)
  (:disturbance_bx_b
   (&optional __disturbance_bx_b)
   (if __disturbance_bx_b (setq _disturbance_bx_b __disturbance_bx_b)) _disturbance_bx_b)
  (:disturbance_by_b
   (&optional __disturbance_by_b)
   (if __disturbance_by_b (setq _disturbance_by_b __disturbance_by_b)) _disturbance_by_b)
  (:controller_enforcing_constraints
   (&optional __controller_enforcing_constraints)
   (if __controller_enforcing_constraints (setq _controller_enforcing_constraints __controller_enforcing_constraints)) _controller_enforcing_constraints)
  (:horizontal_speed_constraint
   (&optional __horizontal_speed_constraint)
   (if __horizontal_speed_constraint (setq _horizontal_speed_constraint __horizontal_speed_constraint)) _horizontal_speed_constraint)
  (:horizontal_acc_constraint
   (&optional __horizontal_acc_constraint)
   (if __horizontal_acc_constraint (setq _horizontal_acc_constraint __horizontal_acc_constraint)) _horizontal_acc_constraint)
  (:vertical_asc_speed_constraint
   (&optional __vertical_asc_speed_constraint)
   (if __vertical_asc_speed_constraint (setq _vertical_asc_speed_constraint __vertical_asc_speed_constraint)) _vertical_asc_speed_constraint)
  (:vertical_asc_acc_constraint
   (&optional __vertical_asc_acc_constraint)
   (if __vertical_asc_acc_constraint (setq _vertical_asc_acc_constraint __vertical_asc_acc_constraint)) _vertical_asc_acc_constraint)
  (:vertical_desc_speed_constraint
   (&optional __vertical_desc_speed_constraint)
   (if __vertical_desc_speed_constraint (setq _vertical_desc_speed_constraint __vertical_desc_speed_constraint)) _vertical_desc_speed_constraint)
  (:vertical_desc_acc_constraint
   (&optional __vertical_desc_acc_constraint)
   (if __vertical_desc_acc_constraint (setq _vertical_desc_acc_constraint __vertical_desc_acc_constraint)) _vertical_desc_acc_constraint)
  (:mode_mask
   (&optional __mode_mask)
   (if __mode_mask (setq _mode_mask __mode_mask)) _mode_mask)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; string _controller
    4 (length _controller)
    ;; bool _ramping_up
    1
    ;; geometry_msgs/Quaternion _attitude
    (send _attitude :serialization-length)
    ;; geometry_msgs/Point _attitude_rate
    (send _attitude_rate :serialization-length)
    ;; geometry_msgs/Point _desired_acceleration
    (send _desired_acceleration :serialization-length)
    ;; float64 _thrust
    8
    ;; float64 _mass_difference
    8
    ;; float64 _total_mass
    8
    ;; float64 _disturbance_wx_w
    8
    ;; float64 _disturbance_wy_w
    8
    ;; float64 _disturbance_bx_w
    8
    ;; float64 _disturbance_by_w
    8
    ;; float64 _disturbance_bx_b
    8
    ;; float64 _disturbance_by_b
    8
    ;; bool _controller_enforcing_constraints
    1
    ;; float64 _horizontal_speed_constraint
    8
    ;; float64 _horizontal_acc_constraint
    8
    ;; float64 _vertical_asc_speed_constraint
    8
    ;; float64 _vertical_asc_acc_constraint
    8
    ;; float64 _vertical_desc_speed_constraint
    8
    ;; float64 _vertical_desc_acc_constraint
    8
    ;; uint8 _mode_mask
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; string _controller
       (write-long (length _controller) s) (princ _controller s)
     ;; bool _ramping_up
       (if _ramping_up (write-byte -1 s) (write-byte 0 s))
     ;; geometry_msgs/Quaternion _attitude
       (send _attitude :serialize s)
     ;; geometry_msgs/Point _attitude_rate
       (send _attitude_rate :serialize s)
     ;; geometry_msgs/Point _desired_acceleration
       (send _desired_acceleration :serialize s)
     ;; float64 _thrust
       (sys::poke _thrust (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _mass_difference
       (sys::poke _mass_difference (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _total_mass
       (sys::poke _total_mass (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _disturbance_wx_w
       (sys::poke _disturbance_wx_w (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _disturbance_wy_w
       (sys::poke _disturbance_wy_w (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _disturbance_bx_w
       (sys::poke _disturbance_bx_w (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _disturbance_by_w
       (sys::poke _disturbance_by_w (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _disturbance_bx_b
       (sys::poke _disturbance_bx_b (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _disturbance_by_b
       (sys::poke _disturbance_by_b (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; bool _controller_enforcing_constraints
       (if _controller_enforcing_constraints (write-byte -1 s) (write-byte 0 s))
     ;; float64 _horizontal_speed_constraint
       (sys::poke _horizontal_speed_constraint (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _horizontal_acc_constraint
       (sys::poke _horizontal_acc_constraint (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _vertical_asc_speed_constraint
       (sys::poke _vertical_asc_speed_constraint (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _vertical_asc_acc_constraint
       (sys::poke _vertical_asc_acc_constraint (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _vertical_desc_speed_constraint
       (sys::poke _vertical_desc_speed_constraint (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _vertical_desc_acc_constraint
       (sys::poke _vertical_desc_acc_constraint (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; uint8 _mode_mask
       (write-byte _mode_mask s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; string _controller
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _controller (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; bool _ramping_up
     (setq _ramping_up (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; geometry_msgs/Quaternion _attitude
     (send _attitude :deserialize buf ptr-) (incf ptr- (send _attitude :serialization-length))
   ;; geometry_msgs/Point _attitude_rate
     (send _attitude_rate :deserialize buf ptr-) (incf ptr- (send _attitude_rate :serialization-length))
   ;; geometry_msgs/Point _desired_acceleration
     (send _desired_acceleration :deserialize buf ptr-) (incf ptr- (send _desired_acceleration :serialization-length))
   ;; float64 _thrust
     (setq _thrust (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _mass_difference
     (setq _mass_difference (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _total_mass
     (setq _total_mass (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _disturbance_wx_w
     (setq _disturbance_wx_w (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _disturbance_wy_w
     (setq _disturbance_wy_w (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _disturbance_bx_w
     (setq _disturbance_bx_w (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _disturbance_by_w
     (setq _disturbance_by_w (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _disturbance_bx_b
     (setq _disturbance_bx_b (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _disturbance_by_b
     (setq _disturbance_by_b (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; bool _controller_enforcing_constraints
     (setq _controller_enforcing_constraints (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; float64 _horizontal_speed_constraint
     (setq _horizontal_speed_constraint (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _horizontal_acc_constraint
     (setq _horizontal_acc_constraint (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _vertical_asc_speed_constraint
     (setq _vertical_asc_speed_constraint (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _vertical_asc_acc_constraint
     (setq _vertical_asc_acc_constraint (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _vertical_desc_speed_constraint
     (setq _vertical_desc_speed_constraint (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _vertical_desc_acc_constraint
     (setq _vertical_desc_acc_constraint (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; uint8 _mode_mask
     (setq _mode_mask (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(setf (get itm_mav_msgs::AttitudeCommand :md5sum-) "ba99a1fcefbbc4c8eb8328bcdd1d674c")
(setf (get itm_mav_msgs::AttitudeCommand :datatype-) "itm_mav_msgs/AttitudeCommand")
(setf (get itm_mav_msgs::AttitudeCommand :definition-)
      "# This represents an output of a UAV feedback controller (mrs_uav_manager::Controller).
# This message is returned from a controller by means of the update() function, called by the mrs_uav_manager::ControlManager.

# fork from CTU work

std_msgs/Header header

# The name of the controller (implementation-wise).
# Beware, multiple instances of the same controller can be running.
# The ControlManagerDiagnostics message contains the name given them
# by the ControlManager.
string controller

# True if the controller is in the initial rampup phase (just after activation).
bool ramping_up

# The desired orientation produced by the controller.
# This field should be filled every time.
geometry_msgs/Quaternion attitude

# The desired attitude rate produced by the controller.
# This field is optional.
geometry_msgs/Point attitude_rate

# Desired acceleration produced by the controller.
# This field is mandatory if flying with \"mrs_odometry\".
# The desired acceleration should be without calculate without
# compensation of external forces and disturbances, e.g., without
# the compensation for the gravity vector, wind and internal UAV biases.
geometry_msgs/Point desired_acceleration

# The desired thrust, [0, 1].
float64 thrust

# The estimated mass difference from the nominal mass.
float64 mass_difference

# Total estimated UAV mass.
float64 total_mass

# World-frame disturbances expressed in the world frame.
float64 disturbance_wx_w
float64 disturbance_wy_w

# Body-frame (fcu_untilted) disturbances expressed in the world frame.
float64 disturbance_bx_w
float64 disturbance_by_w

# Body-frame (fcu_untilted) disturbances expressed in the body frame (fcu_untilted).
float64 disturbance_bx_b
float64 disturbance_by_b

# The controller can enforce dynamics constraints over the trackers.
# This can be used when flying with a controller that is limited to
# some maximum speed and acceleration.
bool controller_enforcing_constraints
float64 horizontal_speed_constraint
float64 horizontal_acc_constraint
float64 vertical_asc_speed_constraint
float64 vertical_asc_acc_constraint
float64 vertical_desc_speed_constraint
float64 vertical_desc_acc_constraint

# Defines what output should be used, whether the attitude
# or the attitude rate.
uint8 mode_mask
uint8 MODE_ATTITUDE=1
uint8 MODE_ATTITUDE_RATE=2
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

")



(provide :itm_mav_msgs/AttitudeCommand "ba99a1fcefbbc4c8eb8328bcdd1d674c")


