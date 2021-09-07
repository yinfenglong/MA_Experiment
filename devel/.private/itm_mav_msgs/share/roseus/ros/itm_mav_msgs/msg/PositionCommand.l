;; Auto-generated. Do not edit!


(when (boundp 'itm_mav_msgs::PositionCommand)
  (if (not (find-package "ITM_MAV_MSGS"))
    (make-package "ITM_MAV_MSGS"))
  (shadow 'PositionCommand (find-package "ITM_MAV_MSGS")))
(unless (find-package "ITM_MAV_MSGS::POSITIONCOMMAND")
  (make-package "ITM_MAV_MSGS::POSITIONCOMMAND"))

(in-package "ROS")
;;//! \htmlinclude PositionCommand.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass itm_mav_msgs::PositionCommand
  :super ros::object
  :slots (_header _position _velocity _acceleration _jerk _snap _orientation _attitude_rate _thrust _heading _heading_rate _heading_acceleration _heading_jerk _disable_position_gains _disable_antiwindups _use_position_horizontal _use_position_vertical _use_velocity_horizontal _use_velocity_vertical _use_acceleration _use_jerk _use_snap _use_attitude_rate _use_heading _use_heading_rate _use_heading_acceleration _use_heading_jerk _use_orientation _use_thrust ))

(defmethod itm_mav_msgs::PositionCommand
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:position __position) (instance geometry_msgs::Point :init))
    ((:velocity __velocity) (instance geometry_msgs::Vector3 :init))
    ((:acceleration __acceleration) (instance geometry_msgs::Vector3 :init))
    ((:jerk __jerk) (instance geometry_msgs::Vector3 :init))
    ((:snap __snap) (instance geometry_msgs::Vector3 :init))
    ((:orientation __orientation) (instance geometry_msgs::Quaternion :init))
    ((:attitude_rate __attitude_rate) (instance geometry_msgs::Point :init))
    ((:thrust __thrust) 0.0)
    ((:heading __heading) 0.0)
    ((:heading_rate __heading_rate) 0.0)
    ((:heading_acceleration __heading_acceleration) 0.0)
    ((:heading_jerk __heading_jerk) 0.0)
    ((:disable_position_gains __disable_position_gains) nil)
    ((:disable_antiwindups __disable_antiwindups) nil)
    ((:use_position_horizontal __use_position_horizontal) 0)
    ((:use_position_vertical __use_position_vertical) 0)
    ((:use_velocity_horizontal __use_velocity_horizontal) 0)
    ((:use_velocity_vertical __use_velocity_vertical) 0)
    ((:use_acceleration __use_acceleration) 0)
    ((:use_jerk __use_jerk) 0)
    ((:use_snap __use_snap) 0)
    ((:use_attitude_rate __use_attitude_rate) 0)
    ((:use_heading __use_heading) 0)
    ((:use_heading_rate __use_heading_rate) 0)
    ((:use_heading_acceleration __use_heading_acceleration) 0)
    ((:use_heading_jerk __use_heading_jerk) 0)
    ((:use_orientation __use_orientation) 0)
    ((:use_thrust __use_thrust) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _position __position)
   (setq _velocity __velocity)
   (setq _acceleration __acceleration)
   (setq _jerk __jerk)
   (setq _snap __snap)
   (setq _orientation __orientation)
   (setq _attitude_rate __attitude_rate)
   (setq _thrust (float __thrust))
   (setq _heading (float __heading))
   (setq _heading_rate (float __heading_rate))
   (setq _heading_acceleration (float __heading_acceleration))
   (setq _heading_jerk (float __heading_jerk))
   (setq _disable_position_gains __disable_position_gains)
   (setq _disable_antiwindups __disable_antiwindups)
   (setq _use_position_horizontal (round __use_position_horizontal))
   (setq _use_position_vertical (round __use_position_vertical))
   (setq _use_velocity_horizontal (round __use_velocity_horizontal))
   (setq _use_velocity_vertical (round __use_velocity_vertical))
   (setq _use_acceleration (round __use_acceleration))
   (setq _use_jerk (round __use_jerk))
   (setq _use_snap (round __use_snap))
   (setq _use_attitude_rate (round __use_attitude_rate))
   (setq _use_heading (round __use_heading))
   (setq _use_heading_rate (round __use_heading_rate))
   (setq _use_heading_acceleration (round __use_heading_acceleration))
   (setq _use_heading_jerk (round __use_heading_jerk))
   (setq _use_orientation (round __use_orientation))
   (setq _use_thrust (round __use_thrust))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:position
   (&rest __position)
   (if (keywordp (car __position))
       (send* _position __position)
     (progn
       (if __position (setq _position (car __position)))
       _position)))
  (:velocity
   (&rest __velocity)
   (if (keywordp (car __velocity))
       (send* _velocity __velocity)
     (progn
       (if __velocity (setq _velocity (car __velocity)))
       _velocity)))
  (:acceleration
   (&rest __acceleration)
   (if (keywordp (car __acceleration))
       (send* _acceleration __acceleration)
     (progn
       (if __acceleration (setq _acceleration (car __acceleration)))
       _acceleration)))
  (:jerk
   (&rest __jerk)
   (if (keywordp (car __jerk))
       (send* _jerk __jerk)
     (progn
       (if __jerk (setq _jerk (car __jerk)))
       _jerk)))
  (:snap
   (&rest __snap)
   (if (keywordp (car __snap))
       (send* _snap __snap)
     (progn
       (if __snap (setq _snap (car __snap)))
       _snap)))
  (:orientation
   (&rest __orientation)
   (if (keywordp (car __orientation))
       (send* _orientation __orientation)
     (progn
       (if __orientation (setq _orientation (car __orientation)))
       _orientation)))
  (:attitude_rate
   (&rest __attitude_rate)
   (if (keywordp (car __attitude_rate))
       (send* _attitude_rate __attitude_rate)
     (progn
       (if __attitude_rate (setq _attitude_rate (car __attitude_rate)))
       _attitude_rate)))
  (:thrust
   (&optional __thrust)
   (if __thrust (setq _thrust __thrust)) _thrust)
  (:heading
   (&optional __heading)
   (if __heading (setq _heading __heading)) _heading)
  (:heading_rate
   (&optional __heading_rate)
   (if __heading_rate (setq _heading_rate __heading_rate)) _heading_rate)
  (:heading_acceleration
   (&optional __heading_acceleration)
   (if __heading_acceleration (setq _heading_acceleration __heading_acceleration)) _heading_acceleration)
  (:heading_jerk
   (&optional __heading_jerk)
   (if __heading_jerk (setq _heading_jerk __heading_jerk)) _heading_jerk)
  (:disable_position_gains
   (&optional __disable_position_gains)
   (if __disable_position_gains (setq _disable_position_gains __disable_position_gains)) _disable_position_gains)
  (:disable_antiwindups
   (&optional __disable_antiwindups)
   (if __disable_antiwindups (setq _disable_antiwindups __disable_antiwindups)) _disable_antiwindups)
  (:use_position_horizontal
   (&optional __use_position_horizontal)
   (if __use_position_horizontal (setq _use_position_horizontal __use_position_horizontal)) _use_position_horizontal)
  (:use_position_vertical
   (&optional __use_position_vertical)
   (if __use_position_vertical (setq _use_position_vertical __use_position_vertical)) _use_position_vertical)
  (:use_velocity_horizontal
   (&optional __use_velocity_horizontal)
   (if __use_velocity_horizontal (setq _use_velocity_horizontal __use_velocity_horizontal)) _use_velocity_horizontal)
  (:use_velocity_vertical
   (&optional __use_velocity_vertical)
   (if __use_velocity_vertical (setq _use_velocity_vertical __use_velocity_vertical)) _use_velocity_vertical)
  (:use_acceleration
   (&optional __use_acceleration)
   (if __use_acceleration (setq _use_acceleration __use_acceleration)) _use_acceleration)
  (:use_jerk
   (&optional __use_jerk)
   (if __use_jerk (setq _use_jerk __use_jerk)) _use_jerk)
  (:use_snap
   (&optional __use_snap)
   (if __use_snap (setq _use_snap __use_snap)) _use_snap)
  (:use_attitude_rate
   (&optional __use_attitude_rate)
   (if __use_attitude_rate (setq _use_attitude_rate __use_attitude_rate)) _use_attitude_rate)
  (:use_heading
   (&optional __use_heading)
   (if __use_heading (setq _use_heading __use_heading)) _use_heading)
  (:use_heading_rate
   (&optional __use_heading_rate)
   (if __use_heading_rate (setq _use_heading_rate __use_heading_rate)) _use_heading_rate)
  (:use_heading_acceleration
   (&optional __use_heading_acceleration)
   (if __use_heading_acceleration (setq _use_heading_acceleration __use_heading_acceleration)) _use_heading_acceleration)
  (:use_heading_jerk
   (&optional __use_heading_jerk)
   (if __use_heading_jerk (setq _use_heading_jerk __use_heading_jerk)) _use_heading_jerk)
  (:use_orientation
   (&optional __use_orientation)
   (if __use_orientation (setq _use_orientation __use_orientation)) _use_orientation)
  (:use_thrust
   (&optional __use_thrust)
   (if __use_thrust (setq _use_thrust __use_thrust)) _use_thrust)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; geometry_msgs/Point _position
    (send _position :serialization-length)
    ;; geometry_msgs/Vector3 _velocity
    (send _velocity :serialization-length)
    ;; geometry_msgs/Vector3 _acceleration
    (send _acceleration :serialization-length)
    ;; geometry_msgs/Vector3 _jerk
    (send _jerk :serialization-length)
    ;; geometry_msgs/Vector3 _snap
    (send _snap :serialization-length)
    ;; geometry_msgs/Quaternion _orientation
    (send _orientation :serialization-length)
    ;; geometry_msgs/Point _attitude_rate
    (send _attitude_rate :serialization-length)
    ;; float64 _thrust
    8
    ;; float64 _heading
    8
    ;; float64 _heading_rate
    8
    ;; float64 _heading_acceleration
    8
    ;; float64 _heading_jerk
    8
    ;; bool _disable_position_gains
    1
    ;; bool _disable_antiwindups
    1
    ;; uint8 _use_position_horizontal
    1
    ;; uint8 _use_position_vertical
    1
    ;; uint8 _use_velocity_horizontal
    1
    ;; uint8 _use_velocity_vertical
    1
    ;; uint8 _use_acceleration
    1
    ;; uint8 _use_jerk
    1
    ;; uint8 _use_snap
    1
    ;; uint8 _use_attitude_rate
    1
    ;; uint8 _use_heading
    1
    ;; uint8 _use_heading_rate
    1
    ;; uint8 _use_heading_acceleration
    1
    ;; uint8 _use_heading_jerk
    1
    ;; uint8 _use_orientation
    1
    ;; uint8 _use_thrust
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; geometry_msgs/Point _position
       (send _position :serialize s)
     ;; geometry_msgs/Vector3 _velocity
       (send _velocity :serialize s)
     ;; geometry_msgs/Vector3 _acceleration
       (send _acceleration :serialize s)
     ;; geometry_msgs/Vector3 _jerk
       (send _jerk :serialize s)
     ;; geometry_msgs/Vector3 _snap
       (send _snap :serialize s)
     ;; geometry_msgs/Quaternion _orientation
       (send _orientation :serialize s)
     ;; geometry_msgs/Point _attitude_rate
       (send _attitude_rate :serialize s)
     ;; float64 _thrust
       (sys::poke _thrust (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _heading
       (sys::poke _heading (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _heading_rate
       (sys::poke _heading_rate (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _heading_acceleration
       (sys::poke _heading_acceleration (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _heading_jerk
       (sys::poke _heading_jerk (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; bool _disable_position_gains
       (if _disable_position_gains (write-byte -1 s) (write-byte 0 s))
     ;; bool _disable_antiwindups
       (if _disable_antiwindups (write-byte -1 s) (write-byte 0 s))
     ;; uint8 _use_position_horizontal
       (write-byte _use_position_horizontal s)
     ;; uint8 _use_position_vertical
       (write-byte _use_position_vertical s)
     ;; uint8 _use_velocity_horizontal
       (write-byte _use_velocity_horizontal s)
     ;; uint8 _use_velocity_vertical
       (write-byte _use_velocity_vertical s)
     ;; uint8 _use_acceleration
       (write-byte _use_acceleration s)
     ;; uint8 _use_jerk
       (write-byte _use_jerk s)
     ;; uint8 _use_snap
       (write-byte _use_snap s)
     ;; uint8 _use_attitude_rate
       (write-byte _use_attitude_rate s)
     ;; uint8 _use_heading
       (write-byte _use_heading s)
     ;; uint8 _use_heading_rate
       (write-byte _use_heading_rate s)
     ;; uint8 _use_heading_acceleration
       (write-byte _use_heading_acceleration s)
     ;; uint8 _use_heading_jerk
       (write-byte _use_heading_jerk s)
     ;; uint8 _use_orientation
       (write-byte _use_orientation s)
     ;; uint8 _use_thrust
       (write-byte _use_thrust s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; geometry_msgs/Point _position
     (send _position :deserialize buf ptr-) (incf ptr- (send _position :serialization-length))
   ;; geometry_msgs/Vector3 _velocity
     (send _velocity :deserialize buf ptr-) (incf ptr- (send _velocity :serialization-length))
   ;; geometry_msgs/Vector3 _acceleration
     (send _acceleration :deserialize buf ptr-) (incf ptr- (send _acceleration :serialization-length))
   ;; geometry_msgs/Vector3 _jerk
     (send _jerk :deserialize buf ptr-) (incf ptr- (send _jerk :serialization-length))
   ;; geometry_msgs/Vector3 _snap
     (send _snap :deserialize buf ptr-) (incf ptr- (send _snap :serialization-length))
   ;; geometry_msgs/Quaternion _orientation
     (send _orientation :deserialize buf ptr-) (incf ptr- (send _orientation :serialization-length))
   ;; geometry_msgs/Point _attitude_rate
     (send _attitude_rate :deserialize buf ptr-) (incf ptr- (send _attitude_rate :serialization-length))
   ;; float64 _thrust
     (setq _thrust (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _heading
     (setq _heading (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _heading_rate
     (setq _heading_rate (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _heading_acceleration
     (setq _heading_acceleration (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _heading_jerk
     (setq _heading_jerk (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; bool _disable_position_gains
     (setq _disable_position_gains (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _disable_antiwindups
     (setq _disable_antiwindups (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; uint8 _use_position_horizontal
     (setq _use_position_horizontal (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _use_position_vertical
     (setq _use_position_vertical (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _use_velocity_horizontal
     (setq _use_velocity_horizontal (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _use_velocity_vertical
     (setq _use_velocity_vertical (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _use_acceleration
     (setq _use_acceleration (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _use_jerk
     (setq _use_jerk (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _use_snap
     (setq _use_snap (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _use_attitude_rate
     (setq _use_attitude_rate (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _use_heading
     (setq _use_heading (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _use_heading_rate
     (setq _use_heading_rate (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _use_heading_acceleration
     (setq _use_heading_acceleration (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _use_heading_jerk
     (setq _use_heading_jerk (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _use_orientation
     (setq _use_orientation (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _use_thrust
     (setq _use_thrust (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(setf (get itm_mav_msgs::PositionCommand :md5sum-) "4d8e95e3ee792c1a5ce3afe2d9f2396a")
(setf (get itm_mav_msgs::PositionCommand :datatype-) "itm_mav_msgs/PositionCommand")
(setf (get itm_mav_msgs::PositionCommand :definition-)
      "# This represents the output of the currently active Tracker (mrs_uav_manager::Tracker).
# This message is returned from a tracker by means of the update() function, called by the mrs_uav_manager::ControlManager.

std_msgs/Header header

# The desired position.
geometry_msgs/Point position

# The desired velocity.
geometry_msgs/Vector3 velocity

# The desired acceleration.
geometry_msgs/Vector3 acceleration

# The desired jerk.
geometry_msgs/Vector3 jerk

# The desired snap.
geometry_msgs/Vector3 snap

# The desired orientation and attitude rate.
# It is mutually exclusive to heading+heading_rate.
geometry_msgs/Quaternion orientation
geometry_msgs/Point attitude_rate

# when used, it will override the thrust output of the controller
float64 thrust

# The desired heading and heading rate.
# It is mutually exclusive to orientation+attitude_rate.
float64 heading
float64 heading_rate
float64 heading_acceleration
float64 heading_jerk

# Whether the controller should mute its position feedback.
# Useful, e.g., during takeoff or in situations where sudden control
# error is expected but not immediately required to be fixed.
bool disable_position_gains

# Whether the controller should disable its antiwindups
bool disable_antiwindups

# Flags defining whether the XY or Z position reference was filled in.
# If not, the controllers should pay no attention to it and to the corresponding control error.
uint8 use_position_horizontal
uint8 use_position_vertical

# Flags defining whether the XY or Z velocity reference was filled in.
# If not, the controllers should pay no attention to it and to the corresponding control error.
uint8 use_velocity_horizontal
uint8 use_velocity_vertical

# Flags defining which references were filled in.
# If not, the controllers should pay no attention to them and to the corresponding control errors.
uint8 use_acceleration
uint8 use_jerk
uint8 use_snap
uint8 use_attitude_rate
uint8 use_heading
uint8 use_heading_rate
uint8 use_heading_acceleration
uint8 use_heading_jerk
uint8 use_orientation
uint8 use_thrust

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
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

")



(provide :itm_mav_msgs/PositionCommand "4d8e95e3ee792c1a5ce3afe2d9f2396a")


