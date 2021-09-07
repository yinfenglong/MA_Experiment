;; Auto-generated. Do not edit!


(when (boundp 'itm_nonlinear_mpc::mpc_command_rpyt)
  (if (not (find-package "ITM_NONLINEAR_MPC"))
    (make-package "ITM_NONLINEAR_MPC"))
  (shadow 'mpc_command_rpyt (find-package "ITM_NONLINEAR_MPC")))
(unless (find-package "ITM_NONLINEAR_MPC::MPC_COMMAND_RPYT")
  (make-package "ITM_NONLINEAR_MPC::MPC_COMMAND_RPYT"))

(in-package "ROS")
;;//! \htmlinclude mpc_command_rpyt.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass itm_nonlinear_mpc::mpc_command_rpyt
  :super ros::object
  :slots (_header _roll_ref _pitch_ref _yaw_rate_cmd _thrust_ref ))

(defmethod itm_nonlinear_mpc::mpc_command_rpyt
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:roll_ref __roll_ref) 0.0)
    ((:pitch_ref __pitch_ref) 0.0)
    ((:yaw_rate_cmd __yaw_rate_cmd) 0.0)
    ((:thrust_ref __thrust_ref) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _roll_ref (float __roll_ref))
   (setq _pitch_ref (float __pitch_ref))
   (setq _yaw_rate_cmd (float __yaw_rate_cmd))
   (setq _thrust_ref (float __thrust_ref))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:roll_ref
   (&optional __roll_ref)
   (if __roll_ref (setq _roll_ref __roll_ref)) _roll_ref)
  (:pitch_ref
   (&optional __pitch_ref)
   (if __pitch_ref (setq _pitch_ref __pitch_ref)) _pitch_ref)
  (:yaw_rate_cmd
   (&optional __yaw_rate_cmd)
   (if __yaw_rate_cmd (setq _yaw_rate_cmd __yaw_rate_cmd)) _yaw_rate_cmd)
  (:thrust_ref
   (&optional __thrust_ref)
   (if __thrust_ref (setq _thrust_ref __thrust_ref)) _thrust_ref)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float64 _roll_ref
    8
    ;; float64 _pitch_ref
    8
    ;; float64 _yaw_rate_cmd
    8
    ;; float64 _thrust_ref
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float64 _roll_ref
       (sys::poke _roll_ref (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _pitch_ref
       (sys::poke _pitch_ref (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _yaw_rate_cmd
       (sys::poke _yaw_rate_cmd (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _thrust_ref
       (sys::poke _thrust_ref (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float64 _roll_ref
     (setq _roll_ref (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _pitch_ref
     (setq _pitch_ref (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _yaw_rate_cmd
     (setq _yaw_rate_cmd (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _thrust_ref
     (setq _thrust_ref (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get itm_nonlinear_mpc::mpc_command_rpyt :md5sum-) "3a1aaed29b0fec0f986f12a3290ec8b8")
(setf (get itm_nonlinear_mpc::mpc_command_rpyt :datatype-) "itm_nonlinear_mpc/mpc_command_rpyt")
(setf (get itm_nonlinear_mpc::mpc_command_rpyt :definition-)
      "std_msgs/Header header
float64 roll_ref
float64 pitch_ref
float64 yaw_rate_cmd
float64 thrust_ref
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

")



(provide :itm_nonlinear_mpc/mpc_command_rpyt "3a1aaed29b0fec0f986f12a3290ec8b8")


