;; Auto-generated. Do not edit!


(when (boundp 'itm_nonlinear_mpc::SetMission)
  (if (not (find-package "ITM_NONLINEAR_MPC"))
    (make-package "ITM_NONLINEAR_MPC"))
  (shadow 'SetMission (find-package "ITM_NONLINEAR_MPC")))
(unless (find-package "ITM_NONLINEAR_MPC::SETMISSION")
  (make-package "ITM_NONLINEAR_MPC::SETMISSION"))

(in-package "ROS")
;;//! \htmlinclude SetMission.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass itm_nonlinear_mpc::SetMission
  :super ros::object
  :slots (_header _command_idx _mission_mode ))

(defmethod itm_nonlinear_mpc::SetMission
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:command_idx __command_idx) 0)
    ((:mission_mode __mission_mode) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _command_idx (round __command_idx))
   (setq _mission_mode (round __mission_mode))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:command_idx
   (&optional __command_idx)
   (if __command_idx (setq _command_idx __command_idx)) _command_idx)
  (:mission_mode
   (&optional __mission_mode)
   (if __mission_mode (setq _mission_mode __mission_mode)) _mission_mode)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uint32 _command_idx
    4
    ;; uint8 _mission_mode
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; uint32 _command_idx
       (write-long _command_idx s)
     ;; uint8 _mission_mode
       (write-byte _mission_mode s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; uint32 _command_idx
     (setq _command_idx (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint8 _mission_mode
     (setq _mission_mode (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(setf (get itm_nonlinear_mpc::SetMission :md5sum-) "1631998827fd12d678dc74adb693f0b9")
(setf (get itm_nonlinear_mpc::SetMission :datatype-) "itm_nonlinear_mpc/SetMission")
(setf (get itm_nonlinear_mpc::SetMission :definition-)
      "std_msgs/Header header
uint32 command_idx
uint8 mission_mode
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



(provide :itm_nonlinear_mpc/SetMission "1631998827fd12d678dc74adb693f0b9")


