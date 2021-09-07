;; Auto-generated. Do not edit!


(when (boundp 'itm_nonlinear_mpc::itm_trajectory_msg)
  (if (not (find-package "ITM_NONLINEAR_MPC"))
    (make-package "ITM_NONLINEAR_MPC"))
  (shadow 'itm_trajectory_msg (find-package "ITM_NONLINEAR_MPC")))
(unless (find-package "ITM_NONLINEAR_MPC::ITM_TRAJECTORY_MSG")
  (make-package "ITM_NONLINEAR_MPC::ITM_TRAJECTORY_MSG"))

(in-package "ROS")
;;//! \htmlinclude itm_trajectory_msg.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass itm_nonlinear_mpc::itm_trajectory_msg
  :super ros::object
  :slots (_header _size _traj _data ))

(defmethod itm_nonlinear_mpc::itm_trajectory_msg
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:size __size) 0)
    ((:traj __traj) (let (r) (dotimes (i 0) (push (instance itm_nonlinear_mpc::itm_trajectory_point :init) r)) r))
    ((:data __data) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _header __header)
   (setq _size (round __size))
   (setq _traj __traj)
   (setq _data __data)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:size
   (&optional __size)
   (if __size (setq _size __size)) _size)
  (:traj
   (&rest __traj)
   (if (keywordp (car __traj))
       (send* _traj __traj)
     (progn
       (if __traj (setq _traj (car __traj)))
       _traj)))
  (:data
   (&optional __data)
   (if __data (setq _data __data)) _data)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uint32 _size
    4
    ;; itm_nonlinear_mpc/itm_trajectory_point[] _traj
    (apply #'+ (send-all _traj :serialization-length)) 4
    ;; float64[] _data
    (* 8    (length _data)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; uint32 _size
       (write-long _size s)
     ;; itm_nonlinear_mpc/itm_trajectory_point[] _traj
     (write-long (length _traj) s)
     (dolist (elem _traj)
       (send elem :serialize s)
       )
     ;; float64[] _data
     (write-long (length _data) s)
     (dotimes (i (length _data))
       (sys::poke (elt _data i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; uint32 _size
     (setq _size (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; itm_nonlinear_mpc/itm_trajectory_point[] _traj
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _traj (let (r) (dotimes (i n) (push (instance itm_nonlinear_mpc::itm_trajectory_point :init) r)) r))
     (dolist (elem- _traj)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; float64[] _data
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _data (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _data i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;;
   self)
  )

(setf (get itm_nonlinear_mpc::itm_trajectory_msg :md5sum-) "5fbf4a175c3bba98dd9f6a860e83cf2f")
(setf (get itm_nonlinear_mpc::itm_trajectory_msg :datatype-) "itm_nonlinear_mpc/itm_trajectory_msg")
(setf (get itm_nonlinear_mpc::itm_trajectory_msg :definition-)
      "Header header
uint32 size
itm_nonlinear_mpc/itm_trajectory_point[] traj
float64[] data
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
MSG: itm_nonlinear_mpc/itm_trajectory_point
float64 x
float64 y
float64 z
float64 vx
float64 vy
float64 vz
float64 roll
float64 pitch
float64 yaw
float64[4] q
float64[2] cube_x
float64[2] cube_y
float64[2] cube_z
float64[2] cube_yaw
bool fixed
bool time_known
int8 derivative
int8 segment_index
float64 time_stamp
bool quaternion_given

")



(provide :itm_nonlinear_mpc/itm_trajectory_msg "5fbf4a175c3bba98dd9f6a860e83cf2f")


