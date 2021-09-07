;; Auto-generated. Do not edit!


(when (boundp 'itm_nonlinear_mpc::itm_trajectory_srv)
  (if (not (find-package "ITM_NONLINEAR_MPC"))
    (make-package "ITM_NONLINEAR_MPC"))
  (shadow 'itm_trajectory_srv (find-package "ITM_NONLINEAR_MPC")))
(unless (find-package "ITM_NONLINEAR_MPC::ITM_TRAJECTORY_SRV")
  (make-package "ITM_NONLINEAR_MPC::ITM_TRAJECTORY_SRV"))
(unless (find-package "ITM_NONLINEAR_MPC::ITM_TRAJECTORY_SRVREQUEST")
  (make-package "ITM_NONLINEAR_MPC::ITM_TRAJECTORY_SRVREQUEST"))
(unless (find-package "ITM_NONLINEAR_MPC::ITM_TRAJECTORY_SRVRESPONSE")
  (make-package "ITM_NONLINEAR_MPC::ITM_TRAJECTORY_SRVRESPONSE"))

(in-package "ROS")





(defclass itm_nonlinear_mpc::itm_trajectory_srvRequest
  :super ros::object
  :slots (_index _traj ))

(defmethod itm_nonlinear_mpc::itm_trajectory_srvRequest
  (:init
   (&key
    ((:index __index) 0)
    ((:traj __traj) (let (r) (dotimes (i 0) (push (instance itm_nonlinear_mpc::itm_trajectory_point :init) r)) r))
    )
   (send-super :init)
   (setq _index (round __index))
   (setq _traj __traj)
   self)
  (:index
   (&optional __index)
   (if __index (setq _index __index)) _index)
  (:traj
   (&rest __traj)
   (if (keywordp (car __traj))
       (send* _traj __traj)
     (progn
       (if __traj (setq _traj (car __traj)))
       _traj)))
  (:serialization-length
   ()
   (+
    ;; uint32 _index
    4
    ;; itm_nonlinear_mpc/itm_trajectory_point[] _traj
    (apply #'+ (send-all _traj :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _index
       (write-long _index s)
     ;; itm_nonlinear_mpc/itm_trajectory_point[] _traj
     (write-long (length _traj) s)
     (dolist (elem _traj)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _index
     (setq _index (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; itm_nonlinear_mpc/itm_trajectory_point[] _traj
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _traj (let (r) (dotimes (i n) (push (instance itm_nonlinear_mpc::itm_trajectory_point :init) r)) r))
     (dolist (elem- _traj)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(defclass itm_nonlinear_mpc::itm_trajectory_srvResponse
  :super ros::object
  :slots (_success ))

(defmethod itm_nonlinear_mpc::itm_trajectory_srvResponse
  (:init
   (&key
    ((:success __success) nil)
    )
   (send-super :init)
   (setq _success __success)
   self)
  (:success
   (&optional __success)
   (if __success (setq _success __success)) _success)
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass itm_nonlinear_mpc::itm_trajectory_srv
  :super ros::object
  :slots ())

(setf (get itm_nonlinear_mpc::itm_trajectory_srv :md5sum-) "26ff1de4ce8cbafd1f9f9a97392ec44f")
(setf (get itm_nonlinear_mpc::itm_trajectory_srv :datatype-) "itm_nonlinear_mpc/itm_trajectory_srv")
(setf (get itm_nonlinear_mpc::itm_trajectory_srv :request) itm_nonlinear_mpc::itm_trajectory_srvRequest)
(setf (get itm_nonlinear_mpc::itm_trajectory_srv :response) itm_nonlinear_mpc::itm_trajectory_srvResponse)

(defmethod itm_nonlinear_mpc::itm_trajectory_srvRequest
  (:response () (instance itm_nonlinear_mpc::itm_trajectory_srvResponse :init)))

(setf (get itm_nonlinear_mpc::itm_trajectory_srvRequest :md5sum-) "26ff1de4ce8cbafd1f9f9a97392ec44f")
(setf (get itm_nonlinear_mpc::itm_trajectory_srvRequest :datatype-) "itm_nonlinear_mpc/itm_trajectory_srvRequest")
(setf (get itm_nonlinear_mpc::itm_trajectory_srvRequest :definition-)
      "uint32 index
itm_nonlinear_mpc/itm_trajectory_point[] traj

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
---
bool success
")

(setf (get itm_nonlinear_mpc::itm_trajectory_srvResponse :md5sum-) "26ff1de4ce8cbafd1f9f9a97392ec44f")
(setf (get itm_nonlinear_mpc::itm_trajectory_srvResponse :datatype-) "itm_nonlinear_mpc/itm_trajectory_srvResponse")
(setf (get itm_nonlinear_mpc::itm_trajectory_srvResponse :definition-)
      "uint32 index
itm_nonlinear_mpc/itm_trajectory_point[] traj

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
---
bool success
")



(provide :itm_nonlinear_mpc/itm_trajectory_srv "26ff1de4ce8cbafd1f9f9a97392ec44f")


