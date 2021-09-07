;; Auto-generated. Do not edit!


(when (boundp 'itm_nonlinear_mpc::SetVelocity)
  (if (not (find-package "ITM_NONLINEAR_MPC"))
    (make-package "ITM_NONLINEAR_MPC"))
  (shadow 'SetVelocity (find-package "ITM_NONLINEAR_MPC")))
(unless (find-package "ITM_NONLINEAR_MPC::SETVELOCITY")
  (make-package "ITM_NONLINEAR_MPC::SETVELOCITY"))
(unless (find-package "ITM_NONLINEAR_MPC::SETVELOCITYREQUEST")
  (make-package "ITM_NONLINEAR_MPC::SETVELOCITYREQUEST"))
(unless (find-package "ITM_NONLINEAR_MPC::SETVELOCITYRESPONSE")
  (make-package "ITM_NONLINEAR_MPC::SETVELOCITYRESPONSE"))

(in-package "ROS")





(defclass itm_nonlinear_mpc::SetVelocityRequest
  :super ros::object
  :slots (_vx _vy _vz _yaw _yaw_rate _frame_id _auto_arm ))

(defmethod itm_nonlinear_mpc::SetVelocityRequest
  (:init
   (&key
    ((:vx __vx) 0.0)
    ((:vy __vy) 0.0)
    ((:vz __vz) 0.0)
    ((:yaw __yaw) 0.0)
    ((:yaw_rate __yaw_rate) 0.0)
    ((:frame_id __frame_id) "")
    ((:auto_arm __auto_arm) nil)
    )
   (send-super :init)
   (setq _vx (float __vx))
   (setq _vy (float __vy))
   (setq _vz (float __vz))
   (setq _yaw (float __yaw))
   (setq _yaw_rate (float __yaw_rate))
   (setq _frame_id (string __frame_id))
   (setq _auto_arm __auto_arm)
   self)
  (:vx
   (&optional __vx)
   (if __vx (setq _vx __vx)) _vx)
  (:vy
   (&optional __vy)
   (if __vy (setq _vy __vy)) _vy)
  (:vz
   (&optional __vz)
   (if __vz (setq _vz __vz)) _vz)
  (:yaw
   (&optional __yaw)
   (if __yaw (setq _yaw __yaw)) _yaw)
  (:yaw_rate
   (&optional __yaw_rate)
   (if __yaw_rate (setq _yaw_rate __yaw_rate)) _yaw_rate)
  (:frame_id
   (&optional __frame_id)
   (if __frame_id (setq _frame_id __frame_id)) _frame_id)
  (:auto_arm
   (&optional __auto_arm)
   (if __auto_arm (setq _auto_arm __auto_arm)) _auto_arm)
  (:serialization-length
   ()
   (+
    ;; float32 _vx
    4
    ;; float32 _vy
    4
    ;; float32 _vz
    4
    ;; float32 _yaw
    4
    ;; float32 _yaw_rate
    4
    ;; string _frame_id
    4 (length _frame_id)
    ;; bool _auto_arm
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _vx
       (sys::poke _vx (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _vy
       (sys::poke _vy (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _vz
       (sys::poke _vz (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _yaw
       (sys::poke _yaw (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _yaw_rate
       (sys::poke _yaw_rate (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; string _frame_id
       (write-long (length _frame_id) s) (princ _frame_id s)
     ;; bool _auto_arm
       (if _auto_arm (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _vx
     (setq _vx (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _vy
     (setq _vy (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _vz
     (setq _vz (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _yaw
     (setq _yaw (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _yaw_rate
     (setq _yaw_rate (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; string _frame_id
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _frame_id (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; bool _auto_arm
     (setq _auto_arm (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass itm_nonlinear_mpc::SetVelocityResponse
  :super ros::object
  :slots (_success _message ))

(defmethod itm_nonlinear_mpc::SetVelocityResponse
  (:init
   (&key
    ((:success __success) nil)
    ((:message __message) "")
    )
   (send-super :init)
   (setq _success __success)
   (setq _message (string __message))
   self)
  (:success
   (&optional __success)
   (if __success (setq _success __success)) _success)
  (:message
   (&optional __message)
   (if __message (setq _message __message)) _message)
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ;; string _message
    4 (length _message)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;; string _message
       (write-long (length _message) s) (princ _message s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _message
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _message (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass itm_nonlinear_mpc::SetVelocity
  :super ros::object
  :slots ())

(setf (get itm_nonlinear_mpc::SetVelocity :md5sum-) "0c536fc3fdea57936646d4924e14c3d1")
(setf (get itm_nonlinear_mpc::SetVelocity :datatype-) "itm_nonlinear_mpc/SetVelocity")
(setf (get itm_nonlinear_mpc::SetVelocity :request) itm_nonlinear_mpc::SetVelocityRequest)
(setf (get itm_nonlinear_mpc::SetVelocity :response) itm_nonlinear_mpc::SetVelocityResponse)

(defmethod itm_nonlinear_mpc::SetVelocityRequest
  (:response () (instance itm_nonlinear_mpc::SetVelocityResponse :init)))

(setf (get itm_nonlinear_mpc::SetVelocityRequest :md5sum-) "0c536fc3fdea57936646d4924e14c3d1")
(setf (get itm_nonlinear_mpc::SetVelocityRequest :datatype-) "itm_nonlinear_mpc/SetVelocityRequest")
(setf (get itm_nonlinear_mpc::SetVelocityRequest :definition-)
      "float32 vx
float32 vy
float32 vz
float32 yaw
float32 yaw_rate
string frame_id
bool auto_arm
---
bool success
string message

")

(setf (get itm_nonlinear_mpc::SetVelocityResponse :md5sum-) "0c536fc3fdea57936646d4924e14c3d1")
(setf (get itm_nonlinear_mpc::SetVelocityResponse :datatype-) "itm_nonlinear_mpc/SetVelocityResponse")
(setf (get itm_nonlinear_mpc::SetVelocityResponse :definition-)
      "float32 vx
float32 vy
float32 vz
float32 yaw
float32 yaw_rate
string frame_id
bool auto_arm
---
bool success
string message

")



(provide :itm_nonlinear_mpc/SetVelocity "0c536fc3fdea57936646d4924e14c3d1")


