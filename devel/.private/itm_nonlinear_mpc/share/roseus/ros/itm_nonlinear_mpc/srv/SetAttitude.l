;; Auto-generated. Do not edit!


(when (boundp 'itm_nonlinear_mpc::SetAttitude)
  (if (not (find-package "ITM_NONLINEAR_MPC"))
    (make-package "ITM_NONLINEAR_MPC"))
  (shadow 'SetAttitude (find-package "ITM_NONLINEAR_MPC")))
(unless (find-package "ITM_NONLINEAR_MPC::SETATTITUDE")
  (make-package "ITM_NONLINEAR_MPC::SETATTITUDE"))
(unless (find-package "ITM_NONLINEAR_MPC::SETATTITUDEREQUEST")
  (make-package "ITM_NONLINEAR_MPC::SETATTITUDEREQUEST"))
(unless (find-package "ITM_NONLINEAR_MPC::SETATTITUDERESPONSE")
  (make-package "ITM_NONLINEAR_MPC::SETATTITUDERESPONSE"))

(in-package "ROS")





(defclass itm_nonlinear_mpc::SetAttitudeRequest
  :super ros::object
  :slots (_pitch _roll _yaw _thrust _frame_id _auto_arm ))

(defmethod itm_nonlinear_mpc::SetAttitudeRequest
  (:init
   (&key
    ((:pitch __pitch) 0.0)
    ((:roll __roll) 0.0)
    ((:yaw __yaw) 0.0)
    ((:thrust __thrust) 0.0)
    ((:frame_id __frame_id) "")
    ((:auto_arm __auto_arm) nil)
    )
   (send-super :init)
   (setq _pitch (float __pitch))
   (setq _roll (float __roll))
   (setq _yaw (float __yaw))
   (setq _thrust (float __thrust))
   (setq _frame_id (string __frame_id))
   (setq _auto_arm __auto_arm)
   self)
  (:pitch
   (&optional __pitch)
   (if __pitch (setq _pitch __pitch)) _pitch)
  (:roll
   (&optional __roll)
   (if __roll (setq _roll __roll)) _roll)
  (:yaw
   (&optional __yaw)
   (if __yaw (setq _yaw __yaw)) _yaw)
  (:thrust
   (&optional __thrust)
   (if __thrust (setq _thrust __thrust)) _thrust)
  (:frame_id
   (&optional __frame_id)
   (if __frame_id (setq _frame_id __frame_id)) _frame_id)
  (:auto_arm
   (&optional __auto_arm)
   (if __auto_arm (setq _auto_arm __auto_arm)) _auto_arm)
  (:serialization-length
   ()
   (+
    ;; float32 _pitch
    4
    ;; float32 _roll
    4
    ;; float32 _yaw
    4
    ;; float32 _thrust
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
     ;; float32 _pitch
       (sys::poke _pitch (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _roll
       (sys::poke _roll (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _yaw
       (sys::poke _yaw (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _thrust
       (sys::poke _thrust (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; string _frame_id
       (write-long (length _frame_id) s) (princ _frame_id s)
     ;; bool _auto_arm
       (if _auto_arm (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _pitch
     (setq _pitch (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _roll
     (setq _roll (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _yaw
     (setq _yaw (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _thrust
     (setq _thrust (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; string _frame_id
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _frame_id (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; bool _auto_arm
     (setq _auto_arm (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass itm_nonlinear_mpc::SetAttitudeResponse
  :super ros::object
  :slots (_success _message ))

(defmethod itm_nonlinear_mpc::SetAttitudeResponse
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

(defclass itm_nonlinear_mpc::SetAttitude
  :super ros::object
  :slots ())

(setf (get itm_nonlinear_mpc::SetAttitude :md5sum-) "cc2080a08d58c004756f050a7b2dc084")
(setf (get itm_nonlinear_mpc::SetAttitude :datatype-) "itm_nonlinear_mpc/SetAttitude")
(setf (get itm_nonlinear_mpc::SetAttitude :request) itm_nonlinear_mpc::SetAttitudeRequest)
(setf (get itm_nonlinear_mpc::SetAttitude :response) itm_nonlinear_mpc::SetAttitudeResponse)

(defmethod itm_nonlinear_mpc::SetAttitudeRequest
  (:response () (instance itm_nonlinear_mpc::SetAttitudeResponse :init)))

(setf (get itm_nonlinear_mpc::SetAttitudeRequest :md5sum-) "cc2080a08d58c004756f050a7b2dc084")
(setf (get itm_nonlinear_mpc::SetAttitudeRequest :datatype-) "itm_nonlinear_mpc/SetAttitudeRequest")
(setf (get itm_nonlinear_mpc::SetAttitudeRequest :definition-)
      "float32 pitch
float32 roll
float32 yaw
float32 thrust
string frame_id
bool auto_arm
---
bool success
string message

")

(setf (get itm_nonlinear_mpc::SetAttitudeResponse :md5sum-) "cc2080a08d58c004756f050a7b2dc084")
(setf (get itm_nonlinear_mpc::SetAttitudeResponse :datatype-) "itm_nonlinear_mpc/SetAttitudeResponse")
(setf (get itm_nonlinear_mpc::SetAttitudeResponse :definition-)
      "float32 pitch
float32 roll
float32 yaw
float32 thrust
string frame_id
bool auto_arm
---
bool success
string message

")



(provide :itm_nonlinear_mpc/SetAttitude "cc2080a08d58c004756f050a7b2dc084")


