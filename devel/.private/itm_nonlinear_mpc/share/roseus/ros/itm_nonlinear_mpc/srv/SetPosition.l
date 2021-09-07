;; Auto-generated. Do not edit!


(when (boundp 'itm_nonlinear_mpc::SetPosition)
  (if (not (find-package "ITM_NONLINEAR_MPC"))
    (make-package "ITM_NONLINEAR_MPC"))
  (shadow 'SetPosition (find-package "ITM_NONLINEAR_MPC")))
(unless (find-package "ITM_NONLINEAR_MPC::SETPOSITION")
  (make-package "ITM_NONLINEAR_MPC::SETPOSITION"))
(unless (find-package "ITM_NONLINEAR_MPC::SETPOSITIONREQUEST")
  (make-package "ITM_NONLINEAR_MPC::SETPOSITIONREQUEST"))
(unless (find-package "ITM_NONLINEAR_MPC::SETPOSITIONRESPONSE")
  (make-package "ITM_NONLINEAR_MPC::SETPOSITIONRESPONSE"))

(in-package "ROS")





(defclass itm_nonlinear_mpc::SetPositionRequest
  :super ros::object
  :slots (_x _y _z _yaw _yaw_rate _frame_id _auto_arm ))

(defmethod itm_nonlinear_mpc::SetPositionRequest
  (:init
   (&key
    ((:x __x) 0.0)
    ((:y __y) 0.0)
    ((:z __z) 0.0)
    ((:yaw __yaw) 0.0)
    ((:yaw_rate __yaw_rate) 0.0)
    ((:frame_id __frame_id) "")
    ((:auto_arm __auto_arm) nil)
    )
   (send-super :init)
   (setq _x (float __x))
   (setq _y (float __y))
   (setq _z (float __z))
   (setq _yaw (float __yaw))
   (setq _yaw_rate (float __yaw_rate))
   (setq _frame_id (string __frame_id))
   (setq _auto_arm __auto_arm)
   self)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:z
   (&optional __z)
   (if __z (setq _z __z)) _z)
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
    ;; float32 _x
    4
    ;; float32 _y
    4
    ;; float32 _z
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
     ;; float32 _x
       (sys::poke _x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _y
       (sys::poke _y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _z
       (sys::poke _z (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
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
   ;; float32 _x
     (setq _x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _y
     (setq _y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _z
     (setq _z (sys::peek buf ptr- :float)) (incf ptr- 4)
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

(defclass itm_nonlinear_mpc::SetPositionResponse
  :super ros::object
  :slots (_success _message ))

(defmethod itm_nonlinear_mpc::SetPositionResponse
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

(defclass itm_nonlinear_mpc::SetPosition
  :super ros::object
  :slots ())

(setf (get itm_nonlinear_mpc::SetPosition :md5sum-) "69cb8e201b1f4163f507e2b13fa97684")
(setf (get itm_nonlinear_mpc::SetPosition :datatype-) "itm_nonlinear_mpc/SetPosition")
(setf (get itm_nonlinear_mpc::SetPosition :request) itm_nonlinear_mpc::SetPositionRequest)
(setf (get itm_nonlinear_mpc::SetPosition :response) itm_nonlinear_mpc::SetPositionResponse)

(defmethod itm_nonlinear_mpc::SetPositionRequest
  (:response () (instance itm_nonlinear_mpc::SetPositionResponse :init)))

(setf (get itm_nonlinear_mpc::SetPositionRequest :md5sum-) "69cb8e201b1f4163f507e2b13fa97684")
(setf (get itm_nonlinear_mpc::SetPositionRequest :datatype-) "itm_nonlinear_mpc/SetPositionRequest")
(setf (get itm_nonlinear_mpc::SetPositionRequest :definition-)
      "float32 x
float32 y
float32 z
float32 yaw
float32 yaw_rate
string frame_id
bool auto_arm
---
bool success
string message

")

(setf (get itm_nonlinear_mpc::SetPositionResponse :md5sum-) "69cb8e201b1f4163f507e2b13fa97684")
(setf (get itm_nonlinear_mpc::SetPositionResponse :datatype-) "itm_nonlinear_mpc/SetPositionResponse")
(setf (get itm_nonlinear_mpc::SetPositionResponse :definition-)
      "float32 x
float32 y
float32 z
float32 yaw
float32 yaw_rate
string frame_id
bool auto_arm
---
bool success
string message

")



(provide :itm_nonlinear_mpc/SetPosition "69cb8e201b1f4163f507e2b13fa97684")


