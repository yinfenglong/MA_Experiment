;; Auto-generated. Do not edit!


(when (boundp 'itm_nonlinear_mpc::SetRates)
  (if (not (find-package "ITM_NONLINEAR_MPC"))
    (make-package "ITM_NONLINEAR_MPC"))
  (shadow 'SetRates (find-package "ITM_NONLINEAR_MPC")))
(unless (find-package "ITM_NONLINEAR_MPC::SETRATES")
  (make-package "ITM_NONLINEAR_MPC::SETRATES"))
(unless (find-package "ITM_NONLINEAR_MPC::SETRATESREQUEST")
  (make-package "ITM_NONLINEAR_MPC::SETRATESREQUEST"))
(unless (find-package "ITM_NONLINEAR_MPC::SETRATESRESPONSE")
  (make-package "ITM_NONLINEAR_MPC::SETRATESRESPONSE"))

(in-package "ROS")





(defclass itm_nonlinear_mpc::SetRatesRequest
  :super ros::object
  :slots (_pitch_rate _roll_rate _yaw_rate _thrust _auto_arm ))

(defmethod itm_nonlinear_mpc::SetRatesRequest
  (:init
   (&key
    ((:pitch_rate __pitch_rate) 0.0)
    ((:roll_rate __roll_rate) 0.0)
    ((:yaw_rate __yaw_rate) 0.0)
    ((:thrust __thrust) 0.0)
    ((:auto_arm __auto_arm) nil)
    )
   (send-super :init)
   (setq _pitch_rate (float __pitch_rate))
   (setq _roll_rate (float __roll_rate))
   (setq _yaw_rate (float __yaw_rate))
   (setq _thrust (float __thrust))
   (setq _auto_arm __auto_arm)
   self)
  (:pitch_rate
   (&optional __pitch_rate)
   (if __pitch_rate (setq _pitch_rate __pitch_rate)) _pitch_rate)
  (:roll_rate
   (&optional __roll_rate)
   (if __roll_rate (setq _roll_rate __roll_rate)) _roll_rate)
  (:yaw_rate
   (&optional __yaw_rate)
   (if __yaw_rate (setq _yaw_rate __yaw_rate)) _yaw_rate)
  (:thrust
   (&optional __thrust)
   (if __thrust (setq _thrust __thrust)) _thrust)
  (:auto_arm
   (&optional __auto_arm)
   (if __auto_arm (setq _auto_arm __auto_arm)) _auto_arm)
  (:serialization-length
   ()
   (+
    ;; float32 _pitch_rate
    4
    ;; float32 _roll_rate
    4
    ;; float32 _yaw_rate
    4
    ;; float32 _thrust
    4
    ;; bool _auto_arm
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _pitch_rate
       (sys::poke _pitch_rate (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _roll_rate
       (sys::poke _roll_rate (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _yaw_rate
       (sys::poke _yaw_rate (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _thrust
       (sys::poke _thrust (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; bool _auto_arm
       (if _auto_arm (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _pitch_rate
     (setq _pitch_rate (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _roll_rate
     (setq _roll_rate (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _yaw_rate
     (setq _yaw_rate (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _thrust
     (setq _thrust (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; bool _auto_arm
     (setq _auto_arm (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass itm_nonlinear_mpc::SetRatesResponse
  :super ros::object
  :slots (_success _message ))

(defmethod itm_nonlinear_mpc::SetRatesResponse
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

(defclass itm_nonlinear_mpc::SetRates
  :super ros::object
  :slots ())

(setf (get itm_nonlinear_mpc::SetRates :md5sum-) "a9cc2408dc007c6dd1f503c73d267539")
(setf (get itm_nonlinear_mpc::SetRates :datatype-) "itm_nonlinear_mpc/SetRates")
(setf (get itm_nonlinear_mpc::SetRates :request) itm_nonlinear_mpc::SetRatesRequest)
(setf (get itm_nonlinear_mpc::SetRates :response) itm_nonlinear_mpc::SetRatesResponse)

(defmethod itm_nonlinear_mpc::SetRatesRequest
  (:response () (instance itm_nonlinear_mpc::SetRatesResponse :init)))

(setf (get itm_nonlinear_mpc::SetRatesRequest :md5sum-) "a9cc2408dc007c6dd1f503c73d267539")
(setf (get itm_nonlinear_mpc::SetRatesRequest :datatype-) "itm_nonlinear_mpc/SetRatesRequest")
(setf (get itm_nonlinear_mpc::SetRatesRequest :definition-)
      "float32 pitch_rate
float32 roll_rate
float32 yaw_rate
float32 thrust
bool auto_arm
---
bool success
string message

")

(setf (get itm_nonlinear_mpc::SetRatesResponse :md5sum-) "a9cc2408dc007c6dd1f503c73d267539")
(setf (get itm_nonlinear_mpc::SetRatesResponse :datatype-) "itm_nonlinear_mpc/SetRatesResponse")
(setf (get itm_nonlinear_mpc::SetRatesResponse :definition-)
      "float32 pitch_rate
float32 roll_rate
float32 yaw_rate
float32 thrust
bool auto_arm
---
bool success
string message

")



(provide :itm_nonlinear_mpc/SetRates "a9cc2408dc007c6dd1f503c73d267539")


