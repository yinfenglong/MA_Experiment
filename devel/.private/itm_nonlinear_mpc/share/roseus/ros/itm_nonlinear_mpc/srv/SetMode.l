;; Auto-generated. Do not edit!


(when (boundp 'itm_nonlinear_mpc::SetMode)
  (if (not (find-package "ITM_NONLINEAR_MPC"))
    (make-package "ITM_NONLINEAR_MPC"))
  (shadow 'SetMode (find-package "ITM_NONLINEAR_MPC")))
(unless (find-package "ITM_NONLINEAR_MPC::SETMODE")
  (make-package "ITM_NONLINEAR_MPC::SETMODE"))
(unless (find-package "ITM_NONLINEAR_MPC::SETMODEREQUEST")
  (make-package "ITM_NONLINEAR_MPC::SETMODEREQUEST"))
(unless (find-package "ITM_NONLINEAR_MPC::SETMODERESPONSE")
  (make-package "ITM_NONLINEAR_MPC::SETMODERESPONSE"))

(in-package "ROS")





(defclass itm_nonlinear_mpc::SetModeRequest
  :super ros::object
  :slots (_mode ))

(defmethod itm_nonlinear_mpc::SetModeRequest
  (:init
   (&key
    ((:mode __mode) 0)
    )
   (send-super :init)
   (setq _mode (round __mode))
   self)
  (:mode
   (&optional __mode)
   (if __mode (setq _mode __mode)) _mode)
  (:serialization-length
   ()
   (+
    ;; uint8 _mode
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _mode
       (write-byte _mode s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _mode
     (setq _mode (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(defclass itm_nonlinear_mpc::SetModeResponse
  :super ros::object
  :slots (_success _message ))

(defmethod itm_nonlinear_mpc::SetModeResponse
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

(defclass itm_nonlinear_mpc::SetMode
  :super ros::object
  :slots ())

(setf (get itm_nonlinear_mpc::SetMode :md5sum-) "718da2351c63fdc303e9567a9bc6772c")
(setf (get itm_nonlinear_mpc::SetMode :datatype-) "itm_nonlinear_mpc/SetMode")
(setf (get itm_nonlinear_mpc::SetMode :request) itm_nonlinear_mpc::SetModeRequest)
(setf (get itm_nonlinear_mpc::SetMode :response) itm_nonlinear_mpc::SetModeResponse)

(defmethod itm_nonlinear_mpc::SetModeRequest
  (:response () (instance itm_nonlinear_mpc::SetModeResponse :init)))

(setf (get itm_nonlinear_mpc::SetModeRequest :md5sum-) "718da2351c63fdc303e9567a9bc6772c")
(setf (get itm_nonlinear_mpc::SetModeRequest :datatype-) "itm_nonlinear_mpc/SetModeRequest")
(setf (get itm_nonlinear_mpc::SetModeRequest :definition-)
      "uint8 mode
---
bool success
string message
")

(setf (get itm_nonlinear_mpc::SetModeResponse :md5sum-) "718da2351c63fdc303e9567a9bc6772c")
(setf (get itm_nonlinear_mpc::SetModeResponse :datatype-) "itm_nonlinear_mpc/SetModeResponse")
(setf (get itm_nonlinear_mpc::SetModeResponse :definition-)
      "uint8 mode
---
bool success
string message
")



(provide :itm_nonlinear_mpc/SetMode "718da2351c63fdc303e9567a9bc6772c")


