;; Auto-generated. Do not edit!


(when (boundp 'itm_nonlinear_mpc::GetControllerState)
  (if (not (find-package "ITM_NONLINEAR_MPC"))
    (make-package "ITM_NONLINEAR_MPC"))
  (shadow 'GetControllerState (find-package "ITM_NONLINEAR_MPC")))
(unless (find-package "ITM_NONLINEAR_MPC::GETCONTROLLERSTATE")
  (make-package "ITM_NONLINEAR_MPC::GETCONTROLLERSTATE"))
(unless (find-package "ITM_NONLINEAR_MPC::GETCONTROLLERSTATEREQUEST")
  (make-package "ITM_NONLINEAR_MPC::GETCONTROLLERSTATEREQUEST"))
(unless (find-package "ITM_NONLINEAR_MPC::GETCONTROLLERSTATERESPONSE")
  (make-package "ITM_NONLINEAR_MPC::GETCONTROLLERSTATERESPONSE"))

(in-package "ROS")





(defclass itm_nonlinear_mpc::GetControllerStateRequest
  :super ros::object
  :slots (_robot_name _command_id ))

(defmethod itm_nonlinear_mpc::GetControllerStateRequest
  (:init
   (&key
    ((:robot_name __robot_name) "")
    ((:command_id __command_id) 0)
    )
   (send-super :init)
   (setq _robot_name (string __robot_name))
   (setq _command_id (round __command_id))
   self)
  (:robot_name
   (&optional __robot_name)
   (if __robot_name (setq _robot_name __robot_name)) _robot_name)
  (:command_id
   (&optional __command_id)
   (if __command_id (setq _command_id __command_id)) _command_id)
  (:serialization-length
   ()
   (+
    ;; string _robot_name
    4 (length _robot_name)
    ;; uint8 _command_id
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _robot_name
       (write-long (length _robot_name) s) (princ _robot_name s)
     ;; uint8 _command_id
       (write-byte _command_id s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _robot_name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _robot_name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; uint8 _command_id
     (setq _command_id (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(defclass itm_nonlinear_mpc::GetControllerStateResponse
  :super ros::object
  :slots (_connected ))

(defmethod itm_nonlinear_mpc::GetControllerStateResponse
  (:init
   (&key
    ((:connected __connected) nil)
    )
   (send-super :init)
   (setq _connected __connected)
   self)
  (:connected
   (&optional __connected)
   (if __connected (setq _connected __connected)) _connected)
  (:serialization-length
   ()
   (+
    ;; bool _connected
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _connected
       (if _connected (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _connected
     (setq _connected (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass itm_nonlinear_mpc::GetControllerState
  :super ros::object
  :slots ())

(setf (get itm_nonlinear_mpc::GetControllerState :md5sum-) "5ded73ff38c3ac0ab83d4d33c5cfb2fd")
(setf (get itm_nonlinear_mpc::GetControllerState :datatype-) "itm_nonlinear_mpc/GetControllerState")
(setf (get itm_nonlinear_mpc::GetControllerState :request) itm_nonlinear_mpc::GetControllerStateRequest)
(setf (get itm_nonlinear_mpc::GetControllerState :response) itm_nonlinear_mpc::GetControllerStateResponse)

(defmethod itm_nonlinear_mpc::GetControllerStateRequest
  (:response () (instance itm_nonlinear_mpc::GetControllerStateResponse :init)))

(setf (get itm_nonlinear_mpc::GetControllerStateRequest :md5sum-) "5ded73ff38c3ac0ab83d4d33c5cfb2fd")
(setf (get itm_nonlinear_mpc::GetControllerStateRequest :datatype-) "itm_nonlinear_mpc/GetControllerStateRequest")
(setf (get itm_nonlinear_mpc::GetControllerStateRequest :definition-)
      "string robot_name
uint8 command_id
---
bool connected

")

(setf (get itm_nonlinear_mpc::GetControllerStateResponse :md5sum-) "5ded73ff38c3ac0ab83d4d33c5cfb2fd")
(setf (get itm_nonlinear_mpc::GetControllerStateResponse :datatype-) "itm_nonlinear_mpc/GetControllerStateResponse")
(setf (get itm_nonlinear_mpc::GetControllerStateResponse :definition-)
      "string robot_name
uint8 command_id
---
bool connected

")



(provide :itm_nonlinear_mpc/GetControllerState "5ded73ff38c3ac0ab83d4d33c5cfb2fd")


