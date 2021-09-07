;; Auto-generated. Do not edit!


(when (boundp 'itm_mav_srvs::GetControllerState)
  (if (not (find-package "ITM_MAV_SRVS"))
    (make-package "ITM_MAV_SRVS"))
  (shadow 'GetControllerState (find-package "ITM_MAV_SRVS")))
(unless (find-package "ITM_MAV_SRVS::GETCONTROLLERSTATE")
  (make-package "ITM_MAV_SRVS::GETCONTROLLERSTATE"))
(unless (find-package "ITM_MAV_SRVS::GETCONTROLLERSTATEREQUEST")
  (make-package "ITM_MAV_SRVS::GETCONTROLLERSTATEREQUEST"))
(unless (find-package "ITM_MAV_SRVS::GETCONTROLLERSTATERESPONSE")
  (make-package "ITM_MAV_SRVS::GETCONTROLLERSTATERESPONSE"))

(in-package "ROS")





(defclass itm_mav_srvs::GetControllerStateRequest
  :super ros::object
  :slots (_robot_name _command_id ))

(defmethod itm_mav_srvs::GetControllerStateRequest
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

(defclass itm_mav_srvs::GetControllerStateResponse
  :super ros::object
  :slots (_connected ))

(defmethod itm_mav_srvs::GetControllerStateResponse
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

(defclass itm_mav_srvs::GetControllerState
  :super ros::object
  :slots ())

(setf (get itm_mav_srvs::GetControllerState :md5sum-) "5ded73ff38c3ac0ab83d4d33c5cfb2fd")
(setf (get itm_mav_srvs::GetControllerState :datatype-) "itm_mav_srvs/GetControllerState")
(setf (get itm_mav_srvs::GetControllerState :request) itm_mav_srvs::GetControllerStateRequest)
(setf (get itm_mav_srvs::GetControllerState :response) itm_mav_srvs::GetControllerStateResponse)

(defmethod itm_mav_srvs::GetControllerStateRequest
  (:response () (instance itm_mav_srvs::GetControllerStateResponse :init)))

(setf (get itm_mav_srvs::GetControllerStateRequest :md5sum-) "5ded73ff38c3ac0ab83d4d33c5cfb2fd")
(setf (get itm_mav_srvs::GetControllerStateRequest :datatype-) "itm_mav_srvs/GetControllerStateRequest")
(setf (get itm_mav_srvs::GetControllerStateRequest :definition-)
      "string robot_name
uint8 command_id
---
bool connected

")

(setf (get itm_mav_srvs::GetControllerStateResponse :md5sum-) "5ded73ff38c3ac0ab83d4d33c5cfb2fd")
(setf (get itm_mav_srvs::GetControllerStateResponse :datatype-) "itm_mav_srvs/GetControllerStateResponse")
(setf (get itm_mav_srvs::GetControllerStateResponse :definition-)
      "string robot_name
uint8 command_id
---
bool connected

")



(provide :itm_mav_srvs/GetControllerState "5ded73ff38c3ac0ab83d4d33c5cfb2fd")


