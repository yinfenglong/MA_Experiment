;; Auto-generated. Do not edit!


(when (boundp 'itm_mav_msgs::UavState)
  (if (not (find-package "ITM_MAV_MSGS"))
    (make-package "ITM_MAV_MSGS"))
  (shadow 'UavState (find-package "ITM_MAV_MSGS")))
(unless (find-package "ITM_MAV_MSGS::UAVSTATE")
  (make-package "ITM_MAV_MSGS::UAVSTATE"))

(in-package "ROS")
;;//! \htmlinclude UavState.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass itm_mav_msgs::UavState
  :super ros::object
  :slots (_header _child_frame_id _estimator_iteration _pose _velocity _acceleration _acceleration_disturbance ))

(defmethod itm_mav_msgs::UavState
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:child_frame_id __child_frame_id) "")
    ((:estimator_iteration __estimator_iteration) 0)
    ((:pose __pose) (instance geometry_msgs::Pose :init))
    ((:velocity __velocity) (instance geometry_msgs::Twist :init))
    ((:acceleration __acceleration) (instance geometry_msgs::Accel :init))
    ((:acceleration_disturbance __acceleration_disturbance) (instance geometry_msgs::Accel :init))
    )
   (send-super :init)
   (setq _header __header)
   (setq _child_frame_id (string __child_frame_id))
   (setq _estimator_iteration (round __estimator_iteration))
   (setq _pose __pose)
   (setq _velocity __velocity)
   (setq _acceleration __acceleration)
   (setq _acceleration_disturbance __acceleration_disturbance)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:child_frame_id
   (&optional __child_frame_id)
   (if __child_frame_id (setq _child_frame_id __child_frame_id)) _child_frame_id)
  (:estimator_iteration
   (&optional __estimator_iteration)
   (if __estimator_iteration (setq _estimator_iteration __estimator_iteration)) _estimator_iteration)
  (:pose
   (&rest __pose)
   (if (keywordp (car __pose))
       (send* _pose __pose)
     (progn
       (if __pose (setq _pose (car __pose)))
       _pose)))
  (:velocity
   (&rest __velocity)
   (if (keywordp (car __velocity))
       (send* _velocity __velocity)
     (progn
       (if __velocity (setq _velocity (car __velocity)))
       _velocity)))
  (:acceleration
   (&rest __acceleration)
   (if (keywordp (car __acceleration))
       (send* _acceleration __acceleration)
     (progn
       (if __acceleration (setq _acceleration (car __acceleration)))
       _acceleration)))
  (:acceleration_disturbance
   (&rest __acceleration_disturbance)
   (if (keywordp (car __acceleration_disturbance))
       (send* _acceleration_disturbance __acceleration_disturbance)
     (progn
       (if __acceleration_disturbance (setq _acceleration_disturbance (car __acceleration_disturbance)))
       _acceleration_disturbance)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; string _child_frame_id
    4 (length _child_frame_id)
    ;; uint32 _estimator_iteration
    4
    ;; geometry_msgs/Pose _pose
    (send _pose :serialization-length)
    ;; geometry_msgs/Twist _velocity
    (send _velocity :serialization-length)
    ;; geometry_msgs/Accel _acceleration
    (send _acceleration :serialization-length)
    ;; geometry_msgs/Accel _acceleration_disturbance
    (send _acceleration_disturbance :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; string _child_frame_id
       (write-long (length _child_frame_id) s) (princ _child_frame_id s)
     ;; uint32 _estimator_iteration
       (write-long _estimator_iteration s)
     ;; geometry_msgs/Pose _pose
       (send _pose :serialize s)
     ;; geometry_msgs/Twist _velocity
       (send _velocity :serialize s)
     ;; geometry_msgs/Accel _acceleration
       (send _acceleration :serialize s)
     ;; geometry_msgs/Accel _acceleration_disturbance
       (send _acceleration_disturbance :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; string _child_frame_id
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _child_frame_id (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; uint32 _estimator_iteration
     (setq _estimator_iteration (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; geometry_msgs/Pose _pose
     (send _pose :deserialize buf ptr-) (incf ptr- (send _pose :serialization-length))
   ;; geometry_msgs/Twist _velocity
     (send _velocity :deserialize buf ptr-) (incf ptr- (send _velocity :serialization-length))
   ;; geometry_msgs/Accel _acceleration
     (send _acceleration :deserialize buf ptr-) (incf ptr- (send _acceleration :serialization-length))
   ;; geometry_msgs/Accel _acceleration_disturbance
     (send _acceleration_disturbance :deserialize buf ptr-) (incf ptr- (send _acceleration_disturbance :serialization-length))
   ;;
   self)
  )

(setf (get itm_mav_msgs::UavState :md5sum-) "3670a7f01f0a7e5bdc5175d2850230a3")
(setf (get itm_mav_msgs::UavState :datatype-) "itm_mav_msgs/UavState")
(setf (get itm_mav_msgs::UavState :definition-)
      "std_msgs/Header header
string child_frame_id

# # text names of the used estimators
# mrs_msgs/EstimatorType estimator_horizontal
# mrs_msgs/AltitudeType estimator_vertical
# mrs_msgs/HeadingType estimator_heading

# initiated with 0
# increases if any estimator changes frame of reference
# or if any estimator is resetted
uint32 estimator_iteration

geometry_msgs/Pose pose
geometry_msgs/Twist velocity
geometry_msgs/Accel acceleration
geometry_msgs/Accel acceleration_disturbance
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
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

================================================================================
MSG: geometry_msgs/Twist
# This expresses velocity in free space broken into its linear and angular parts.
Vector3  linear
Vector3  angular

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Accel
# This expresses acceleration in free space broken into its linear and angular parts.
Vector3  linear
Vector3  angular

")



(provide :itm_mav_msgs/UavState "3670a7f01f0a7e5bdc5175d2850230a3")


