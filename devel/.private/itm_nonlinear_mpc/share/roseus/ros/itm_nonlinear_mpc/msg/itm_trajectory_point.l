;; Auto-generated. Do not edit!


(when (boundp 'itm_nonlinear_mpc::itm_trajectory_point)
  (if (not (find-package "ITM_NONLINEAR_MPC"))
    (make-package "ITM_NONLINEAR_MPC"))
  (shadow 'itm_trajectory_point (find-package "ITM_NONLINEAR_MPC")))
(unless (find-package "ITM_NONLINEAR_MPC::ITM_TRAJECTORY_POINT")
  (make-package "ITM_NONLINEAR_MPC::ITM_TRAJECTORY_POINT"))

(in-package "ROS")
;;//! \htmlinclude itm_trajectory_point.msg.html


(defclass itm_nonlinear_mpc::itm_trajectory_point
  :super ros::object
  :slots (_x _y _z _vx _vy _vz _roll _pitch _yaw _q _cube_x _cube_y _cube_z _cube_yaw _fixed _time_known _derivative _segment_index _time_stamp _quaternion_given ))

(defmethod itm_nonlinear_mpc::itm_trajectory_point
  (:init
   (&key
    ((:x __x) 0.0)
    ((:y __y) 0.0)
    ((:z __z) 0.0)
    ((:vx __vx) 0.0)
    ((:vy __vy) 0.0)
    ((:vz __vz) 0.0)
    ((:roll __roll) 0.0)
    ((:pitch __pitch) 0.0)
    ((:yaw __yaw) 0.0)
    ((:q __q) (make-array 4 :initial-element 0.0 :element-type :float))
    ((:cube_x __cube_x) (make-array 2 :initial-element 0.0 :element-type :float))
    ((:cube_y __cube_y) (make-array 2 :initial-element 0.0 :element-type :float))
    ((:cube_z __cube_z) (make-array 2 :initial-element 0.0 :element-type :float))
    ((:cube_yaw __cube_yaw) (make-array 2 :initial-element 0.0 :element-type :float))
    ((:fixed __fixed) nil)
    ((:time_known __time_known) nil)
    ((:derivative __derivative) 0)
    ((:segment_index __segment_index) 0)
    ((:time_stamp __time_stamp) 0.0)
    ((:quaternion_given __quaternion_given) nil)
    )
   (send-super :init)
   (setq _x (float __x))
   (setq _y (float __y))
   (setq _z (float __z))
   (setq _vx (float __vx))
   (setq _vy (float __vy))
   (setq _vz (float __vz))
   (setq _roll (float __roll))
   (setq _pitch (float __pitch))
   (setq _yaw (float __yaw))
   (setq _q __q)
   (setq _cube_x __cube_x)
   (setq _cube_y __cube_y)
   (setq _cube_z __cube_z)
   (setq _cube_yaw __cube_yaw)
   (setq _fixed __fixed)
   (setq _time_known __time_known)
   (setq _derivative (round __derivative))
   (setq _segment_index (round __segment_index))
   (setq _time_stamp (float __time_stamp))
   (setq _quaternion_given __quaternion_given)
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
  (:vx
   (&optional __vx)
   (if __vx (setq _vx __vx)) _vx)
  (:vy
   (&optional __vy)
   (if __vy (setq _vy __vy)) _vy)
  (:vz
   (&optional __vz)
   (if __vz (setq _vz __vz)) _vz)
  (:roll
   (&optional __roll)
   (if __roll (setq _roll __roll)) _roll)
  (:pitch
   (&optional __pitch)
   (if __pitch (setq _pitch __pitch)) _pitch)
  (:yaw
   (&optional __yaw)
   (if __yaw (setq _yaw __yaw)) _yaw)
  (:q
   (&optional __q)
   (if __q (setq _q __q)) _q)
  (:cube_x
   (&optional __cube_x)
   (if __cube_x (setq _cube_x __cube_x)) _cube_x)
  (:cube_y
   (&optional __cube_y)
   (if __cube_y (setq _cube_y __cube_y)) _cube_y)
  (:cube_z
   (&optional __cube_z)
   (if __cube_z (setq _cube_z __cube_z)) _cube_z)
  (:cube_yaw
   (&optional __cube_yaw)
   (if __cube_yaw (setq _cube_yaw __cube_yaw)) _cube_yaw)
  (:fixed
   (&optional __fixed)
   (if __fixed (setq _fixed __fixed)) _fixed)
  (:time_known
   (&optional __time_known)
   (if __time_known (setq _time_known __time_known)) _time_known)
  (:derivative
   (&optional __derivative)
   (if __derivative (setq _derivative __derivative)) _derivative)
  (:segment_index
   (&optional __segment_index)
   (if __segment_index (setq _segment_index __segment_index)) _segment_index)
  (:time_stamp
   (&optional __time_stamp)
   (if __time_stamp (setq _time_stamp __time_stamp)) _time_stamp)
  (:quaternion_given
   (&optional __quaternion_given)
   (if __quaternion_given (setq _quaternion_given __quaternion_given)) _quaternion_given)
  (:serialization-length
   ()
   (+
    ;; float64 _x
    8
    ;; float64 _y
    8
    ;; float64 _z
    8
    ;; float64 _vx
    8
    ;; float64 _vy
    8
    ;; float64 _vz
    8
    ;; float64 _roll
    8
    ;; float64 _pitch
    8
    ;; float64 _yaw
    8
    ;; float64[4] _q
    (* 8    4)
    ;; float64[2] _cube_x
    (* 8    2)
    ;; float64[2] _cube_y
    (* 8    2)
    ;; float64[2] _cube_z
    (* 8    2)
    ;; float64[2] _cube_yaw
    (* 8    2)
    ;; bool _fixed
    1
    ;; bool _time_known
    1
    ;; int8 _derivative
    1
    ;; int8 _segment_index
    1
    ;; float64 _time_stamp
    8
    ;; bool _quaternion_given
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _x
       (sys::poke _x (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _y
       (sys::poke _y (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _z
       (sys::poke _z (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _vx
       (sys::poke _vx (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _vy
       (sys::poke _vy (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _vz
       (sys::poke _vz (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _roll
       (sys::poke _roll (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _pitch
       (sys::poke _pitch (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _yaw
       (sys::poke _yaw (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64[4] _q
     (dotimes (i 4)
       (sys::poke (elt _q i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[2] _cube_x
     (dotimes (i 2)
       (sys::poke (elt _cube_x i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[2] _cube_y
     (dotimes (i 2)
       (sys::poke (elt _cube_y i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[2] _cube_z
     (dotimes (i 2)
       (sys::poke (elt _cube_z i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[2] _cube_yaw
     (dotimes (i 2)
       (sys::poke (elt _cube_yaw i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; bool _fixed
       (if _fixed (write-byte -1 s) (write-byte 0 s))
     ;; bool _time_known
       (if _time_known (write-byte -1 s) (write-byte 0 s))
     ;; int8 _derivative
       (write-byte _derivative s)
     ;; int8 _segment_index
       (write-byte _segment_index s)
     ;; float64 _time_stamp
       (sys::poke _time_stamp (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; bool _quaternion_given
       (if _quaternion_given (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _x
     (setq _x (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _y
     (setq _y (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _z
     (setq _z (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _vx
     (setq _vx (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _vy
     (setq _vy (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _vz
     (setq _vz (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _roll
     (setq _roll (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _pitch
     (setq _pitch (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _yaw
     (setq _yaw (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64[4] _q
   (dotimes (i (length _q))
     (setf (elt _q i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[2] _cube_x
   (dotimes (i (length _cube_x))
     (setf (elt _cube_x i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[2] _cube_y
   (dotimes (i (length _cube_y))
     (setf (elt _cube_y i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[2] _cube_z
   (dotimes (i (length _cube_z))
     (setf (elt _cube_z i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[2] _cube_yaw
   (dotimes (i (length _cube_yaw))
     (setf (elt _cube_yaw i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; bool _fixed
     (setq _fixed (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _time_known
     (setq _time_known (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; int8 _derivative
     (setq _derivative (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _derivative 127) (setq _derivative (- _derivative 256)))
   ;; int8 _segment_index
     (setq _segment_index (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _segment_index 127) (setq _segment_index (- _segment_index 256)))
   ;; float64 _time_stamp
     (setq _time_stamp (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; bool _quaternion_given
     (setq _quaternion_given (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get itm_nonlinear_mpc::itm_trajectory_point :md5sum-) "8bc93db1142dc497f7fb791295f3f567")
(setf (get itm_nonlinear_mpc::itm_trajectory_point :datatype-) "itm_nonlinear_mpc/itm_trajectory_point")
(setf (get itm_nonlinear_mpc::itm_trajectory_point :definition-)
      "float64 x
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



(provide :itm_nonlinear_mpc/itm_trajectory_point "8bc93db1142dc497f7fb791295f3f567")


