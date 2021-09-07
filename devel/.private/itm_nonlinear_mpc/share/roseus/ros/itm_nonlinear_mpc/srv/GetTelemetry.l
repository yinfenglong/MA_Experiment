;; Auto-generated. Do not edit!


(when (boundp 'itm_nonlinear_mpc::GetTelemetry)
  (if (not (find-package "ITM_NONLINEAR_MPC"))
    (make-package "ITM_NONLINEAR_MPC"))
  (shadow 'GetTelemetry (find-package "ITM_NONLINEAR_MPC")))
(unless (find-package "ITM_NONLINEAR_MPC::GETTELEMETRY")
  (make-package "ITM_NONLINEAR_MPC::GETTELEMETRY"))
(unless (find-package "ITM_NONLINEAR_MPC::GETTELEMETRYREQUEST")
  (make-package "ITM_NONLINEAR_MPC::GETTELEMETRYREQUEST"))
(unless (find-package "ITM_NONLINEAR_MPC::GETTELEMETRYRESPONSE")
  (make-package "ITM_NONLINEAR_MPC::GETTELEMETRYRESPONSE"))

(in-package "ROS")





(defclass itm_nonlinear_mpc::GetTelemetryRequest
  :super ros::object
  :slots (_frame_id ))

(defmethod itm_nonlinear_mpc::GetTelemetryRequest
  (:init
   (&key
    ((:frame_id __frame_id) "")
    )
   (send-super :init)
   (setq _frame_id (string __frame_id))
   self)
  (:frame_id
   (&optional __frame_id)
   (if __frame_id (setq _frame_id __frame_id)) _frame_id)
  (:serialization-length
   ()
   (+
    ;; string _frame_id
    4 (length _frame_id)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _frame_id
       (write-long (length _frame_id) s) (princ _frame_id s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _frame_id
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _frame_id (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass itm_nonlinear_mpc::GetTelemetryResponse
  :super ros::object
  :slots (_frame_id _connected _armed _mode _x _y _z _lat _lon _alt _vx _vy _vz _pitch _roll _yaw _pitch_rate _roll_rate _yaw_rate _voltage _cell_voltage ))

(defmethod itm_nonlinear_mpc::GetTelemetryResponse
  (:init
   (&key
    ((:frame_id __frame_id) "")
    ((:connected __connected) nil)
    ((:armed __armed) nil)
    ((:mode __mode) "")
    ((:x __x) 0.0)
    ((:y __y) 0.0)
    ((:z __z) 0.0)
    ((:lat __lat) 0.0)
    ((:lon __lon) 0.0)
    ((:alt __alt) 0.0)
    ((:vx __vx) 0.0)
    ((:vy __vy) 0.0)
    ((:vz __vz) 0.0)
    ((:pitch __pitch) 0.0)
    ((:roll __roll) 0.0)
    ((:yaw __yaw) 0.0)
    ((:pitch_rate __pitch_rate) 0.0)
    ((:roll_rate __roll_rate) 0.0)
    ((:yaw_rate __yaw_rate) 0.0)
    ((:voltage __voltage) 0.0)
    ((:cell_voltage __cell_voltage) 0.0)
    )
   (send-super :init)
   (setq _frame_id (string __frame_id))
   (setq _connected __connected)
   (setq _armed __armed)
   (setq _mode (string __mode))
   (setq _x (float __x))
   (setq _y (float __y))
   (setq _z (float __z))
   (setq _lat (float __lat))
   (setq _lon (float __lon))
   (setq _alt (float __alt))
   (setq _vx (float __vx))
   (setq _vy (float __vy))
   (setq _vz (float __vz))
   (setq _pitch (float __pitch))
   (setq _roll (float __roll))
   (setq _yaw (float __yaw))
   (setq _pitch_rate (float __pitch_rate))
   (setq _roll_rate (float __roll_rate))
   (setq _yaw_rate (float __yaw_rate))
   (setq _voltage (float __voltage))
   (setq _cell_voltage (float __cell_voltage))
   self)
  (:frame_id
   (&optional __frame_id)
   (if __frame_id (setq _frame_id __frame_id)) _frame_id)
  (:connected
   (&optional __connected)
   (if __connected (setq _connected __connected)) _connected)
  (:armed
   (&optional __armed)
   (if __armed (setq _armed __armed)) _armed)
  (:mode
   (&optional __mode)
   (if __mode (setq _mode __mode)) _mode)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:z
   (&optional __z)
   (if __z (setq _z __z)) _z)
  (:lat
   (&optional __lat)
   (if __lat (setq _lat __lat)) _lat)
  (:lon
   (&optional __lon)
   (if __lon (setq _lon __lon)) _lon)
  (:alt
   (&optional __alt)
   (if __alt (setq _alt __alt)) _alt)
  (:vx
   (&optional __vx)
   (if __vx (setq _vx __vx)) _vx)
  (:vy
   (&optional __vy)
   (if __vy (setq _vy __vy)) _vy)
  (:vz
   (&optional __vz)
   (if __vz (setq _vz __vz)) _vz)
  (:pitch
   (&optional __pitch)
   (if __pitch (setq _pitch __pitch)) _pitch)
  (:roll
   (&optional __roll)
   (if __roll (setq _roll __roll)) _roll)
  (:yaw
   (&optional __yaw)
   (if __yaw (setq _yaw __yaw)) _yaw)
  (:pitch_rate
   (&optional __pitch_rate)
   (if __pitch_rate (setq _pitch_rate __pitch_rate)) _pitch_rate)
  (:roll_rate
   (&optional __roll_rate)
   (if __roll_rate (setq _roll_rate __roll_rate)) _roll_rate)
  (:yaw_rate
   (&optional __yaw_rate)
   (if __yaw_rate (setq _yaw_rate __yaw_rate)) _yaw_rate)
  (:voltage
   (&optional __voltage)
   (if __voltage (setq _voltage __voltage)) _voltage)
  (:cell_voltage
   (&optional __cell_voltage)
   (if __cell_voltage (setq _cell_voltage __cell_voltage)) _cell_voltage)
  (:serialization-length
   ()
   (+
    ;; string _frame_id
    4 (length _frame_id)
    ;; bool _connected
    1
    ;; bool _armed
    1
    ;; string _mode
    4 (length _mode)
    ;; float32 _x
    4
    ;; float32 _y
    4
    ;; float32 _z
    4
    ;; float64 _lat
    8
    ;; float64 _lon
    8
    ;; float32 _alt
    4
    ;; float32 _vx
    4
    ;; float32 _vy
    4
    ;; float32 _vz
    4
    ;; float32 _pitch
    4
    ;; float32 _roll
    4
    ;; float32 _yaw
    4
    ;; float32 _pitch_rate
    4
    ;; float32 _roll_rate
    4
    ;; float32 _yaw_rate
    4
    ;; float32 _voltage
    4
    ;; float32 _cell_voltage
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _frame_id
       (write-long (length _frame_id) s) (princ _frame_id s)
     ;; bool _connected
       (if _connected (write-byte -1 s) (write-byte 0 s))
     ;; bool _armed
       (if _armed (write-byte -1 s) (write-byte 0 s))
     ;; string _mode
       (write-long (length _mode) s) (princ _mode s)
     ;; float32 _x
       (sys::poke _x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _y
       (sys::poke _y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _z
       (sys::poke _z (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float64 _lat
       (sys::poke _lat (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _lon
       (sys::poke _lon (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float32 _alt
       (sys::poke _alt (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _vx
       (sys::poke _vx (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _vy
       (sys::poke _vy (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _vz
       (sys::poke _vz (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _pitch
       (sys::poke _pitch (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _roll
       (sys::poke _roll (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _yaw
       (sys::poke _yaw (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _pitch_rate
       (sys::poke _pitch_rate (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _roll_rate
       (sys::poke _roll_rate (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _yaw_rate
       (sys::poke _yaw_rate (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _voltage
       (sys::poke _voltage (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _cell_voltage
       (sys::poke _cell_voltage (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _frame_id
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _frame_id (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; bool _connected
     (setq _connected (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _armed
     (setq _armed (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _mode
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _mode (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float32 _x
     (setq _x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _y
     (setq _y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _z
     (setq _z (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float64 _lat
     (setq _lat (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _lon
     (setq _lon (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float32 _alt
     (setq _alt (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _vx
     (setq _vx (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _vy
     (setq _vy (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _vz
     (setq _vz (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _pitch
     (setq _pitch (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _roll
     (setq _roll (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _yaw
     (setq _yaw (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _pitch_rate
     (setq _pitch_rate (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _roll_rate
     (setq _roll_rate (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _yaw_rate
     (setq _yaw_rate (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _voltage
     (setq _voltage (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _cell_voltage
     (setq _cell_voltage (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(defclass itm_nonlinear_mpc::GetTelemetry
  :super ros::object
  :slots ())

(setf (get itm_nonlinear_mpc::GetTelemetry :md5sum-) "3f1c3a9f95e1194fe8ebc671ce98ef91")
(setf (get itm_nonlinear_mpc::GetTelemetry :datatype-) "itm_nonlinear_mpc/GetTelemetry")
(setf (get itm_nonlinear_mpc::GetTelemetry :request) itm_nonlinear_mpc::GetTelemetryRequest)
(setf (get itm_nonlinear_mpc::GetTelemetry :response) itm_nonlinear_mpc::GetTelemetryResponse)

(defmethod itm_nonlinear_mpc::GetTelemetryRequest
  (:response () (instance itm_nonlinear_mpc::GetTelemetryResponse :init)))

(setf (get itm_nonlinear_mpc::GetTelemetryRequest :md5sum-) "3f1c3a9f95e1194fe8ebc671ce98ef91")
(setf (get itm_nonlinear_mpc::GetTelemetryRequest :datatype-) "itm_nonlinear_mpc/GetTelemetryRequest")
(setf (get itm_nonlinear_mpc::GetTelemetryRequest :definition-)
      "string frame_id
---
string frame_id
bool connected
bool armed
string mode
float32 x
float32 y
float32 z
float64 lat
float64 lon
float32 alt
float32 vx
float32 vy
float32 vz
float32 pitch
float32 roll
float32 yaw
float32 pitch_rate
float32 roll_rate
float32 yaw_rate
float32 voltage
float32 cell_voltage

")

(setf (get itm_nonlinear_mpc::GetTelemetryResponse :md5sum-) "3f1c3a9f95e1194fe8ebc671ce98ef91")
(setf (get itm_nonlinear_mpc::GetTelemetryResponse :datatype-) "itm_nonlinear_mpc/GetTelemetryResponse")
(setf (get itm_nonlinear_mpc::GetTelemetryResponse :definition-)
      "string frame_id
---
string frame_id
bool connected
bool armed
string mode
float32 x
float32 y
float32 z
float64 lat
float64 lon
float32 alt
float32 vx
float32 vy
float32 vz
float32 pitch
float32 roll
float32 yaw
float32 pitch_rate
float32 roll_rate
float32 yaw_rate
float32 voltage
float32 cell_voltage

")



(provide :itm_nonlinear_mpc/GetTelemetry "3f1c3a9f95e1194fe8ebc671ce98ef91")


