
(cl:in-package :asdf)

(defsystem "itm_mav_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "AttitudeCommand" :depends-on ("_package_AttitudeCommand"))
    (:file "_package_AttitudeCommand" :depends-on ("_package"))
    (:file "PositionCommand" :depends-on ("_package_PositionCommand"))
    (:file "_package_PositionCommand" :depends-on ("_package"))
    (:file "SetMission" :depends-on ("_package_SetMission"))
    (:file "_package_SetMission" :depends-on ("_package"))
    (:file "UavState" :depends-on ("_package_UavState"))
    (:file "_package_UavState" :depends-on ("_package"))
    (:file "itm_trajectory_msg" :depends-on ("_package_itm_trajectory_msg"))
    (:file "_package_itm_trajectory_msg" :depends-on ("_package"))
    (:file "itm_trajectory_point" :depends-on ("_package_itm_trajectory_point"))
    (:file "_package_itm_trajectory_point" :depends-on ("_package"))
  ))