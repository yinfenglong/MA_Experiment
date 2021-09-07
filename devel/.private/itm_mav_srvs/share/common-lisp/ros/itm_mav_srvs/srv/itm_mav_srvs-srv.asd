
(cl:in-package :asdf)

(defsystem "itm_mav_srvs-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :itm_mav_msgs-msg
)
  :components ((:file "_package")
    (:file "GetControllerState" :depends-on ("_package_GetControllerState"))
    (:file "_package_GetControllerState" :depends-on ("_package"))
    (:file "SetMode" :depends-on ("_package_SetMode"))
    (:file "_package_SetMode" :depends-on ("_package"))
    (:file "itm_trajectory_srv" :depends-on ("_package_itm_trajectory_srv"))
    (:file "_package_itm_trajectory_srv" :depends-on ("_package"))
    (:file "mpc_set_point_pos" :depends-on ("_package_mpc_set_point_pos"))
    (:file "_package_mpc_set_point_pos" :depends-on ("_package"))
  ))