
(cl:in-package :asdf)

(defsystem "unitree_a1-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "MotorCmd" :depends-on ("_package_MotorCmd"))
    (:file "_package_MotorCmd" :depends-on ("_package"))
    (:file "MotorData" :depends-on ("_package_MotorData"))
    (:file "_package_MotorData" :depends-on ("_package"))
  ))