
(cl:in-package :asdf)

(defsystem "damiao_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "DmCommand" :depends-on ("_package_DmCommand"))
    (:file "_package_DmCommand" :depends-on ("_package"))
    (:file "DmState" :depends-on ("_package_DmState"))
    (:file "_package_DmState" :depends-on ("_package"))
  ))