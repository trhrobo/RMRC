
(cl:in-package :asdf)

(defsystem "dozap_second-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Button" :depends-on ("_package_Button"))
    (:file "_package_Button" :depends-on ("_package"))
    (:file "Main" :depends-on ("_package_Main"))
    (:file "_package_Main" :depends-on ("_package"))
  ))