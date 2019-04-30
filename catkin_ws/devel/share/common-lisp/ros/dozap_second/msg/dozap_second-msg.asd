
(cl:in-package :asdf)

(defsystem "dozap_second-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Button" :depends-on ("_package_Button"))
    (:file "_package_Button" :depends-on ("_package"))
  ))