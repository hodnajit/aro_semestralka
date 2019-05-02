
(cl:in-package :asdf)

(defsystem "dummy_grid-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "ValuePoint" :depends-on ("_package_ValuePoint"))
    (:file "_package_ValuePoint" :depends-on ("_package"))
  ))