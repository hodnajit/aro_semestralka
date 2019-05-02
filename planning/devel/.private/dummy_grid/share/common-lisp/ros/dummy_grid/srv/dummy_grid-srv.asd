
(cl:in-package :asdf)

(defsystem "dummy_grid-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :dummy_grid-msg
)
  :components ((:file "_package")
    (:file "DrawGrid" :depends-on ("_package_DrawGrid"))
    (:file "_package_DrawGrid" :depends-on ("_package"))
  ))