;; Auto-generated. Do not edit!


(when (boundp 'exploration::AnyFrontiersLeft)
  (if (not (find-package "EXPLORATION"))
    (make-package "EXPLORATION"))
  (shadow 'AnyFrontiersLeft (find-package "EXPLORATION")))
(unless (find-package "EXPLORATION::ANYFRONTIERSLEFT")
  (make-package "EXPLORATION::ANYFRONTIERSLEFT"))
(unless (find-package "EXPLORATION::ANYFRONTIERSLEFTREQUEST")
  (make-package "EXPLORATION::ANYFRONTIERSLEFTREQUEST"))
(unless (find-package "EXPLORATION::ANYFRONTIERSLEFTRESPONSE")
  (make-package "EXPLORATION::ANYFRONTIERSLEFTRESPONSE"))

(in-package "ROS")





(defclass exploration::AnyFrontiersLeftRequest
  :super ros::object
  :slots ())

(defmethod exploration::AnyFrontiersLeftRequest
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(defclass exploration::AnyFrontiersLeftResponse
  :super ros::object
  :slots (_any_frontiers_left ))

(defmethod exploration::AnyFrontiersLeftResponse
  (:init
   (&key
    ((:any_frontiers_left __any_frontiers_left) nil)
    )
   (send-super :init)
   (setq _any_frontiers_left __any_frontiers_left)
   self)
  (:any_frontiers_left
   (&optional __any_frontiers_left)
   (if __any_frontiers_left (setq _any_frontiers_left __any_frontiers_left)) _any_frontiers_left)
  (:serialization-length
   ()
   (+
    ;; bool _any_frontiers_left
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _any_frontiers_left
       (if _any_frontiers_left (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _any_frontiers_left
     (setq _any_frontiers_left (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass exploration::AnyFrontiersLeft
  :super ros::object
  :slots ())

(setf (get exploration::AnyFrontiersLeft :md5sum-) "9c55a8e73f02e61ed1fd7718907efa64")
(setf (get exploration::AnyFrontiersLeft :datatype-) "exploration/AnyFrontiersLeft")
(setf (get exploration::AnyFrontiersLeft :request) exploration::AnyFrontiersLeftRequest)
(setf (get exploration::AnyFrontiersLeft :response) exploration::AnyFrontiersLeftResponse)

(defmethod exploration::AnyFrontiersLeftRequest
  (:response () (instance exploration::AnyFrontiersLeftResponse :init)))

(setf (get exploration::AnyFrontiersLeftRequest :md5sum-) "9c55a8e73f02e61ed1fd7718907efa64")
(setf (get exploration::AnyFrontiersLeftRequest :datatype-) "exploration/AnyFrontiersLeftRequest")
(setf (get exploration::AnyFrontiersLeftRequest :definition-)
      "---
bool any_frontiers_left
")

(setf (get exploration::AnyFrontiersLeftResponse :md5sum-) "9c55a8e73f02e61ed1fd7718907efa64")
(setf (get exploration::AnyFrontiersLeftResponse :datatype-) "exploration/AnyFrontiersLeftResponse")
(setf (get exploration::AnyFrontiersLeftResponse :definition-)
      "---
bool any_frontiers_left
")



(provide :exploration/AnyFrontiersLeft "9c55a8e73f02e61ed1fd7718907efa64")


