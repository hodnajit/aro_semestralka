;; Auto-generated. Do not edit!


(when (boundp 'dummy_grid::ValuePoint)
  (if (not (find-package "DUMMY_GRID"))
    (make-package "DUMMY_GRID"))
  (shadow 'ValuePoint (find-package "DUMMY_GRID")))
(unless (find-package "DUMMY_GRID::VALUEPOINT")
  (make-package "DUMMY_GRID::VALUEPOINT"))

(in-package "ROS")
;;//! \htmlinclude ValuePoint.msg.html


(defclass dummy_grid::ValuePoint
  :super ros::object
  :slots (_x _y _value ))

(defmethod dummy_grid::ValuePoint
  (:init
   (&key
    ((:x __x) 0)
    ((:y __y) 0)
    ((:value __value) 0)
    )
   (send-super :init)
   (setq _x (round __x))
   (setq _y (round __y))
   (setq _value (round __value))
   self)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:value
   (&optional __value)
   (if __value (setq _value __value)) _value)
  (:serialization-length
   ()
   (+
    ;; int16 _x
    2
    ;; int16 _y
    2
    ;; int8 _value
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int16 _x
       (write-word _x s)
     ;; int16 _y
       (write-word _y s)
     ;; int8 _value
       (write-byte _value s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int16 _x
     (setq _x (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _y
     (setq _y (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int8 _value
     (setq _value (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _value 127) (setq _value (- _value 256)))
   ;;
   self)
  )

(setf (get dummy_grid::ValuePoint :md5sum-) "f251dc5cdacac9c0ea3ef6f992e4d62a")
(setf (get dummy_grid::ValuePoint :datatype-) "dummy_grid/ValuePoint")
(setf (get dummy_grid::ValuePoint :definition-)
      "int16 x
int16 y
int8 value

")



(provide :dummy_grid/ValuePoint "f251dc5cdacac9c0ea3ef6f992e4d62a")


