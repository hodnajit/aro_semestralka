;; Auto-generated. Do not edit!


(when (boundp 'dummy_grid::DrawGrid)
  (if (not (find-package "DUMMY_GRID"))
    (make-package "DUMMY_GRID"))
  (shadow 'DrawGrid (find-package "DUMMY_GRID")))
(unless (find-package "DUMMY_GRID::DRAWGRID")
  (make-package "DUMMY_GRID::DRAWGRID"))
(unless (find-package "DUMMY_GRID::DRAWGRIDREQUEST")
  (make-package "DUMMY_GRID::DRAWGRIDREQUEST"))
(unless (find-package "DUMMY_GRID::DRAWGRIDRESPONSE")
  (make-package "DUMMY_GRID::DRAWGRIDRESPONSE"))

(in-package "ROS")





(defclass dummy_grid::DrawGridRequest
  :super ros::object
  :slots (_points ))

(defmethod dummy_grid::DrawGridRequest
  (:init
   (&key
    ((:points __points) (let (r) (dotimes (i 0) (push (instance dummy_grid::ValuePoint :init) r)) r))
    )
   (send-super :init)
   (setq _points __points)
   self)
  (:points
   (&rest __points)
   (if (keywordp (car __points))
       (send* _points __points)
     (progn
       (if __points (setq _points (car __points)))
       _points)))
  (:serialization-length
   ()
   (+
    ;; dummy_grid/ValuePoint[] _points
    (apply #'+ (send-all _points :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; dummy_grid/ValuePoint[] _points
     (write-long (length _points) s)
     (dolist (elem _points)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; dummy_grid/ValuePoint[] _points
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _points (let (r) (dotimes (i n) (push (instance dummy_grid::ValuePoint :init) r)) r))
     (dolist (elem- _points)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(defclass dummy_grid::DrawGridResponse
  :super ros::object
  :slots ())

(defmethod dummy_grid::DrawGridResponse
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

(defclass dummy_grid::DrawGrid
  :super ros::object
  :slots ())

(setf (get dummy_grid::DrawGrid :md5sum-) "58bbaa800dd9242d5466364decd790f6")
(setf (get dummy_grid::DrawGrid :datatype-) "dummy_grid/DrawGrid")
(setf (get dummy_grid::DrawGrid :request) dummy_grid::DrawGridRequest)
(setf (get dummy_grid::DrawGrid :response) dummy_grid::DrawGridResponse)

(defmethod dummy_grid::DrawGridRequest
  (:response () (instance dummy_grid::DrawGridResponse :init)))

(setf (get dummy_grid::DrawGridRequest :md5sum-) "58bbaa800dd9242d5466364decd790f6")
(setf (get dummy_grid::DrawGridRequest :datatype-) "dummy_grid/DrawGridRequest")
(setf (get dummy_grid::DrawGridRequest :definition-)
      "dummy_grid/ValuePoint[] points

================================================================================
MSG: dummy_grid/ValuePoint
int16 x
int16 y
int8 value
---
")

(setf (get dummy_grid::DrawGridResponse :md5sum-) "58bbaa800dd9242d5466364decd790f6")
(setf (get dummy_grid::DrawGridResponse :datatype-) "dummy_grid/DrawGridResponse")
(setf (get dummy_grid::DrawGridResponse :definition-)
      "dummy_grid/ValuePoint[] points

================================================================================
MSG: dummy_grid/ValuePoint
int16 x
int16 y
int8 value
---
")



(provide :dummy_grid/DrawGrid "58bbaa800dd9242d5466364decd790f6")


