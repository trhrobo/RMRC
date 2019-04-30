;; Auto-generated. Do not edit!


(when (boundp 'dozap_second::Button)
  (if (not (find-package "DOZAP_SECOND"))
    (make-package "DOZAP_SECOND"))
  (shadow 'Button (find-package "DOZAP_SECOND")))
(unless (find-package "DOZAP_SECOND::BUTTON")
  (make-package "DOZAP_SECOND::BUTTON"))

(in-package "ROS")
;;//! \htmlinclude Button.msg.html


(defclass dozap_second::Button
  :super ros::object
  :slots (_move _rotation_right _rotation_left ))

(defmethod dozap_second::Button
  (:init
   (&key
    ((:move __move) 0)
    ((:rotation_right __rotation_right) 0)
    ((:rotation_left __rotation_left) 0)
    )
   (send-super :init)
   (setq _move (round __move))
   (setq _rotation_right (round __rotation_right))
   (setq _rotation_left (round __rotation_left))
   self)
  (:move
   (&optional __move)
   (if __move (setq _move __move)) _move)
  (:rotation_right
   (&optional __rotation_right)
   (if __rotation_right (setq _rotation_right __rotation_right)) _rotation_right)
  (:rotation_left
   (&optional __rotation_left)
   (if __rotation_left (setq _rotation_left __rotation_left)) _rotation_left)
  (:serialization-length
   ()
   (+
    ;; int32 _move
    4
    ;; int32 _rotation_right
    4
    ;; int32 _rotation_left
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _move
       (write-long _move s)
     ;; int32 _rotation_right
       (write-long _rotation_right s)
     ;; int32 _rotation_left
       (write-long _rotation_left s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _move
     (setq _move (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _rotation_right
     (setq _rotation_right (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _rotation_left
     (setq _rotation_left (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get dozap_second::Button :md5sum-) "df37f56eb5634ffffccd37fb25f1f170")
(setf (get dozap_second::Button :datatype-) "dozap_second/Button")
(setf (get dozap_second::Button :definition-)
      "int32 move
int32 rotation_right
int32 rotation_left

")



(provide :dozap_second/Button "df37f56eb5634ffffccd37fb25f1f170")


