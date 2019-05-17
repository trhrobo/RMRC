;; Auto-generated. Do not edit!


(when (boundp 'dozap_second::Main)
  (if (not (find-package "DOZAP_SECOND"))
    (make-package "DOZAP_SECOND"))
  (shadow 'Main (find-package "DOZAP_SECOND")))
(unless (find-package "DOZAP_SECOND::MAIN")
  (make-package "DOZAP_SECOND::MAIN"))

(in-package "ROS")
;;//! \htmlinclude Main.msg.html


(defclass dozap_second::Main
  :super ros::object
  :slots (_motor_right _motor_left _rotation_right _rotation_left ))

(defmethod dozap_second::Main
  (:init
   (&key
    ((:motor_right __motor_right) 0)
    ((:motor_left __motor_left) 0)
    ((:rotation_right __rotation_right) 0)
    ((:rotation_left __rotation_left) 0)
    )
   (send-super :init)
   (setq _motor_right (round __motor_right))
   (setq _motor_left (round __motor_left))
   (setq _rotation_right (round __rotation_right))
   (setq _rotation_left (round __rotation_left))
   self)
  (:motor_right
   (&optional __motor_right)
   (if __motor_right (setq _motor_right __motor_right)) _motor_right)
  (:motor_left
   (&optional __motor_left)
   (if __motor_left (setq _motor_left __motor_left)) _motor_left)
  (:rotation_right
   (&optional __rotation_right)
   (if __rotation_right (setq _rotation_right __rotation_right)) _rotation_right)
  (:rotation_left
   (&optional __rotation_left)
   (if __rotation_left (setq _rotation_left __rotation_left)) _rotation_left)
  (:serialization-length
   ()
   (+
    ;; int32 _motor_right
    4
    ;; int32 _motor_left
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
     ;; int32 _motor_right
       (write-long _motor_right s)
     ;; int32 _motor_left
       (write-long _motor_left s)
     ;; int32 _rotation_right
       (write-long _rotation_right s)
     ;; int32 _rotation_left
       (write-long _rotation_left s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _motor_right
     (setq _motor_right (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _motor_left
     (setq _motor_left (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _rotation_right
     (setq _rotation_right (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _rotation_left
     (setq _rotation_left (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get dozap_second::Main :md5sum-) "7a83861f39451721eaac1e365039256f")
(setf (get dozap_second::Main :datatype-) "dozap_second/Main")
(setf (get dozap_second::Main :definition-)
      "int32 motor_right
int32 motor_left
int32 rotation_right
int32 rotation_left

")



(provide :dozap_second/Main "7a83861f39451721eaac1e365039256f")


