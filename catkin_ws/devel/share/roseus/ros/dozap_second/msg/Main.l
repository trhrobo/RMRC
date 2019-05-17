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
  :slots (_motor_right _motor_left _rotation_a_right _rotation_a_left _rotation_b_right _rotation_b_left ))

(defmethod dozap_second::Main
  (:init
   (&key
    ((:motor_right __motor_right) 0)
    ((:motor_left __motor_left) 0)
    ((:rotation_a_right __rotation_a_right) 0)
    ((:rotation_a_left __rotation_a_left) 0)
    ((:rotation_b_right __rotation_b_right) 0)
    ((:rotation_b_left __rotation_b_left) 0)
    )
   (send-super :init)
   (setq _motor_right (round __motor_right))
   (setq _motor_left (round __motor_left))
   (setq _rotation_a_right (round __rotation_a_right))
   (setq _rotation_a_left (round __rotation_a_left))
   (setq _rotation_b_right (round __rotation_b_right))
   (setq _rotation_b_left (round __rotation_b_left))
   self)
  (:motor_right
   (&optional __motor_right)
   (if __motor_right (setq _motor_right __motor_right)) _motor_right)
  (:motor_left
   (&optional __motor_left)
   (if __motor_left (setq _motor_left __motor_left)) _motor_left)
  (:rotation_a_right
   (&optional __rotation_a_right)
   (if __rotation_a_right (setq _rotation_a_right __rotation_a_right)) _rotation_a_right)
  (:rotation_a_left
   (&optional __rotation_a_left)
   (if __rotation_a_left (setq _rotation_a_left __rotation_a_left)) _rotation_a_left)
  (:rotation_b_right
   (&optional __rotation_b_right)
   (if __rotation_b_right (setq _rotation_b_right __rotation_b_right)) _rotation_b_right)
  (:rotation_b_left
   (&optional __rotation_b_left)
   (if __rotation_b_left (setq _rotation_b_left __rotation_b_left)) _rotation_b_left)
  (:serialization-length
   ()
   (+
    ;; int32 _motor_right
    4
    ;; int32 _motor_left
    4
    ;; int32 _rotation_a_right
    4
    ;; int32 _rotation_a_left
    4
    ;; int32 _rotation_b_right
    4
    ;; int32 _rotation_b_left
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
     ;; int32 _rotation_a_right
       (write-long _rotation_a_right s)
     ;; int32 _rotation_a_left
       (write-long _rotation_a_left s)
     ;; int32 _rotation_b_right
       (write-long _rotation_b_right s)
     ;; int32 _rotation_b_left
       (write-long _rotation_b_left s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _motor_right
     (setq _motor_right (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _motor_left
     (setq _motor_left (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _rotation_a_right
     (setq _rotation_a_right (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _rotation_a_left
     (setq _rotation_a_left (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _rotation_b_right
     (setq _rotation_b_right (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _rotation_b_left
     (setq _rotation_b_left (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get dozap_second::Main :md5sum-) "83aace50d71246340a8e7cc2d2789279")
(setf (get dozap_second::Main :datatype-) "dozap_second/Main")
(setf (get dozap_second::Main :definition-)
      "int32 motor_right
int32 motor_left
int32 rotation_a_right
int32 rotation_a_left
int32 rotation_b_right
int32 rotation_b_left

")



(provide :dozap_second/Main "83aace50d71246340a8e7cc2d2789279")


