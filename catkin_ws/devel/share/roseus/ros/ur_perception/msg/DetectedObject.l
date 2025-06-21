;; Auto-generated. Do not edit!


(when (boundp 'ur_perception::DetectedObject)
  (if (not (find-package "UR_PERCEPTION"))
    (make-package "UR_PERCEPTION"))
  (shadow 'DetectedObject (find-package "UR_PERCEPTION")))
(unless (find-package "UR_PERCEPTION::DETECTEDOBJECT")
  (make-package "UR_PERCEPTION::DETECTEDOBJECT"))

(in-package "ROS")
;;//! \htmlinclude DetectedObject.msg.html


(defclass ur_perception::DetectedObject
  :super ros::object
  :slots (_label _confidence _x1 _y1 _x2 _y2 ))

(defmethod ur_perception::DetectedObject
  (:init
   (&key
    ((:label __label) "")
    ((:confidence __confidence) 0.0)
    ((:x1 __x1) 0)
    ((:y1 __y1) 0)
    ((:x2 __x2) 0)
    ((:y2 __y2) 0)
    )
   (send-super :init)
   (setq _label (string __label))
   (setq _confidence (float __confidence))
   (setq _x1 (round __x1))
   (setq _y1 (round __y1))
   (setq _x2 (round __x2))
   (setq _y2 (round __y2))
   self)
  (:label
   (&optional __label)
   (if __label (setq _label __label)) _label)
  (:confidence
   (&optional __confidence)
   (if __confidence (setq _confidence __confidence)) _confidence)
  (:x1
   (&optional __x1)
   (if __x1 (setq _x1 __x1)) _x1)
  (:y1
   (&optional __y1)
   (if __y1 (setq _y1 __y1)) _y1)
  (:x2
   (&optional __x2)
   (if __x2 (setq _x2 __x2)) _x2)
  (:y2
   (&optional __y2)
   (if __y2 (setq _y2 __y2)) _y2)
  (:serialization-length
   ()
   (+
    ;; string _label
    4 (length _label)
    ;; float32 _confidence
    4
    ;; int32 _x1
    4
    ;; int32 _y1
    4
    ;; int32 _x2
    4
    ;; int32 _y2
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _label
       (write-long (length _label) s) (princ _label s)
     ;; float32 _confidence
       (sys::poke _confidence (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; int32 _x1
       (write-long _x1 s)
     ;; int32 _y1
       (write-long _y1 s)
     ;; int32 _x2
       (write-long _x2 s)
     ;; int32 _y2
       (write-long _y2 s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _label
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _label (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float32 _confidence
     (setq _confidence (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; int32 _x1
     (setq _x1 (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _y1
     (setq _y1 (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _x2
     (setq _x2 (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _y2
     (setq _y2 (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get ur_perception::DetectedObject :md5sum-) "3c092459209ab249fb5c5c03a15ab9cb")
(setf (get ur_perception::DetectedObject :datatype-) "ur_perception/DetectedObject")
(setf (get ur_perception::DetectedObject :definition-)
      "string label
float32 confidence
int32 x1
int32 y1
int32 x2
int32 y2

")



(provide :ur_perception/DetectedObject "3c092459209ab249fb5c5c03a15ab9cb")


