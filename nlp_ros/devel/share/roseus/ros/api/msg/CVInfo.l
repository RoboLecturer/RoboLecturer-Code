;; Auto-generated. Do not edit!


(when (boundp 'api::CVInfo)
  (if (not (find-package "API"))
    (make-package "API"))
  (shadow 'CVInfo (find-package "API")))
(unless (find-package "API::CVINFO")
  (make-package "API::CVINFO"))

(in-package "ROS")
;;//! \htmlinclude CVInfo.msg.html


(defclass api::CVInfo
  :super ros::object
  :slots (_x _y _w _h _frame_width _frame_height _score ))

(defmethod api::CVInfo
  (:init
   (&key
    ((:x __x) 0.0)
    ((:y __y) 0.0)
    ((:w __w) 0.0)
    ((:h __h) 0.0)
    ((:frame_width __frame_width) 0)
    ((:frame_height __frame_height) 0)
    ((:score __score) 0.0)
    )
   (send-super :init)
   (setq _x (float __x))
   (setq _y (float __y))
   (setq _w (float __w))
   (setq _h (float __h))
   (setq _frame_width (round __frame_width))
   (setq _frame_height (round __frame_height))
   (setq _score (float __score))
   self)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:w
   (&optional __w)
   (if __w (setq _w __w)) _w)
  (:h
   (&optional __h)
   (if __h (setq _h __h)) _h)
  (:frame_width
   (&optional __frame_width)
   (if __frame_width (setq _frame_width __frame_width)) _frame_width)
  (:frame_height
   (&optional __frame_height)
   (if __frame_height (setq _frame_height __frame_height)) _frame_height)
  (:score
   (&optional __score)
   (if __score (setq _score __score)) _score)
  (:serialization-length
   ()
   (+
    ;; float32 _x
    4
    ;; float32 _y
    4
    ;; float32 _w
    4
    ;; float32 _h
    4
    ;; uint16 _frame_width
    2
    ;; uint16 _frame_height
    2
    ;; float32 _score
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _x
       (sys::poke _x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _y
       (sys::poke _y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _w
       (sys::poke _w (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _h
       (sys::poke _h (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; uint16 _frame_width
       (write-word _frame_width s)
     ;; uint16 _frame_height
       (write-word _frame_height s)
     ;; float32 _score
       (sys::poke _score (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _x
     (setq _x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _y
     (setq _y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _w
     (setq _w (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _h
     (setq _h (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; uint16 _frame_width
     (setq _frame_width (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint16 _frame_height
     (setq _frame_height (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; float32 _score
     (setq _score (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get api::CVInfo :md5sum-) "99332bc3002971b2c3503a06ab6aa2f6")
(setf (get api::CVInfo :datatype-) "api/CVInfo")
(setf (get api::CVInfo :definition-)
      "float32 x
float32 y
float32 w
float32 h
uint16 frame_width
uint16 frame_height
float32 score

")



(provide :api/CVInfo "99332bc3002971b2c3503a06ab6aa2f6")


