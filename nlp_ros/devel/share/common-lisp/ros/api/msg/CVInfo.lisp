; Auto-generated. Do not edit!


(cl:in-package api-msg)


;//! \htmlinclude CVInfo.msg.html

(cl:defclass <CVInfo> (roslisp-msg-protocol:ros-message)
  ((x
    :reader x
    :initarg :x
    :type cl:float
    :initform 0.0)
   (y
    :reader y
    :initarg :y
    :type cl:float
    :initform 0.0)
   (w
    :reader w
    :initarg :w
    :type cl:float
    :initform 0.0)
   (h
    :reader h
    :initarg :h
    :type cl:float
    :initform 0.0)
   (frame_width
    :reader frame_width
    :initarg :frame_width
    :type cl:fixnum
    :initform 0)
   (frame_height
    :reader frame_height
    :initarg :frame_height
    :type cl:fixnum
    :initform 0)
   (score
    :reader score
    :initarg :score
    :type cl:float
    :initform 0.0))
)

(cl:defclass CVInfo (<CVInfo>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CVInfo>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CVInfo)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name api-msg:<CVInfo> is deprecated: use api-msg:CVInfo instead.")))

(cl:ensure-generic-function 'x-val :lambda-list '(m))
(cl:defmethod x-val ((m <CVInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader api-msg:x-val is deprecated.  Use api-msg:x instead.")
  (x m))

(cl:ensure-generic-function 'y-val :lambda-list '(m))
(cl:defmethod y-val ((m <CVInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader api-msg:y-val is deprecated.  Use api-msg:y instead.")
  (y m))

(cl:ensure-generic-function 'w-val :lambda-list '(m))
(cl:defmethod w-val ((m <CVInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader api-msg:w-val is deprecated.  Use api-msg:w instead.")
  (w m))

(cl:ensure-generic-function 'h-val :lambda-list '(m))
(cl:defmethod h-val ((m <CVInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader api-msg:h-val is deprecated.  Use api-msg:h instead.")
  (h m))

(cl:ensure-generic-function 'frame_width-val :lambda-list '(m))
(cl:defmethod frame_width-val ((m <CVInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader api-msg:frame_width-val is deprecated.  Use api-msg:frame_width instead.")
  (frame_width m))

(cl:ensure-generic-function 'frame_height-val :lambda-list '(m))
(cl:defmethod frame_height-val ((m <CVInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader api-msg:frame_height-val is deprecated.  Use api-msg:frame_height instead.")
  (frame_height m))

(cl:ensure-generic-function 'score-val :lambda-list '(m))
(cl:defmethod score-val ((m <CVInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader api-msg:score-val is deprecated.  Use api-msg:score instead.")
  (score m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CVInfo>) ostream)
  "Serializes a message object of type '<CVInfo>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'w))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'h))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'frame_width)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'frame_width)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'frame_height)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'frame_height)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'score))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CVInfo>) istream)
  "Deserializes a message object of type '<CVInfo>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'w) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'h) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'frame_width)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'frame_width)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'frame_height)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'frame_height)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'score) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CVInfo>)))
  "Returns string type for a message object of type '<CVInfo>"
  "api/CVInfo")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CVInfo)))
  "Returns string type for a message object of type 'CVInfo"
  "api/CVInfo")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CVInfo>)))
  "Returns md5sum for a message object of type '<CVInfo>"
  "99332bc3002971b2c3503a06ab6aa2f6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CVInfo)))
  "Returns md5sum for a message object of type 'CVInfo"
  "99332bc3002971b2c3503a06ab6aa2f6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CVInfo>)))
  "Returns full string definition for message of type '<CVInfo>"
  (cl:format cl:nil "float32 x~%float32 y~%float32 w~%float32 h~%uint16 frame_width~%uint16 frame_height~%float32 score~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CVInfo)))
  "Returns full string definition for message of type 'CVInfo"
  (cl:format cl:nil "float32 x~%float32 y~%float32 w~%float32 h~%uint16 frame_width~%uint16 frame_height~%float32 score~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CVInfo>))
  (cl:+ 0
     4
     4
     4
     4
     2
     2
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CVInfo>))
  "Converts a ROS message object to a list"
  (cl:list 'CVInfo
    (cl:cons ':x (x msg))
    (cl:cons ':y (y msg))
    (cl:cons ':w (w msg))
    (cl:cons ':h (h msg))
    (cl:cons ':frame_width (frame_width msg))
    (cl:cons ':frame_height (frame_height msg))
    (cl:cons ':score (score msg))
))
