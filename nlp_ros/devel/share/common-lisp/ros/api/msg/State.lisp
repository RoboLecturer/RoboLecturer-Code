; Auto-generated. Do not edit!


(cl:in-package api-msg)


;//! \htmlinclude State.msg.html

(cl:defclass <State> (roslisp-msg-protocol:ros-message)
  ((Start
    :reader Start
    :initarg :Start
    :type cl:string
    :initform "")
   (AnyQuestions
    :reader AnyQuestions
    :initarg :AnyQuestions
    :type cl:string
    :initform "")
   (NoiseLevel
    :reader NoiseLevel
    :initarg :NoiseLevel
    :type cl:string
    :initform "")
   (Attentiveness
    :reader Attentiveness
    :initarg :Attentiveness
    :type cl:string
    :initform "")
   (NoQuestionsLoop
    :reader NoQuestionsLoop
    :initarg :NoQuestionsLoop
    :type cl:string
    :initform ""))
)

(cl:defclass State (<State>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <State>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'State)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name api-msg:<State> is deprecated: use api-msg:State instead.")))

(cl:ensure-generic-function 'Start-val :lambda-list '(m))
(cl:defmethod Start-val ((m <State>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader api-msg:Start-val is deprecated.  Use api-msg:Start instead.")
  (Start m))

(cl:ensure-generic-function 'AnyQuestions-val :lambda-list '(m))
(cl:defmethod AnyQuestions-val ((m <State>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader api-msg:AnyQuestions-val is deprecated.  Use api-msg:AnyQuestions instead.")
  (AnyQuestions m))

(cl:ensure-generic-function 'NoiseLevel-val :lambda-list '(m))
(cl:defmethod NoiseLevel-val ((m <State>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader api-msg:NoiseLevel-val is deprecated.  Use api-msg:NoiseLevel instead.")
  (NoiseLevel m))

(cl:ensure-generic-function 'Attentiveness-val :lambda-list '(m))
(cl:defmethod Attentiveness-val ((m <State>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader api-msg:Attentiveness-val is deprecated.  Use api-msg:Attentiveness instead.")
  (Attentiveness m))

(cl:ensure-generic-function 'NoQuestionsLoop-val :lambda-list '(m))
(cl:defmethod NoQuestionsLoop-val ((m <State>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader api-msg:NoQuestionsLoop-val is deprecated.  Use api-msg:NoQuestionsLoop instead.")
  (NoQuestionsLoop m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <State>) ostream)
  "Serializes a message object of type '<State>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'Start))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'Start))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'AnyQuestions))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'AnyQuestions))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'NoiseLevel))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'NoiseLevel))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'Attentiveness))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'Attentiveness))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'NoQuestionsLoop))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'NoQuestionsLoop))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <State>) istream)
  "Deserializes a message object of type '<State>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'Start) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'Start) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'AnyQuestions) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'AnyQuestions) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'NoiseLevel) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'NoiseLevel) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'Attentiveness) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'Attentiveness) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'NoQuestionsLoop) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'NoQuestionsLoop) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<State>)))
  "Returns string type for a message object of type '<State>"
  "api/State")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'State)))
  "Returns string type for a message object of type 'State"
  "api/State")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<State>)))
  "Returns md5sum for a message object of type '<State>"
  "814c22ab7e9ed8b959e5c73c87910fce")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'State)))
  "Returns md5sum for a message object of type 'State"
  "814c22ab7e9ed8b959e5c73c87910fce")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<State>)))
  "Returns full string definition for message of type '<State>"
  (cl:format cl:nil "string Start~%string AnyQuestions ~%string NoiseLevel~%string Attentiveness~%string NoQuestionsLoop~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'State)))
  "Returns full string definition for message of type 'State"
  (cl:format cl:nil "string Start~%string AnyQuestions ~%string NoiseLevel~%string Attentiveness~%string NoQuestionsLoop~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <State>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'Start))
     4 (cl:length (cl:slot-value msg 'AnyQuestions))
     4 (cl:length (cl:slot-value msg 'NoiseLevel))
     4 (cl:length (cl:slot-value msg 'Attentiveness))
     4 (cl:length (cl:slot-value msg 'NoQuestionsLoop))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <State>))
  "Converts a ROS message object to a list"
  (cl:list 'State
    (cl:cons ':Start (Start msg))
    (cl:cons ':AnyQuestions (AnyQuestions msg))
    (cl:cons ':NoiseLevel (NoiseLevel msg))
    (cl:cons ':Attentiveness (Attentiveness msg))
    (cl:cons ':NoQuestionsLoop (NoQuestionsLoop msg))
))
