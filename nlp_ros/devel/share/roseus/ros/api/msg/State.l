;; Auto-generated. Do not edit!


(when (boundp 'api::State)
  (if (not (find-package "API"))
    (make-package "API"))
  (shadow 'State (find-package "API")))
(unless (find-package "API::STATE")
  (make-package "API::STATE"))

(in-package "ROS")
;;//! \htmlinclude State.msg.html


(defclass api::State
  :super ros::object
  :slots (_Start _AnyQuestions _NoiseLevel _Attentiveness _NoQuestionsLoop ))

(defmethod api::State
  (:init
   (&key
    ((:Start __Start) "")
    ((:AnyQuestions __AnyQuestions) "")
    ((:NoiseLevel __NoiseLevel) "")
    ((:Attentiveness __Attentiveness) "")
    ((:NoQuestionsLoop __NoQuestionsLoop) "")
    )
   (send-super :init)
   (setq _Start (string __Start))
   (setq _AnyQuestions (string __AnyQuestions))
   (setq _NoiseLevel (string __NoiseLevel))
   (setq _Attentiveness (string __Attentiveness))
   (setq _NoQuestionsLoop (string __NoQuestionsLoop))
   self)
  (:Start
   (&optional __Start)
   (if __Start (setq _Start __Start)) _Start)
  (:AnyQuestions
   (&optional __AnyQuestions)
   (if __AnyQuestions (setq _AnyQuestions __AnyQuestions)) _AnyQuestions)
  (:NoiseLevel
   (&optional __NoiseLevel)
   (if __NoiseLevel (setq _NoiseLevel __NoiseLevel)) _NoiseLevel)
  (:Attentiveness
   (&optional __Attentiveness)
   (if __Attentiveness (setq _Attentiveness __Attentiveness)) _Attentiveness)
  (:NoQuestionsLoop
   (&optional __NoQuestionsLoop)
   (if __NoQuestionsLoop (setq _NoQuestionsLoop __NoQuestionsLoop)) _NoQuestionsLoop)
  (:serialization-length
   ()
   (+
    ;; string _Start
    4 (length _Start)
    ;; string _AnyQuestions
    4 (length _AnyQuestions)
    ;; string _NoiseLevel
    4 (length _NoiseLevel)
    ;; string _Attentiveness
    4 (length _Attentiveness)
    ;; string _NoQuestionsLoop
    4 (length _NoQuestionsLoop)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _Start
       (write-long (length _Start) s) (princ _Start s)
     ;; string _AnyQuestions
       (write-long (length _AnyQuestions) s) (princ _AnyQuestions s)
     ;; string _NoiseLevel
       (write-long (length _NoiseLevel) s) (princ _NoiseLevel s)
     ;; string _Attentiveness
       (write-long (length _Attentiveness) s) (princ _Attentiveness s)
     ;; string _NoQuestionsLoop
       (write-long (length _NoQuestionsLoop) s) (princ _NoQuestionsLoop s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _Start
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _Start (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _AnyQuestions
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _AnyQuestions (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _NoiseLevel
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _NoiseLevel (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _Attentiveness
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _Attentiveness (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _NoQuestionsLoop
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _NoQuestionsLoop (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get api::State :md5sum-) "814c22ab7e9ed8b959e5c73c87910fce")
(setf (get api::State :datatype-) "api/State")
(setf (get api::State :definition-)
      "string Start
string AnyQuestions 
string NoiseLevel
string Attentiveness
string NoQuestionsLoop

")



(provide :api/State "814c22ab7e9ed8b959e5c73c87910fce")


