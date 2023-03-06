
(cl:in-package :asdf)

(defsystem "api-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "CVInfo" :depends-on ("_package_CVInfo"))
    (:file "_package_CVInfo" :depends-on ("_package"))
  ))