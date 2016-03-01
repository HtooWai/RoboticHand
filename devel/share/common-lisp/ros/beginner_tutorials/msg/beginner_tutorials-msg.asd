
(cl:in-package :asdf)

(defsystem "beginner_tutorials-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Num" :depends-on ("_package_Num"))
    (:file "_package_Num" :depends-on ("_package"))
    (:file "Finger" :depends-on ("_package_Finger"))
    (:file "_package_Finger" :depends-on ("_package"))
    (:file "Pressure" :depends-on ("_package_Pressure"))
    (:file "_package_Pressure" :depends-on ("_package"))
    (:file "Sensor" :depends-on ("_package_Sensor"))
    (:file "_package_Sensor" :depends-on ("_package"))
  ))