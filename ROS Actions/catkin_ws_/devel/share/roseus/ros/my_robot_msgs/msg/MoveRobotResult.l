;; Auto-generated. Do not edit!


(when (boundp 'my_robot_msgs::MoveRobotResult)
  (if (not (find-package "MY_ROBOT_MSGS"))
    (make-package "MY_ROBOT_MSGS"))
  (shadow 'MoveRobotResult (find-package "MY_ROBOT_MSGS")))
(unless (find-package "MY_ROBOT_MSGS::MOVEROBOTRESULT")
  (make-package "MY_ROBOT_MSGS::MOVEROBOTRESULT"))

(in-package "ROS")
;;//! \htmlinclude MoveRobotResult.msg.html


(defclass my_robot_msgs::MoveRobotResult
  :super ros::object
  :slots (_position _message ))

(defmethod my_robot_msgs::MoveRobotResult
  (:init
   (&key
    ((:position __position) 0)
    ((:message __message) "")
    )
   (send-super :init)
   (setq _position (round __position))
   (setq _message (string __message))
   self)
  (:position
   (&optional __position)
   (if __position (setq _position __position)) _position)
  (:message
   (&optional __message)
   (if __message (setq _message __message)) _message)
  (:serialization-length
   ()
   (+
    ;; int64 _position
    8
    ;; string _message
    4 (length _message)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int64 _position
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _position (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _position) (= (length (_position . bv)) 2)) ;; bignum
              (write-long (ash (elt (_position . bv) 0) 0) s)
              (write-long (ash (elt (_position . bv) 1) -1) s))
             ((and (class _position) (= (length (_position . bv)) 1)) ;; big1
              (write-long (elt (_position . bv) 0) s)
              (write-long (if (>= _position 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _position s)(write-long (if (>= _position 0) 0 #xffffffff) s)))
     ;; string _message
       (write-long (length _message) s) (princ _message s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int64 _position
#+(or :alpha :irix6 :x86_64)
      (setf _position (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _position (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; string _message
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _message (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get my_robot_msgs::MoveRobotResult :md5sum-) "06665361780db4be123c7e09b6d2fd85")
(setf (get my_robot_msgs::MoveRobotResult :datatype-) "my_robot_msgs/MoveRobotResult")
(setf (get my_robot_msgs::MoveRobotResult :definition-)
      "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======
#result
int64 position
string message

")



(provide :my_robot_msgs/MoveRobotResult "06665361780db4be123c7e09b6d2fd85")

