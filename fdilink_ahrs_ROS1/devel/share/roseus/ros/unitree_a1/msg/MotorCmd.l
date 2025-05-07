;; Auto-generated. Do not edit!


(when (boundp 'unitree_a1::MotorCmd)
  (if (not (find-package "UNITREE_A1"))
    (make-package "UNITREE_A1"))
  (shadow 'MotorCmd (find-package "UNITREE_A1")))
(unless (find-package "UNITREE_A1::MOTORCMD")
  (make-package "UNITREE_A1::MOTORCMD"))

(in-package "ROS")
;;//! \htmlinclude MotorCmd.msg.html


(defclass unitree_a1::MotorCmd
  :super ros::object
  :slots (_legid _motorid _mode _tau _vel _pos _kp _kd ))

(defmethod unitree_a1::MotorCmd
  (:init
   (&key
    ((:legid __legid) (make-array 0 :initial-element 0 :element-type :char))
    ((:motorid __motorid) (make-array 0 :initial-element 0 :element-type :char))
    ((:mode __mode) (make-array 0 :initial-element 0 :element-type :char))
    ((:tau __tau) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:vel __vel) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:pos __pos) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:kp __kp) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:kd __kd) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _legid __legid)
   (setq _motorid __motorid)
   (setq _mode __mode)
   (setq _tau __tau)
   (setq _vel __vel)
   (setq _pos __pos)
   (setq _kp __kp)
   (setq _kd __kd)
   self)
  (:legid
   (&optional __legid)
   (if __legid (setq _legid __legid)) _legid)
  (:motorid
   (&optional __motorid)
   (if __motorid (setq _motorid __motorid)) _motorid)
  (:mode
   (&optional __mode)
   (if __mode (setq _mode __mode)) _mode)
  (:tau
   (&optional __tau)
   (if __tau (setq _tau __tau)) _tau)
  (:vel
   (&optional __vel)
   (if __vel (setq _vel __vel)) _vel)
  (:pos
   (&optional __pos)
   (if __pos (setq _pos __pos)) _pos)
  (:kp
   (&optional __kp)
   (if __kp (setq _kp __kp)) _kp)
  (:kd
   (&optional __kd)
   (if __kd (setq _kd __kd)) _kd)
  (:serialization-length
   ()
   (+
    ;; uint8[] _legid
    (* 1    (length _legid)) 4
    ;; uint8[] _motorid
    (* 1    (length _motorid)) 4
    ;; uint8[] _mode
    (* 1    (length _mode)) 4
    ;; float32[] _tau
    (* 4    (length _tau)) 4
    ;; float32[] _vel
    (* 4    (length _vel)) 4
    ;; float32[] _pos
    (* 4    (length _pos)) 4
    ;; float32[] _kp
    (* 4    (length _kp)) 4
    ;; float32[] _kd
    (* 4    (length _kd)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8[] _legid
     (write-long (length _legid) s)
     (princ _legid s)
     ;; uint8[] _motorid
     (write-long (length _motorid) s)
     (princ _motorid s)
     ;; uint8[] _mode
     (write-long (length _mode) s)
     (princ _mode s)
     ;; float32[] _tau
     (write-long (length _tau) s)
     (dotimes (i (length _tau))
       (sys::poke (elt _tau i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[] _vel
     (write-long (length _vel) s)
     (dotimes (i (length _vel))
       (sys::poke (elt _vel i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[] _pos
     (write-long (length _pos) s)
     (dotimes (i (length _pos))
       (sys::poke (elt _pos i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[] _kp
     (write-long (length _kp) s)
     (dotimes (i (length _kp))
       (sys::poke (elt _kp i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[] _kd
     (write-long (length _kd) s)
     (dotimes (i (length _kd))
       (sys::poke (elt _kd i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8[] _legid
   (let ((n (sys::peek buf ptr- :integer))) (incf ptr- 4)
     (setq _legid (make-array n :element-type :char))
     (replace _legid buf :start2 ptr-) (incf ptr- n))
   ;; uint8[] _motorid
   (let ((n (sys::peek buf ptr- :integer))) (incf ptr- 4)
     (setq _motorid (make-array n :element-type :char))
     (replace _motorid buf :start2 ptr-) (incf ptr- n))
   ;; uint8[] _mode
   (let ((n (sys::peek buf ptr- :integer))) (incf ptr- 4)
     (setq _mode (make-array n :element-type :char))
     (replace _mode buf :start2 ptr-) (incf ptr- n))
   ;; float32[] _tau
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _tau (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _tau i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32[] _vel
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _vel (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _vel i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32[] _pos
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _pos (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _pos i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32[] _kp
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _kp (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _kp i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32[] _kd
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _kd (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _kd i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;;
   self)
  )

(setf (get unitree_a1::MotorCmd :md5sum-) "8cda35a8825431ce1f5b8477f459672b")
(setf (get unitree_a1::MotorCmd :datatype-) "unitree_a1/MotorCmd")
(setf (get unitree_a1::MotorCmd :definition-)
      "uint8[] legid
uint8[] motorid
uint8[] mode
float32[] tau
float32[] vel
float32[] pos
float32[] kp
float32[] kd
")



(provide :unitree_a1/MotorCmd "8cda35a8825431ce1f5b8477f459672b")


