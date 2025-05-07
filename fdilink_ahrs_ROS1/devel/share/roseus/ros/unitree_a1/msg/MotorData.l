;; Auto-generated. Do not edit!


(when (boundp 'unitree_a1::MotorData)
  (if (not (find-package "UNITREE_A1"))
    (make-package "UNITREE_A1"))
  (shadow 'MotorData (find-package "UNITREE_A1")))
(unless (find-package "UNITREE_A1::MOTORDATA")
  (make-package "UNITREE_A1::MOTORDATA"))

(in-package "ROS")
;;//! \htmlinclude MotorData.msg.html


(defclass unitree_a1::MotorData
  :super ros::object
  :slots (_legid _motorid _mode _tau _vel _acc _pos _temp _error ))

(defmethod unitree_a1::MotorData
  (:init
   (&key
    ((:legid __legid) (make-array 0 :initial-element 0 :element-type :char))
    ((:motorid __motorid) (make-array 0 :initial-element 0 :element-type :char))
    ((:mode __mode) (make-array 0 :initial-element 0 :element-type :char))
    ((:tau __tau) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:vel __vel) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:acc __acc) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:pos __pos) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:temp __temp) (make-array 0 :initial-element 0 :element-type :integer))
    ((:error __error) (make-array 0 :initial-element 0 :element-type :integer))
    )
   (send-super :init)
   (setq _legid __legid)
   (setq _motorid __motorid)
   (setq _mode __mode)
   (setq _tau __tau)
   (setq _vel __vel)
   (setq _acc __acc)
   (setq _pos __pos)
   (setq _temp __temp)
   (setq _error __error)
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
  (:acc
   (&optional __acc)
   (if __acc (setq _acc __acc)) _acc)
  (:pos
   (&optional __pos)
   (if __pos (setq _pos __pos)) _pos)
  (:temp
   (&optional __temp)
   (if __temp (setq _temp __temp)) _temp)
  (:error
   (&optional __error)
   (if __error (setq _error __error)) _error)
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
    ;; float32[] _acc
    (* 4    (length _acc)) 4
    ;; float32[] _pos
    (* 4    (length _pos)) 4
    ;; int8[] _temp
    (* 1    (length _temp)) 4
    ;; int8[] _error
    (* 1    (length _error)) 4
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
     ;; float32[] _acc
     (write-long (length _acc) s)
     (dotimes (i (length _acc))
       (sys::poke (elt _acc i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[] _pos
     (write-long (length _pos) s)
     (dotimes (i (length _pos))
       (sys::poke (elt _pos i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; int8[] _temp
     (write-long (length _temp) s)
     (dotimes (i (length _temp))
       (write-byte (elt _temp i) s)
       )
     ;; int8[] _error
     (write-long (length _error) s)
     (dotimes (i (length _error))
       (write-byte (elt _error i) s)
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
   ;; float32[] _acc
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _acc (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _acc i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32[] _pos
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _pos (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _pos i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; int8[] _temp
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _temp (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _temp i) (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> (elt _temp i) 127) (setf (elt _temp i) (- (elt _temp i) 256)))
     ))
   ;; int8[] _error
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _error (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _error i) (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> (elt _error i) 127) (setf (elt _error i) (- (elt _error i) 256)))
     ))
   ;;
   self)
  )

(setf (get unitree_a1::MotorData :md5sum-) "3b3e3cad6f8f2ba7b0cdd1200569952b")
(setf (get unitree_a1::MotorData :datatype-) "unitree_a1/MotorData")
(setf (get unitree_a1::MotorData :definition-)
      "uint8[] legid
uint8[] motorid
uint8[] mode
float32[] tau
float32[] vel
float32[] acc
float32[] pos
int8[] temp
int8[] error
")



(provide :unitree_a1/MotorData "3b3e3cad6f8f2ba7b0cdd1200569952b")


