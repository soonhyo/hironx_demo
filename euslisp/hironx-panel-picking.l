(load "package://hrpsys_ros_bridge_tutorials/euslisp/hironxjsk-interface.l")

(hironxjsk-init)
(objects (list *hironxjsk*))

(send *hironxjsk* :reset-pose)
(send *ri* :angle-vector (send *hironxjsk* :angle-vector) 10000)
(send *ri* :wait-interpolation)

;; go ready pose
;; (send *hironxjsk* :larm :angle-vector #f(-35.034 -30.6594 -126.532 -37.2002 -18.5552 -91.4232))
;; (setq *ready-pose* (send *hironxjsk* :angle-vector))

;; (send *ri* :angle-vector (send *hironxjsk* :angle-vector) 10000)
;; (send *ri* :wait-interpolation)
(defun start-pose ()
  (send *hironxjsk* :reset-pose)
  (send *ri* :angle-vector (send *hironxjsk* :angle-vector) 10000)
  (send *ri* :wait-interpolation)

  ;; stop-grasp
  (send *ri* :hand-angle-vector :lhand #f(-25 10 25 -25))

  )
  
(defun main ()
  (start-pose)
  (left-arm-ready)
  (left-hand-ready-pose)
  (approach)
  (insert)
  (left-hand-grasp)
  (pull)
  )
  
(defun left-arm-ready ()
  (send *hironxjsk* :larm :move-end-pos #f(0 0 100) :world)
  (send *ri* :angle-vector (send *hironxjsk* :angle-vector) 10000)
  (send *ri* :wait-interpolation)

  (send *hironxjsk* :larm :move-end-pos #f(-100 0 0) :world)
  (send *ri* :angle-vector (send *hironxjsk* :angle-vector) 10000)
  (send *ri* :wait-interpolation)

  (send *hironxjsk* :larm :move-end-pos #f(0 100 0) :world)
  (send *ri* :angle-vector (send *hironxjsk* :angle-vector) 10000)
  (send *ri* :wait-interpolation)
  
  (send *hironxjsk* :larm :move-end-rot -45 :y)
  (send *ri* :angle-vector (send *hironxjsk* :angle-vector) 10000)
  (send *ri* :wait-interpolation)

  (send *hironxjsk* :larm_joint5 :joint-angle 70)
  (send *ri* :angle-vector (send *hironxjsk* :angle-vector) 10000)
  (send *ri* :wait-interpolation)

  )

(defun left-hand-ready-pose ()
  (send *ri* :hand-angle-vector :lhand #f(90 -90 0 -25))
  )

(defun approach ()
  (send *hironxjsk* :larm :move-end-pos #f(100 0 0) :world)
  (send *ri* :angle-vector (send *hironxjsk* :angle-vector) 10000)
  (send *ri* :wait-interpolation)

  (send *hironxjsk* :larm :move-end-pos #f(0 0 100) :world)
  (send *ri* :angle-vector (send *hironxjsk* :angle-vector) 10000)
  (send *ri* :wait-interpolation)

  )

(defun insert ()
  (send *hironxjsk* :larm :move-end-pos #f(0 -100 0) :world)
  (send *ri* :angle-vector (send *hironxjsk* :angle-vector) 10000)
  (send *ri* :wait-interpolation)

  (send *hironxjsk* :larm :move-end-pos #f(0 -100 0) :world)
  (send *ri* :angle-vector (send *hironxjsk* :angle-vector) 10000)
  (send *ri* :wait-interpolation)

  )

(defun left-hand-grasp ()
  ;; (send *ri* :start-grasp :larm)
  ;; stop-grasp ready for start-grasp
  (send *ri* :hand-angle-vector :lhand #f(25 10 -25 -25))

  ;; start-grasp
  (send *ri* :hand-angle-vector :lhand #f(-25 10 25 -25))
  )


(defun pull ()
  (send *hironxjsk* :larm :move-end-pos #f(-100 0 0) :world)
  (send *ri* :angle-vector (send *hironxjsk* :angle-vector) 10000)
  (send *ri* :wait-interpolation)
  
  )

;; (dotimes (i 3)
;;   (combing))

