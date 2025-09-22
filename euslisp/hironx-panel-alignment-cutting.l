(load "package://hrpsys_ros_bridge_tutorials/euslisp/hironxjsk-interface.l")
(load "package://dynamixel_scissors/euslisp/scissor-interface.l")

(hironxjsk-init)
(objects (list *hironxjsk*))

(send *hironxjsk* :reset-pose)
(send *ri* :angle-vector (send *hironxjsk* :angle-vector) 10000)
(send *ri* :wait-interpolation)

;; scissor init
(setq *si* (instance scissor-interface :init))

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
  
(defun panel-lifting ()
  (start-pose)
  (left-arm-ready)
  (left-hand-ready-pose)
  (approach)
  (insert)
  (left-hand-grasp)
  (left-hand-pull)
  (left-hand-pull)
  )
  
(defun left-arm-ready ()
  (send *hironxjsk* :larm :move-end-pos #f(0 0 100) :world)
  (send *ri* :angle-vector (send *hironxjsk* :angle-vector) 3000)
  (send *ri* :wait-interpolation)

  (send *hironxjsk* :larm :move-end-pos #f(-100 0 0) :world)
  (send *ri* :angle-vector (send *hironxjsk* :angle-vector) 1000)
  (send *ri* :wait-interpolation)

  (send *hironxjsk* :larm :move-end-pos #f(0 100 0) :world)
  (send *ri* :angle-vector (send *hironxjsk* :angle-vector) 1000)
  (send *ri* :wait-interpolation)
  
  (send *hironxjsk* :larm :move-end-rot -45 :y)
  (send *ri* :angle-vector (send *hironxjsk* :angle-vector) 1000)
  (send *ri* :wait-interpolation)

  (send *hironxjsk* :larm_joint5 :joint-angle 70)
  (send *ri* :angle-vector (send *hironxjsk* :angle-vector) 1000)
  (send *ri* :wait-interpolation)

  )

(defun left-hand-ready-pose ()
  (send *ri* :hand-angle-vector :lhand #f(90 -90 0 -25))
  )

(defun approach ()
  (send *hironxjsk* :larm :move-end-pos #f(100 0 0) :world)
  (send *ri* :angle-vector (send *hironxjsk* :angle-vector) 1000)
  (send *ri* :wait-interpolation)

  (send *hironxjsk* :larm :move-end-pos #f(0 0 100) :world)
  (send *ri* :angle-vector (send *hironxjsk* :angle-vector) 1000)
  (send *ri* :wait-interpolation)

  )

(setq *left-arm-ready-pose* #f(5.79059 -56.0983 -84.452 6.9821 -34.225 75.1981)) 

(defun insert ()
  (send *hironxjsk* :larm :move-end-pos #f(0 -100 0) :world)
  (send *ri* :angle-vector (send *hironxjsk* :angle-vector) 3000)
  (send *ri* :wait-interpolation)

  (send *hironxjsk* :larm :move-end-pos #f(0 -70 0) :world)
  (send *ri* :angle-vector (send *hironxjsk* :angle-vector) 3000)
  (send *ri* :wait-interpolation)

  )

(defun left-hand-grasp ()
  ;; (send *ri* :start-grasp :larm)
  ;; stop-grasp ready for start-grasp
  (send *ri* :hand-angle-vector :lhand #f(25 10 -25 -25))

  ;; start-grasp
  (send *ri* :hand-angle-vector :lhand #f(-25 10 15 -15))
  )

(defun left-hand-ungrasp ()
  (send *ri* :hand-angle-vector :lhand #f(90 -70 -25 -25))
  )


(defun left-hand-pull ()
  (send *hironxjsk* :larm :move-end-pos #f(-45 0 0) :world)
  (send *ri* :angle-vector (send *hironxjsk* :angle-vector) 1500)
  (send *ri* :wait-interpolation)
  
  )

;; alignment strands in air
;; need to start after panel lifting (main)
(defun alignment-in-air-panel ()
  (comb-in-air-ready-pose)
  (comb-in-air-approach)
  (comb-upward-pose)
  (comb-in-air-insert)
  (comb-in-air-pull-one-step)
  (left-hand-ungrasp)
  (left-hand-release)
  (left-hand-insert)
  (left-hand-grasp)
  (comb-in-air-pull-full)
  (comb-release)
  (left-hand-pull)
  (left-hand-pull)
  (left-hand-pull)
  (left-hand-pull)

  )

(defun comb-in-air-ready-pose ()
  ;; (send *hironxjsk* :rarm :angle-vector #f(15.7539 -44.3845 -105.638 27.2035 -27.8363 86.8512))
  (send *hironxjsk* :rarm :angle-vector #f(-9.31241 -77.6545 -81.6951 -1.30909 -21.3048 55.5165))
  (setq *rarm-comb-ready-pose* (send *hironxjsk* :angle-vector))

  (send *ri* :angle-vector (send *hironxjsk* :angle-vector) 10000)
  (send *ri* :wait-interpolation)
 
  )

(defun comb-in-air-approach ()
  (send *hironxjsk* :rarm :move-end-rot 45 :z)
  (send *ri* :angle-vector (send *hironxjsk* :angle-vector) 1500)
  (send *ri* :wait-interpolation)
  )

(defun comb-upward-pose ()
  ;; upward finger
  (send *ri* :hand-angle-vector :rhand #f(0 0 0 90))
  (send *ri* :wait-interpolation)
  ;; hold another finger
  (send *ri* :hand-angle-vector :rhand #f(-30 10 0 90))
  (send *ri* :wait-interpolation)
  )

(defun comb-in-air-insert ()
  ;; adjust position
  (send *hironxjsk* :rarm :move-end-pos #f(0 0 -100) :world)
  (send *ri* :angle-vector (send *hironxjsk* :angle-vector) 1500)
  (send *ri* :wait-interpolation)
  (send *hironxjsk* :rarm :move-end-pos #f(0 0 -100) :world)
  (send *ri* :angle-vector (send *hironxjsk* :angle-vector) 1500)
  (send *ri* :wait-interpolation)
  (send *hironxjsk* :rarm :move-end-pos #f(0 0 -50) :world)
  (send *ri* :angle-vector (send *hironxjsk* :angle-vector) 1500)
  (send *ri* :wait-interpolation)

  (send *hironxjsk* :rarm :move-end-pos #f(0 -50 0) :world)
  (send *ri* :angle-vector (send *hironxjsk* :angle-vector) 1500)
  (send *ri* :wait-interpolation)

  (send *hironxjsk* :rarm :move-end-pos #f(-30 0 0) :world)
  (send *ri* :angle-vector (send *hironxjsk* :angle-vector) 5000)
  (send *ri* :wait-interpolation)

  ;; go left
  (send *hironxjsk* :rarm :move-end-pos #f(0 100 0) :world)
  (send *ri* :angle-vector (send *hironxjsk* :angle-vector) 1500)
  (send *ri* :wait-interpolation)

  (send *hironxjsk* :rarm :move-end-pos #f(0 100 0) :world)
  (send *ri* :angle-vector (send *hironxjsk* :angle-vector) 1500)
  (send *ri* :wait-interpolation)

  ;; (send *hironxjsk* :rarm :move-end-pos #f(0 50 0) :world)
  ;; (send *ri* :angle-vector (send *hironxjsk* :angle-vector) 1500)
  ;; (send *ri* :wait-interpolation)

  ;; go forward for detecting a contact with the scalp
  (send *hironxjsk* :rarm :move-end-pos #f(30 0 0) :world)
  (send *ri* :angle-vector (send *hironxjsk* :angle-vector) 1500)
  (send *ri* :wait-interpolation)
  
  ;; go upward
  (send *hironxjsk* :rarm :move-end-pos #f(0 0 70) :world)
  (send *ri* :angle-vector (send *hironxjsk* :angle-vector) 1500)
  (send *ri* :wait-interpolation)

  (send *hironxjsk* :rarm :move-end-pos #f(0 0 50) :world)
  (send *ri* :angle-vector (send *hironxjsk* :angle-vector) 1500)
  (send *ri* :wait-interpolation)

  (send *hironxjsk* :rarm :move-end-pos #f(0 0 50) :world)
  (send *ri* :angle-vector (send *hironxjsk* :angle-vector) 1500)
  (send *ri* :wait-interpolation)
  
  )

(defun comb-in-air-pull-one-step ()
  (send *hironxjsk* :rarm :move-end-pos #f(-30 0 0) :world)
  (send *ri* :angle-vector (send *hironxjsk* :angle-vector) 1500)
  (send *ri* :wait-interpolation)

  )

(defun left-hand-release ()
  ;; left larm for collision with hand and hair
  (send *hironxjsk* :larm :move-end-pos #f(0 0 50) :world)
  (send *ri* :angle-vector (send *hironxjsk* :angle-vector) 1500)
  (send *ri* :wait-interpolation)

  (send *hironxjsk* :larm :move-end-pos #f(0 100 0) :world)
  (send *ri* :angle-vector (send *hironxjsk* :angle-vector) 1500)
  (send *ri* :wait-interpolation)

  (send *hironxjsk* :larm :move-end-pos #f(0 100 0) :world)
  (send *ri* :angle-vector (send *hironxjsk* :angle-vector) 1500)
  (send *ri* :wait-interpolation)


  )

(defun left-hand-insert ()
  ;; go forward for re-grip
  (send *hironxjsk* :larm :move-end-pos #f(100 0 0) :world)
  (send *ri* :angle-vector (send *hironxjsk* :angle-vector) 1500)
  (send *ri* :wait-interpolation)

  ;; go upward for finding grip position
  (send *hironxjsk* :larm :move-end-pos #f(0 0 30) :world)
  (send *ri* :angle-vector (send *hironxjsk* :angle-vector) 1500)
  (send *ri* :wait-interpolation)

  ;; go right
  (send *hironxjsk* :larm :move-end-pos #f(0 -100 0) :world)
  (send *ri* :angle-vector (send *hironxjsk* :angle-vector) 1500)
  (send *ri* :wait-interpolation)

  (send *hironxjsk* :larm :move-end-pos #f(0 -50 0) :world)
  (send *ri* :angle-vector (send *hironxjsk* :angle-vector) 1500)
  (send *ri* :wait-interpolation)

  (send *hironxjsk* :larm :move-end-pos #f(0 -30 0) :world)
  (send *ri* :angle-vector (send *hironxjsk* :angle-vector) 1500)
  (send *ri* :wait-interpolation)

  (send *hironxjsk* :larm :move-end-pos #f(-30 0 0) :world)
  (send *ri* :angle-vector (send *hironxjsk* :angle-vector) 1500)
  (send *ri* :wait-interpolation)
  
  )

(defun comb-in-air-pull-full ()
  ;; pull
  (send *hironxjsk* :rarm :move-end-pos #f(-50 0 0) :world)
  (send *ri* :angle-vector (send *hironxjsk* :angle-vector) 1500)
  (send *ri* :wait-interpolation)

  (send *hironxjsk* :rarm :move-end-pos #f(-50 0 0) :world)
  (send *ri* :angle-vector (send *hironxjsk* :angle-vector) 1500)
  (send *ri* :wait-interpolation)

  (send *hironxjsk* :rarm :move-end-pos #f(-50 0 -50) :world)
  (send *ri* :angle-vector (send *hironxjsk* :angle-vector) 1500)
  (send *ri* :wait-interpolation)

  (send *hironxjsk* :rarm :move-end-pos #f(0 0 -50) :world)
  (send *ri* :angle-vector (send *hironxjsk* :angle-vector) 1500)
  (send *ri* :wait-interpolation)

  (send *hironxjsk* :rarm :move-end-pos #f(0 0 -20) :world)
  (send *ri* :angle-vector (send *hironxjsk* :angle-vector) 1500)
  (send *ri* :wait-interpolation)
 
  )

(defun comb-release ()
  (send *hironxjsk* :rarm :move-end-pos #f(0 -200 0) :world)
  (send *ri* :angle-vector (send *hironxjsk* :angle-vector) 1500)
  (send *ri* :wait-interpolation)
 
  )

(defun cutting ()
  (scissors-ready-pose)
  (scissors-approach)
  ;; repeat following actions
  (scissors-open)
  (scissors-insert)
  (scissors-close)

  ;; (scissors-open)
  ;; (scissors-insert)
  ;; (scissors-close)

  ;; (scissors-open)
  ;; (scissors-insert)
  ;; (scissors-close)
  ;; =======================
  
  (scissors-release)

  ;; (scissors-ready-pose)

  ;; ======================
  (left-hand-ungrasp)
  (left-arm-ready-pose)
  )

(defun scissors-ready-pose ()
  (send *hironxjsk* :rarm :angle-vector #f(-9.31241 -77.6545 -81.6951 -1.30909 -21.3048 55.5165))
  (send *hironxjsk* :rarm :move-end-rot -180 :z)
  (send *ri* :angle-vector (send *hironxjsk* :angle-vector) 1500)
  (send *ri* :wait-interpolation)
  
  )

(defun scissors-approach ()
  (send *hironxjsk* :rarm :move-end-rot 45 :z)
  (send *ri* :angle-vector (send *hironxjsk* :angle-vector) 1500)
  (send *ri* :wait-interpolation)

  (send *hironxjsk* :rarm :move-end-pos #f(0 0 -100) :world)
  (send *ri* :angle-vector (send *hironxjsk* :angle-vector) 1500)
  (send *ri* :wait-interpolation)

  (send *hironxjsk* :rarm :move-end-pos #f(-100 0 0) :world)
  (send *ri* :angle-vector (send *hironxjsk* :angle-vector) 1500)
  (send *ri* :wait-interpolation)

  (send *hironxjsk* :rarm :move-end-pos #f(-20 0 0) :world)
  (send *ri* :angle-vector (send *hironxjsk* :angle-vector) 1500)
  (send *ri* :wait-interpolation)

  (send *hironxjsk* :rarm :move-end-pos #f(-10 0 0) :world)
  (send *ri* :angle-vector (send *hironxjsk* :angle-vector) 1500)
  (send *ri* :wait-interpolation)

  ;; adjust scissors angle
  (send *hironxjsk* :rarm :move-end-rot 10 :y)
  (send *ri* :angle-vector (send *hironxjsk* :angle-vector) 1500)
  (send *ri* :wait-interpolation)

  (send *hironxjsk* :rarm :move-end-rot 5 :z)
  (send *ri* :angle-vector (send *hironxjsk* :angle-vector) 1500)
  (send *ri* :wait-interpolation)

  ;; (send *hironxjsk* :rarm :move-end-pos #f(-10 0 0) :world)
  ;;  (send *ri* :angle-vector (send *hironxjsk* :angle-vector) 1500)
  ;; (send *ri* :wait-interpolation)

  ;; adjust scissors position
  (send *hironxjsk* :rarm :move-end-pos #f(-10 0 0) :world)
  (send *ri* :angle-vector (send *hironxjsk* :angle-vector) 1500)
  (send *ri* :wait-interpolation)

  (send *hironxjsk* :rarm :move-end-pos #f(0 0 30) :world)
  (send *ri* :angle-vector (send *hironxjsk* :angle-vector) 1500)
  (send *ri* :wait-interpolation)

  (send *hironxjsk* :rarm :move-end-pos #f(10 0 0) :world)
  (send *ri* :angle-vector (send *hironxjsk* :angle-vector) 1500)
  (send *ri* :wait-interpolation)

  )

(defun scissors-open ()
  (send *si* :open)
  (unix:sleep 1)
  )

(defun scissors-close ()
  (send *si* :close)
  (unix:sleep 1)
  )

(defun scissors-insert ()
  (send *hironxjsk* :rarm :move-end-pos #f(0 50 0) :world)
  (send *ri* :angle-vector (send *hironxjsk* :angle-vector) 1500)
  (send *ri* :wait-interpolation)

  (send *hironxjsk* :rarm :move-end-pos #f(0 50 0) :world)
  (send *ri* :angle-vector (send *hironxjsk* :angle-vector) 1500)
  (send *ri* :wait-interpolation)

  (send *hironxjsk* :rarm :move-end-pos #f(0 50 0) :world)
  (send *ri* :angle-vector (send *hironxjsk* :angle-vector) 1500)
  (send *ri* :wait-interpolation)

  (send *hironxjsk* :rarm :move-end-pos #f(0 50 0) :world)
  (send *ri* :angle-vector (send *hironxjsk* :angle-vector) 1500)
  (send *ri* :wait-interpolation)

  (send *hironxjsk* :rarm :move-end-pos #f(0 20 0) :world)
  (send *ri* :angle-vector (send *hironxjsk* :angle-vector) 1500)
  (send *ri* :wait-interpolation)

  (send *hironxjsk* :rarm :move-end-pos #f(0 0 10) :world)
  (send *ri* :angle-vector (send *hironxjsk* :angle-vector) 1500)
  (send *ri* :wait-interpolation)

  )

(defun scissors-release ()
  (send *hironxjsk* :rarm :move-end-pos #f(0 -100 0) :world)
  (send *ri* :angle-vector (send *hironxjsk* :angle-vector) 1500)
  (send *ri* :wait-interpolation)

  (send *hironxjsk* :rarm :move-end-pos #f(0 -100 0) :world)
  (send *ri* :angle-vector (send *hironxjsk* :angle-vector) 1500)
  (send *ri* :wait-interpolation)

  )

(defun left-arm-ready-pose ()
  (send *hironxjsk* :larm :angle-vector *left-arm-ready-pose*)
  (send *ri* :angle-vector (send *hironxjsk* :angle-vector) 1500)
  (send *ri* :wait-interpolation)

  )

(defun main ()
  (panel-lifting)
  (alignment-in-air-panel)
  (cutting)
  )

;; (dotimes (i 3)
;;   (combing))

