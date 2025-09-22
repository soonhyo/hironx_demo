(load "package://hrpsys_ros_bridge_tutorials/euslisp/hironxjsk-interface.l")

(hironxjsk-init)
(objects (list *hironxjsk*))

(send *hironxjsk* :reset-pose)
(send *ri* :angle-vector (send *hironxjsk* :angle-vector) 10000)
(send *ri* :wait-interpolation)

;; go ready pose
(send *hironxjsk* :rarm :angle-vector #f(15.7539 -44.3845 -105.638 27.2035 -27.8363 86.8512))
(setq *ready-pose* (send *hironxjsk* :angle-vector))

(send *ri* :angle-vector (send *hironxjsk* :angle-vector) 10000)
(send *ri* :wait-interpolation)

(defun combing ()
  ;; go left a little bit
  (send *hironxjsk* :rarm :move-end-pos #f(0 30 0) :world)
  (send *ri* :angle-vector (send *hironxjsk* :angle-vector) 1000)
  (send *ri* :wait-interpolation)

  ;; go front for approach to hair ;; with force sensor
  (send *hironxjsk* :rarm :move-end-pos #f(-20 0 0) :world)
  (send *ri* :angle-vector (send *hironxjsk* :angle-vector) 1000)
  (send *ri* :wait-interpolation)

  ;; go down (combing)
  (send *hironxjsk* :rarm :move-end-pos #f(0 0 -50) :world)
  (send *ri* :angle-vector (send *hironxjsk* :angle-vector) 3000)
  (send *ri* :wait-interpolation)

  (send *hironxjsk* :rarm :move-end-pos #f(0 0 -100) :world)
  (send *ri* :angle-vector (send *hironxjsk* :angle-vector) 3000)
  (send *ri* :wait-interpolation)
  (send *hironxjsk* :rarm :move-end-pos #f(0 0 -100) :world)
  (send *ri* :angle-vector (send *hironxjsk* :angle-vector) 3000)
  (send *ri* :wait-interpolation)
  (send *hironxjsk* :rarm :move-end-pos #f(0 0 -100) :world)
  (send *ri* :angle-vector (send *hironxjsk* :angle-vector) 3000)
  (send *ri* :wait-interpolation)

  ;; go back for release
  (send *hironxjsk* :rarm :move-end-pos #f(-100 0 0) :world)
  (send *ri* :angle-vector (send *hironxjsk* :angle-vector) 10000)
  (send *ri* :wait-interpolation)

  ;; return to ready pose
  (send *hironxjsk* :rarm :move-end-pos #f(0 0 100) :world)
  (send *ri* :angle-vector (send *hironxjsk* :angle-vector) 3000)
  (send *ri* :wait-interpolation)
  
  (send *hironxjsk* :rarm :move-end-pos #f(0 0 100) :world)
  (send *ri* :angle-vector (send *hironxjsk* :angle-vector) 3000)
  (send *ri* :wait-interpolation)

  (send *hironxjsk* :rarm :move-end-pos #f(0 0 100) :world)
  (send *ri* :angle-vector (send *hironxjsk* :angle-vector) 3000)
  (send *ri* :wait-interpolation)

  (send *hironxjsk* :rarm :angle-vector #f(15.7539 -44.3845 -105.638 27.2035 -27.8363 86.8512))
  (send *ri* :angle-vector (send *hironxjsk* :angle-vector) 3000)
  (send *ri* :wait-interpolation)
  )

(defun front-back-right ()
  ;; go left 
  (send *hironxjsk* :rarm :move-end-pos #f(0 60 0) :world)
  (send *ri* :angle-vector (send *hironxjsk* :angle-vector) 5000)
  (send *ri* :wait-interpolation)
  ;; go front for approach to hair
  (send *hironxjsk* :rarm :move-end-pos #f(170 0 0) :world)
  (send *ri* :angle-vector (send *hironxjsk* :angle-vector) 10000)
  (send *ri* :wait-interpolation)

  ;; go right (combing)
  (send *hironxjsk* :rarm :move-end-pos #f(0 -80 0) :world)
  (send *ri* :angle-vector (send *hironxjsk* :angle-vector) 5000)
  (send *ri* :wait-interpolation)

  ;; go down (combing)
  (send *hironxjsk* :rarm :move-end-pos #f(0 0 -300) :world)
  (send *ri* :angle-vector (send *hironxjsk* :angle-vector) 10000)
  (send *ri* :wait-interpolation)

  ;; go back for release
  (send *hironxjsk* :rarm :move-end-pos #f(-100 0 0) :world)
  (send *ri* :angle-vector (send *hironxjsk* :angle-vector) 10000)
  (send *ri* :wait-interpolation)

  ;; return to ready pose
  (send *hironxjsk* :rarm :angle-vector #f(-35.034 -30.6594 -126.532 -37.2002 -18.5552 -91.4232))
  (send *ri* :angle-vector (send *hironxjsk* :angle-vector) 10000)
  (send *ri* :wait-interpolation)
  )

;; (dotimes (i 3)
;;   (combing))

