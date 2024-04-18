#!/usr/bin/env roseus

(require "package://jsk_2015_05_baxter_apc/euslisp/lib/util.l")
(require "package://jsk_2016_01_baxter_apc/euslisp/lib/util.l")
(require "package://jsk_arc2017_baxter/euslisp/lib/arc-interface.l")

(ros::load-ros-manifest "jsk_arc2017_baxter")

(unless (find-package "JSK_ARC2017_BAXTER")
  (make-package "JSK_ARC2017_BAXTER"))

(defclass jsk_arc2017_baxter::stow-interface
  :super jsk_arc2017_baxter::arc-interface
  :slots (grasp-style-
          moveit-p-
          picking-fail-count-
          prev-target-obj-
          recognize-fail-count-
          scale-candidates-
          start-picking-fail-count-
          start-time-
          target-bin-
          target-obj-
          trial-fail-count-))

(defmethod jsk_arc2017_baxter::stow-interface
  (:init (&key (moveit nil))
    (send-super :init)
    (setq picking-fail-count- 0)
    (setq recognize-fail-count- 0)
    (setq start-picking-fail-count- 0)
    (setq trial-fail-count- 0)
    (setq moveit-p- moveit)
    (setq grasp-style- nil))
  (:recognize-bboxes ()
    (ros::ros-info "[main] recognizing bin boxes")
    (send self :recognize-bin-boxes :stamp (ros::time-now))
    (ros::ros-info "[main] recognizing tote")
    (send self :recognize-tote-boxes :stamp (ros::time-now))
    ;; XXX: movable region will be set in :pick-object
    ;; (send self :set-movable-region-for-tote arm :offset (list 100 80 0))
    )
  (:wait-for-user-input (arm)
    (let (can-start)
      (when moveit-p-
        (send self :add-shelf-scene)
        (send self :add-tote-scene arm))
      (ros::ros-info "[:wait-for-user-input] wait for user input to start: ~a" arm)
      (ros::wait-for-service "/rviz/yes_no_button")
      (setq can-start
            (send (ros::service-call "/rviz/yes_no_button"
                                     (instance jsk_gui_msgs::YesNoRequest))
                  :yes))
      (ros::ros-info "[:wait-for-user-input] received user input: ~a" arm)
      (setq start-time- (ros::time-now))
      (send self :reset-object-tables :tote)
      (send self :set-objects-param arm :tote :object-type :all)
      can-start))
  (:calib-prismatic-joint (arm &key (timeout 10))
    (send (send self :get-robot-interface) :start-prismatic-calib arm)
    (let ((time-count 1))
      (while (and (send (send self :get-robot-interface) :prismatic-calibratingp arm)
                  (> (* timeout 10) time-count))
        (unix::usleep (* 100 1000))
        (incf time-count))
      (if (send (send self :get-robot-interface) :prismatic-calibratingp arm) nil t)))
  (:recognize-object (arm &key (trial-times 10))
    (let (is-recognized recognition-count label-names)
      (setq trial-fail-count- 0)
      (setq grasp-style- nil)
      (setq label-names
            (ros::get-param (format nil "/~a_hand_camera/label_names"
                                    (arm2str arm))))
      (send self :set-target-location arm :tote)
      (send (send self :get-robot-interface) :move-hand arm
            (send (send self :get-robot) :hand-grasp-pre-pose arm :opposed) 1000 :wait nil)
      (ros::ros-info "[main] Recognizing objects in tote")
      (unless (> start-picking-fail-count- 0)
        (send self :move-arm-body->tote-overlook-pose arm)
        (send (send self :get-robot-interface) :wait-interpolation))
      (setq recognition-count 1)
      (let ((stamp (ros::time-now)) target-index)
        (while (null (or (> recognition-count trial-times) is-recognized))
          (setq target-index
                (send self :get-largest-object-index arm :tote :stamp stamp
                      :n-random (if (> picking-fail-count- 0) 3 nil)))
          (setq is-recognized
            (when target-index
              (setq target-obj- (elt label-names target-index))
              (ros::set-dynparam
                (format nil "/~a_hand_camera/label_to_mask" (arm2str arm))
                (cons "label_value" target-index))
              (ros::set-param
                (format nil "~a_hand/target_object" (arm2str arm)) target-obj-)
              (ros::ros-info-blue "[~a] [main] :recognize-object, target-obj: ~a" (ros::get-name) target-obj-)

              (ros::service-call
                (format nil "/~a_hand_camera/image_buffer/save" (arm2str arm))
                (instance std_srvs::TriggerRequest :init))
              (send self :recognize-target-object arm :stamp (ros::time-now))))
          (incf recognition-count)))
      is-recognized))
  (:return-from-recognize-object (arm)
    (send self :add-postponed-object arm target-obj- :tote)
    (setq start-picking-fail-count- 0)
    (ros::ros-info "[main] arm: ~a, failed to recognize object in tote" arm)
    (send (send self :get-robot-interface) :angle-vector-sequence-raw
          (list (send (send self :get-robot) :fold-to-keep-object arm)
                (send (send self :get-robot) :arc-reset-pose arm))
          :fast (send (send self :get-robot-interface) :get-arm-controller arm) 0 :scale 3.0)
    (send (send self :get-robot-interface) :wait-interpolation))
  (:check-recognize-fail-count (arm &key (count-limit 2))
    (incf recognize-fail-count-)
    (ros::ros-info "[main] arm: ~a, recognize fail count: ~a" arm recognize-fail-count-)
    (let (recognize-failp)
      (setq recognize-failp (null (> count-limit recognize-fail-count-)))
      (if recognize-failp (setq recognize-fail-count- 0))
      recognize-failp))
  (:check-can-start-picking (arm)
    ;; first trial: trial-time = 1
    (setq grasp-style- (send self :get-grasp-style arm target-obj- 1))
    (ros::ros-info-green "arm: ~a, grasp-style: ~a, trial-time: ~a"
                         (arm2str arm) (symbol2str grasp-style-) (+ trial-fail-count- 1))
    (when grasp-style-
      (setq prev-target-obj- nil)
      (setq start-picking-fail-count- 0))
    (if grasp-style- t nil))
  (:check-start-picking-fail-count (arm &key (count-limit 2))
    (incf start-picking-fail-count-)
    (ros::ros-info "[main] arm: ~a, start picking fail count: ~a" arm start-picking-fail-count-)
    (send self :add-postponed-object arm target-obj- :tote)
    (when (and (> start-picking-fail-count- 1)
               (string= prev-target-obj- target-obj-))
      (setq prev-target-obj- nil)
      (return-from :check-start-picking-fail-count t))
    (setq prev-target-obj- target-obj-)
    (ros::ros-info "prev-target-obj :~A" prev-target-obj-)
    (let (start-picking-failp)
      (setq start-picking-failp (null (> count-limit start-picking-fail-count-)))
      (when start-picking-failp
        (setq prev-target-obj- nil)
        (setq start-picking-fail-count- 0))
      start-picking-failp))
  (:change-grasp-style (arm)
    (incf trial-fail-count-)
    (send (send self :get-robot-interface) :stop-grasp arm)
    (let (next-style)
      ;; try another grasp style
      (setq next-style (send self :get-grasp-style arm target-obj- (+ trial-fail-count- 1)))
      (ros::ros-info-green "arm: ~a, next grasp-style: ~a, trial-time: ~a"
                           (arm2str arm) (symbol2str next-style) (+ trial-fail-count- 1))
      (if next-style
        (progn
          (setq grasp-style- next-style)
          t)
        (progn
          (setq trial-fail-count- 0)
          nil))))
  (:pick-object
    (arm &key (use-scale t) (move-head t) (avoid-tote t))
    (send (send self :get-robot-interface) :speak
          (format nil "~a arm is picking ~a." (arm2str arm) (underscore-to-space target-obj-)))
    (let (pick-result graspingp)
      (setq recognize-fail-count- 0)
      (when moveit-p- (send self :delete-tote-scene arm))
      (if (eq grasp-style- :suction)
        (send self :set-movable-region-for-tote arm :offset (list 50 50 0))
        (send self :set-movable-region-for-tote arm :offset (list 50 50 0)))
      (when use-scale (send self :reset-scale arm))
      (when move-head
        (send (send self :get-robot) :head_pan :joint-angle (if (eq arm :larm) -70 70)))
      (setq pick-result
            (send self :pick-object-in-tote arm
                  :n-trial (if (eq grasp-style- :suction) 2 1)
                  :n-trial-same-pos 1
                  :do-stop-grasp nil
                  :grasp-style grasp-style-))
      (when moveit-p- (send self :add-tote-scene arm))
      (unless avoid-tote
        (return-from :pick-object (send (send self :get-robot-interface) :graspingp arm)))
      ;; Don't trust pressure sensor
      ;; (unless (eq pick-result :ik-failed)
      ;;   (setq graspingp (send (send self :get-robot-interface) :graspingp arm grasp-style-))
      ;;   (unless graspingp (return-from :pick-object nil))
      ;;   )
      (let (end-pos)
        (if (eq grasp-style- :suction)
          (progn
            (send self :ik->tote-center arm
                  :offset #f(0 0 400)
                  :rpy (float-vector (if (eq arm :larm) -pi/2 pi/2) 0 0)
                  :rotation-axis t :use-gripper t)
            (setq end-pos
                  (send (send (send self :get-robot) arm :end-coords :copy-worldcoords)
                        :worldpos))
            (send (send self :get-robot) :rotate-wrist-ik arm
                  (make-coords :pos end-pos
                               :rpy (float-vector (if (eq arm :larm) -pi/2 pi/2) 0 0))
                  :rotate-axis t))
          (progn
            (send self :ik->tote-center arm
                  :offset #f(0 0 400)
                  :rpy (float-vector (if (eq arm :larm) -pi/2 pi/2) 0 0)
                  :rotation-axis t :move-palm-end t)
            (setq end-pos
                  (send (send (send self :get-robot) arm :palm-endpoint :copy-worldcoords)
                        :worldpos))
            (send (send self :get-robot) :rotate-wrist-ik arm
                  (make-coords :pos end-pos
                               :rpy (float-vector (if (eq arm :larm) -pi/2 pi/2) 0 0))
                  :move-palm-end t
                  :rotate-axis t)))
        (send (send self :get-robot-interface) :angle-vector-raw
              (send (send self :get-robot) :angle-vector) 3000
              (send (send self :get-robot-interface) :get-arm-controller arm :gripper nil) 0)
        (send (send self :get-robot-interface) :wait-interpolation))
      (if (eq pick-result :ik-failed) (return-from :pick-object nil))
      (unless use-scale
        (return-from :pick-object (send (send self :get-robot-interface) :graspingp arm)))
      ;; Don't trust pressure sensor
      ;; (setq graspingp (send (send self :get-robot-interface) :graspingp arm grasp-style-))
      ;; (ros::ros-info "[main] arm: ~a graspingp: ~a" arm graspingp)
      ;; graspingp))
      (setq scale-candidates-
            (send self :get-scale-candidates arm :picked :stamp (ros::time-now)))
      ;; If scale value isn't changed, we don't have to verify-object and return-object
      (if (eq scale-candidates- :no-change)
        (progn
          (ros::ros-info-green "[main] arm ~a: grasping nothing is detected by scale" arm)
          nil)
        t)))
  (:return-from-pick-object (arm)
    (send self :add-postponed-object arm target-obj- :tote)
    (ros::ros-info "[main] arm: ~a return from pick-object to arc-reset-pose" arm)
    (send (send self :get-robot-interface) :angle-vector-sequence-raw
          (list (send (send self :get-robot) :avoid-shelf-pose arm (if (eq arm :larm) :d :f))
                (send (send self :get-robot) :arc-reset-pose arm))
          :fast (send (send self :get-robot-interface) :get-arm-controller arm
                      :gripper (eq grasp-style- :suction))
          0 :scale 3.0)
    (send (send self :get-robot-interface) :wait-interpolation)
    (if (eq grasp-style- :pinch)
      (send self :resolve-collision-between-fingers arm)))
  (:verify-object (arm)
    (let ((stamp (ros::time-now)))
      (when (eq scale-candidates- :timeout)
        (ros::ros-error "subscribed weight candidates, but timeout")
        (return-from :verify-object nil))
      (ros::ros-info-green "[main] arm ~a: scale candidates ~a" arm scale-candidates-)
      (unless scale-candidates- (return-from :verify-object nil))
      (when (eq (length scale-candidates-) 1)
        (setq target-obj- (car scale-candidates-))
        (return-from :verify-object t))
      (if (find target-obj- scale-candidates- :test #'string=)
        t  ;; scale can classify the grasped object
        (progn  ;; scale narrows down candidate objects
          (let* ((task-duration (- (send (ros::time-now) :to-sec) (send start-time- :to-sec)))
                 (proba-thre (- 0.80 (* (/ 0.2 900.0) task-duration)))
                 msg new-order)
            (setq msg
              (one-shot-subscribe
                (format nil "/~a_hand_camera/probability_image_classifier/output" (arm2str arm))
                jsk_recognition_msgs::ClassificationResult
                :timeout 15000
                :after-stamp stamp))
            (unless msg (return-from :verify-object nil))
            (ros::ros-info-blue "classifier result: arm ~a, object ~a, proba ~a"
                                (arm2str arm) (car (send msg :label_names))
                                (elt (send msg :label_proba) 0))
            (ros::ros-info-cyan "task time: ~A [s], arm ~A, classifier threshold: ~A"
                                task-duration (arm2str arm) proba-thre)
            (if (> (elt (send msg :label_proba) 0) proba-thre)
                (setq target-obj- (car (send msg :label_names)))
              (return-from :verify-object nil))
            (ros::ros-info-blue "[~a] [main] :verify-object, overwrite target, target-obj: ~a"
                                (ros::get-name) target-obj-)
            t)))))
  (:check-picking-fail-count (arm &key (count-limit 2))
    (incf picking-fail-count-)
    (ros::ros-info "[main] arm: ~a, picking fail count: ~a" arm picking-fail-count-)
    (let (picking-failp)
      (setq picking-failp (null (> count-limit picking-fail-count-)))
      (if picking-failp (setq picking-fail-count- 0))
      picking-failp))
  (:set-target-bin (arm)
    (setq picking-fail-count- 0)
    (setq target-bin- (send self :get-stowing-target-bin arm))
    (if (eq target-bin- :tote)
      (progn
        (ros::ros-info "[main] unable to recognize, return to tote")
        nil)
      (progn
        (ros::set-param (format nil "~a_hand/target_bin" (arm2str arm)) (symbol2str target-bin-))
        (ros::ros-info "[main] target-bin: ~a" target-bin-)
        t)))
  (:return-object (arm)
    (send self :add-postponed-object arm target-obj- :tote)
    (when moveit-p- (send self :delete-tote-scene arm))
    (let ((offset (send self :decide-tote-return-area arm)))
      ;; (setq offset (send self :decide-tote-return-area arm))
      ;; (ros::ros-info "return object to tote: arm, ~A, offset ~A" (arm2str arm) offset)
      (if (eq grasp-style- :suction)
        (send (send self :get-robot-interface) :angle-vector-raw
              (send self :ik->tote-center arm
                    :offset offset
                    :rotation-axis :z :use-gripper t)
              :fast (send (send self :get-robot-interface) :get-arm-controller arm) 0 :scale 5.0)
        (send (send self :get-robot-interface) :angle-vector-raw
              (send self :ik->tote-center arm
                    :offset offset
                    :rotation-axis :z :move-palm-end t)
            :fast (send (send self :get-robot-interface) :get-arm-controller arm :gripper nil)
            0 :scale 5.0)))
    (send (send self :get-robot-interface) :wait-interpolation)
    (send (send self :get-robot-interface) :stop-grasp arm)
    (when (eq grasp-style- :pinch)
      (send (send self :get-robot-interface) :stop-grasp arm :pinch)
      (send (send self :get-robot) :rotate-gripper arm 90 :relative nil))
    (send self :spin-off-by-wrist arm :times 3)
    (send (send self :get-robot-interface) :wait-interpolation)
    (ros::ros-info "[main] ~a, return object in tote" arm)
    (send (send self :get-robot-interface) :angle-vector-raw
          (send (send self :get-robot) arm :move-end-pos #f(0 0 200) :world :rotation-axis :z)
          :fast (send (send self :get-robot-interface) :get-arm-controller arm) 0 :scale 5.0)
    (send (send self :get-robot-interface) :wait-interpolation)
    (when moveit-p- (send self :add-tote-scene arm))
    (send self :arc-reset-pose arm 3000
          (send (send self :get-robot-interface) :get-arm-controller arm) 0 :move-arm arm)
    (send (send self :get-robot-interface) :wait-interpolation)
    (if (eq grasp-style- :pinch)
      (send self :resolve-collision-between-fingers arm)))
  (:place-object (arm)
    (let (dropped place-av)
      (send (send self :get-robot-interface) :speak
            (format nil "~a arm is placing ~a into bin ~a."
                    (arm2str arm) (underscore-to-space target-obj-) target-bin-))
      (ros::ros-info "[main] ~a, place object in bin ~a" arm target-bin-)
      (when moveit-p- (send self :add-object-in-hand-scene arm target-obj-))
      (send (send self :get-robot) :head_pan :joint-angle (if (eq arm :larm) 70 -70))
      (if (eq grasp-style- :suction)
        (send (send self :get-robot-interface) :angle-vector
              (send self :ik->bin-center arm target-bin-
                    :offset (v+ #f(0 0 300)
                                (send self :decide-bin-place-area arm target-bin- grasp-style-))
                    :rotation-axis :z :use-gripper t)
              4000 (send (send self :get-robot-interface) :get-arm-controller arm :head t)
              0 :move-arm arm)
        (send (send self :get-robot-interface) :angle-vector
              (send self :ik->bin-center arm target-bin-
                    :offset (v+ #f(0 0 300)
                                (send self :decide-bin-place-area arm target-bin- grasp-style-))
                    :rotation-axis nil :move-palm-end t)
              4000 (send (send self :get-robot-interface)
                         :get-arm-controller arm :gripper nil :head t)
              0 :move-arm arm))
      (send (send self :get-robot-interface) :wait-interpolation)
      (setq dropped (not (send (send self :get-robot-interface) :graspingp arm grasp-style-)))
      (when moveit-p- (send self :delete-object-in-hand-scene arm))
      (if dropped
        (progn
          (send (send self :get-robot-interface) :speak
                (format nil "~a arm dropped ~a to somewhere."
                        (arm2str arm) (underscore-to-space target-obj-)))
          (ros::ros-error "[main] arm ~a: dropped object" arm)
          (send (send self :get-robot-interface) :stop-grasp arm)
          (when (eq grasp-style- :pinch)
            (send (send self :get-robot-interface) :stop-grasp arm :pinch)
            (send (send self :get-robot) :rotate-gripper arm 90 :relative nil))
          ;; we assume object is dropped, but it can be just a mis-detection of grasp
          (send self :spin-off-by-wrist arm :times 3)
          (send (send self :get-robot-interface) :wait-interpolation)
          (send self :add-postponed-object arm target-obj- :tote))
        (progn
          (send self :update-json target-obj- :src :tote :dst (cons :bin target-bin-))
          (send self :add-finished-object arm target-obj- :tote)
          (ros::ros-info-green "[main] arm ~a: place object ~a in bin ~a" arm target-obj- target-bin-)
          (when moveit-p- (send self :delete-bin-scene target-bin-))
          (if (eq grasp-style- :suction)
            (send (send self :get-robot-interface) :angle-vector-raw
                  (send (send self :get-robot) arm :move-end-pos #f(0 0 -300) :world
                        :rotation-axis :z
                        :use-gripper t)
                  2000 (send (send self :get-robot-interface) :get-arm-controller arm) 0)
            (progn
              (setq place-av
                    (send (send self :get-robot) arm :move-end-pos #f(0 0 -250)
                          :world :rotation-axis nil))
              (if place-av
                (send (send self :get-robot-interface) :angle-vector-raw place-av 2000
                      (send (send self :get-robot-interface) :get-arm-controller arm) 0))))
          (send (send self :get-robot-interface) :wait-interpolation)
          (send (send self :get-robot-interface) :stop-grasp arm) ;; release object
          (when (eq grasp-style- :pinch)
            (send (send self :get-robot-interface) :stop-grasp arm :pinch)
            (send (send self :get-robot) :rotate-gripper arm 90 :relative nil)) ;; release object
          (send self :spin-off-by-wrist arm :times 5)
          (send (send self :get-robot-interface) :wait-interpolation)
          (if (eq grasp-style- :suction)
            (send (send self :get-robot-interface) :angle-vector-raw
                  (send (send self :get-robot) arm :move-end-pos #f(0 0 300) :world
                        :rotation-axis :z
                        :use-gripper t)
                  2000 (send (send self :get-robot-interface) :get-arm-controller arm) 0)
            (if place-av
                (send (send self :get-robot-interface) :angle-vector-raw
                      (send (send self :get-robot) arm :move-end-pos #f(0 0 250) :world
                            :rotation-axis nil)
                      2000 (send (send self :get-robot-interface) :get-arm-controller arm) 0)))
          (send (send self :get-robot-interface) :wait-interpolation)
          (when moveit-p- (send self :add-bin-scene target-bin-))))))
  (:return-from-place-object (arm)
    (send self :arc-reset-pose arm 3000
          (send (send self :get-robot-interface) :get-arm-controller arm) 0 :move-arm arm)
    (send (send self :get-robot-interface) :wait-interpolation)
    (if (eq grasp-style- :pinch)
      (send self :resolve-collision-between-fingers arm))
    (setq grasp-style- nil)))


(defun jsk_arc2017_baxter::stow-init (&key (ctype :default-controller) (moveit nil))
  (let (mvit-env mvit-rb)
    (when moveit
      (setq mvit-env (instance jsk_arc2017_baxter::baxter-moveit-environment))
      (setq mvit-rb (instance jsk_arc2017_baxter::baxter-robot :init)))
    (unless (boundp '*ri*)
      (setq *ri* (instance jsk_arc2017_baxter::baxter-interface :init :type ctype
                           :moveit-environment mvit-env
                           :moveit-robot mvit-rb)))
    (unless (boundp '*baxter*)
      (setq *baxter* (instance jsk_arc2017_baxter::baxter-robot :init)))
    (unless (boundp '*co*)
      (setq *co* (when moveit (instance collision-object-publisher :init))))
    (unless (boundp '*ti*)
      (setq *ti* (instance jsk_arc2017_baxter::stow-interface :init :moveit moveit))
      (send *ti* :set-robot-interface *ri*))
    (send (send *ti* :get-robot) :angle-vector
          (send (send *ti* :get-robot-interface) :state :potentio-vector))
    (send (send *ti* :get-robot-interface) :calib-grasp :arms)))
