#!/usr/bin/env roseus

(require "package://jsk_2015_05_baxter_apc/euslisp/lib/util.l")
(require "package://jsk_2016_01_baxter_apc/euslisp/lib/util.l")
(require "package://jsk_arc2017_baxter/euslisp/lib/arc-interface.l")

(ros::load-ros-manifest "jsk_arc2017_baxter")

(unless (find-package "JSK_ARC2017_BAXTER")
  (make-package "JSK_ARC2017_BAXTER"))

(defclass jsk_arc2017_baxter::pick-interface
  :super jsk_arc2017_baxter::arc-interface
  :slots (bins-
          grasp-style-
          label-names-
          moveit-p-
          moving-object-p-
          picking-fail-count-
          prev-target-obj-
          recognize-fail-count-
          scale-candidates-
          start-picking-fail-count-
          target-bin-
          target-cardboard-
          target-obj-
          trial-fail-count-))

(defmethod jsk_arc2017_baxter::pick-interface
  (:init (&key (moveit nil))
    (send-super :init)
    (setq picking-fail-count- 0)
    (setq trial-fail-count- 0)
    (setq recognize-fail-count- 0)
    (setq start-picking-fail-count- 0)
    (setq bins- (list :a :b :c))
    (setq moveit-p- moveit)
    (setq grasp-style- nil))
  (:recognize-bboxes ()
    (ros::ros-info "[main] recognizing shelf bin boxes")
    (send self :recognize-bin-boxes :stamp (ros::time-now))
    (ros::ros-info "[main] recognizing cardboard boxes")
    (send self :recognize-cardboard-boxes :stamp (ros::time-now))
    ;; XXX: movable region will be set in :pick-object
    ;; (send self :set-movable-region-for-bin :offset (list 80 120 0))
    )
  (:wait-for-user-input (arm)
    (let (can-start)
      (when moveit-p-
        (send self :add-shelf-scene)
        ;; (send self :add-cardboard-rack-scene)
        )
      (send (send self :get-robot-interface) :speak
            (format nil "~a arm is waiting for user input to start the picking task."
                    (arm2str arm)))
      (ros::ros-info "[:wait-for-user-input] wait for user input to start: ~a" arm)
      (ros::wait-for-service "/rviz/yes_no_button")
      (setq can-start
            (send (ros::service-call "/rviz/yes_no_button"
                                     (instance jsk_gui_msgs::YesNoRequest))
                  :yes))
      (ros::ros-info "[:wait-for-user-input] received user input: ~a" arm)
      (send self :get-work-orders arm)
      (send self :reset-object-tables :bins)
      (dolist (bin bins-)
        (send self :set-objects-param arm bin :object-type :all))
      can-start))
  (:calib-prismatic-joint (arm &key (timeout 10))
    (send (send self :get-robot-interface) :start-prismatic-calib arm)
    (let ((time-count 1))
      (while (and (send (send self :get-robot-interface) :prismatic-calibratingp arm)
                  (> (* timeout 10) time-count))
        (unix::usleep (* 100 1000))
        (incf time-count))
      (if (send (send self :get-robot-interface) :prismatic-calibratingp arm) nil t)))
  (:set-target (arm)
    (setq label-names-
          (ros::get-param (format nil "/~a_hand_camera/label_names"
                                  (arm2str arm))))
    ;; get target param
    (setq target-bin- (send self :get-next-target-bin arm))
    (unless (send self :check-bin-exist target-bin-)
      (ros::ros-warn "[~a] [main] could not find bin box: ~a" (ros::get-name) target-bin-)
      (return-from :set-target nil))
    (ros::set-dynparam
      (format nil "/~a_hand_camera/bbox_array_to_bbox" (arm2str arm))
      (cons "index" (position target-bin- bins-)))
    (send self :set-target-location arm (cons :bin target-bin-))
    (ros::set-param
      (format nil "~a_hand/target_bin" (arm2str arm))
      (symbol2str target-bin-))
    (unless target-bin-
      (ros::ros-info "there is no order to execute"))
    (if target-bin- t nil))
  (:recognize-object (arm &key (trial-times 10))
    (let (is-recognized recognition-count)
      (setq trial-fail-count- 0)
      (setq grasp-style- nil)
      (send (send self :get-robot-interface) :move-hand arm
            (send *baxter* :hand-grasp-pre-pose arm :opposed) 1000 :wait nil)
      (ros::ros-info "[main] Recognizing objects in bin ~a" target-bin-)
      (unless (> start-picking-fail-count- 0)
        (send (send self :get-robot-interface) :speak (format nil "~a arm is going to bin ~a." (arm2str arm) target-bin-))
        (send self :move-arm-body->bin-overlook-pose arm target-bin-)
        (send (send self :get-robot-interface) :wait-interpolation))
      (setq recognition-count 1)
      (let ((stamp (ros::time-now)) order)
        (while (null (or (> recognition-count trial-times) is-recognized))
          (setq order (send self :select-work-order-in-bin arm target-bin- :stamp stamp))
          (setq is-recognized
            (when order
              (setq target-obj- (send order :item))
              (setq target-cardboard- (str2symbol (send order :box)))
              (ros::set-param
                (format nil "~a_hand/target_object" (arm2str arm)) target-obj-)
              (ros::set-dynparam
                (format nil "/~a_hand_camera/label_to_mask" (arm2str arm))
                (cons "label_value" (position target-obj- label-names- :test #'string=)))
              (ros::ros-info-blue "[~a] [main] :recognize-object, target-bin: ~a, target-obj: ~a" (ros::get-name) target-bin- target-obj-)
              (ros::service-call
                (format nil "/~a_hand_camera/image_buffer/save" (arm2str arm))
                (instance std_srvs::TriggerRequest :init))
              (send self :recognize-target-object arm :stamp (ros::time-now))))
          (incf recognition-count)))
      (unless is-recognized
        (setq recognition-count 1)
        (let ((stamp (ros::time-now)) target-index)
          (while (null (or (> recognition-count trial-times) is-recognized))
            (setq target-index
                  (send self :get-largest-object-index arm target-bin-
                        :stamp stamp :n-random nil))
            (setq is-recognized
              (when target-index
                (setq target-obj- (elt label-names- target-index))
                (ros::set-dynparam
                  (format nil "/~a_hand_camera/label_to_mask" (arm2str arm))
                  (cons "label_value" target-index))
                (ros::set-param
                  (format nil "~a_hand/target_object" (arm2str arm)) target-obj-)
                (ros::ros-info-blue "[~a] [main] target recognition failed" (ros::get-name) target-bin-)
                (ros::ros-info-blue "[~a] [main] moving object in bin ~A" (ros::get-name) target-bin-)
                (ros::ros-info-blue "[~a] [main] :recognize-object, target-obj: ~a" (ros::get-name) target-obj-)
                (setq moving-object-p- t)
                (send self :recognize-target-object arm :stamp (ros::time-now))))
            (incf recognition-count))))
      is-recognized))
  (:return-from-recognize-object (arm)
    (send self :add-postponed-object arm target-obj- target-bin-)
    (setq start-picking-fail-count- 0)
    (ros::ros-info "[main] arm: ~a, failed to recognize object ~a" arm target-obj-)
    (send (send self :get-robot-interface) :angle-vector-sequence-raw
          (list (send *baxter* :fold-to-keep-object arm)
                (send *baxter* :arc-reset-pose arm))
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
    (send self :add-postponed-object arm target-obj- target-bin-)
    (when (and (> start-picking-fail-count- 1)
               (string= prev-target-obj- target-obj-))
      (setq prev-target-obj- nil)
      (return-from :check-start-picking-fail-count t))
    (setq prev-target-obj- target-obj-)
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
    (arm)
    (send (send self :get-robot-interface) :speak
          (format nil "~a arm is picking ~a, which is~a an ordered object."
                  (arm2str arm)
                  (underscore-to-space target-obj-)
                  (if moving-object-p- " not" "")))
    (let (pick-result graspingp)
      ;; slots reset
      (setq start-picking-fail-count- 0)
      (setq recognize-fail-count- 0)
      (when moveit-p- (send self :delete-bin-scene target-bin-))
      (if (eq grasp-style- :suction)
        (send self :set-movable-region-for-bin :offset (list 50 50 0))
        (send self :set-movable-region-for-bin :offset (list 50 50 0)))
      (send self :reset-scale arm)
      ; (send *baxter* :head_pan :joint-angle (if (eq arm :larm) -70 70))
      (setq pick-result
            (send self :pick-object-in-bin arm target-bin-
                  :n-trial (if (eq grasp-style- :suction) 2 1)
                  :n-trial-same-pos 1
                  :do-stop-grasp nil
                  :grasp-style grasp-style-))
      (when moveit-p- (send self :add-bin-scene target-bin-))
      ;; Don't trust pressure sensor
      ;; (unless (eq pick-result :ik-failed)
      ;;   (setq graspingp (send (send self :get-robot-interface) :graspingp arm grasp-style-))
      ;;   (unless graspingp (return-from :pick-object nil))
      ;;   )
      (let (avs end-pos)
        (if (eq grasp-style- :suction)
          (progn
            (send self :ik->bin-center arm target-bin-
                  :offset #f(0 0 300)
                  :rotation-axis :z :use-gripper t)
            (setq end-pos (send (send *baxter* arm :end-coords :copy-worldcoords) :worldpos))
            (send *baxter* :rotate-wrist-ik arm
                  (make-coords :pos end-pos
                               :rpy (float-vector (if (eq arm :larm) -pi/2 pi/2) 0 0))
                  :rotate-axis t))
          (progn
            (send self :ik->bin-center arm target-bin-
                  :offset #f(0 0 300)
                  :rotation-axis :z :move-palm-end t)
            (setq end-pos (send (send *baxter* arm :palm-endpoint :copy-worldcoords) :worldpos))
            (send *baxter* :rotate-wrist-ik arm
                  (make-coords :pos end-pos
                               :rpy (float-vector (if (eq arm :larm) -pi/2 pi/2) 0 0))
                  :move-palm-end t
                  :rotate-axis t)))
        (send (send self :get-robot-interface) :angle-vector-raw (send *baxter* :angle-vector) 3000
              (send (send self :get-robot-interface) :get-arm-controller arm :gripper nil) 0)
        (send (send self :get-robot-interface) :wait-interpolation))
      (if (eq pick-result :ik-failed) (return-from :pick-object nil))
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
    (send self :add-postponed-object arm target-obj- target-bin-)
    (setq start-picking-fail-count- 0)
    (send (send self :get-robot-interface) :angle-vector-sequence-raw
          (list (send *baxter* :avoid-shelf-pose arm (if (eq arm :larm) :d :f))
                (send *baxter* :arc-reset-pose arm))
          :fast (send (send self :get-robot-interface) :get-arm-controller arm
                      :gripper (eq grasp-style- :suction))
          0 :scale 3.0)
    (send (send self :get-robot-interface) :wait-interpolation)
    (if (eq grasp-style- :pinch)
      (send self :resolve-collision-between-fingers arm)))
  (:verify-object (arm)
    (when moving-object-p-
      (setq moving-object-p- nil)
      (ros::ros-info "moving object in bin")
      (return-from :verify-object nil))
    (let ((stamp (ros::time-now)))
      (when (eq scale-candidates- :timeout)
        (ros::ros-error "subscribed weight candidates, but timeout")
        (return-from :verify-object nil))
      (ros::ros-info-green "[main] arm ~a: scale candidates ~a" arm scale-candidates-)
      (unless scale-candidates- (return-from :verify-object nil))
      (if (find target-obj- scale-candidates- :test #'string=)
        t  ;; scale can classify the grasped object
        (progn  ;; scale narrows down candidate objects
          (let (msg (proba-thre 0.80) new-order)
            (if (eq (length scale-candidates-) 1)
              (setq new-order (send self :check-exist-in-order (car scale-candidates-)))
              (progn
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
                (when (> (elt (send msg :label_proba) 0) proba-thre)
                  (setq new-order
                        (send self :check-exist-in-order (car (send msg :label_names)))))))
            (if new-order
              (progn
                (setq target-obj- (send new-order :item))
                (setq target-cardboard- (str2symbol (send new-order :box)))
                (ros::ros-info-blue "[~a] [main] :verify-object, overwrite target, target-obj: ~a"
                                    (ros::get-name) target-obj-)
                t)
              nil))))))
  (:check-picking-fail-count (arm &key (count-limit 2))
    (incf picking-fail-count-)
    (ros::ros-info "[main] arm: ~a, picking fail count: ~a" arm picking-fail-count-)
    (let (picking-failp)
      (setq picking-failp (null (> count-limit picking-fail-count-)))
      (if picking-failp (setq picking-fail-count- 0))
      picking-failp))
  (:set-target-cardboard (arm)
    (setq picking-fail-count- 0)
    (if (eq target-cardboard- :shelf)
      (progn
        (ros::ros-info "[main] blacklisted object, return to shelf")
        nil)
      (progn
        (ros::set-param (format nil "~a_hand/target_box" (arm2str arm)) (symbol2str target-cardboard-))
        ;; logging
        (ros::ros-info "[main] target-cardboard: ~a" target-cardboard-)
        t)))
  (:return-object (arm)
    (send self :add-postponed-object arm target-obj- target-bin-)
    (when moveit-p- (send self :delete-bin-scene target-bin-))
    (if (eq grasp-style- :suction)
      (send (send self :get-robot-interface) :angle-vector-raw
            (send self :ik->bin-center arm target-bin-
                  :offset (send self :decide-bin-return-area arm target-bin-)
                  :rotation-axis :z :use-gripper t)
            :fast (send (send self :get-robot-interface) :get-arm-controller arm) 0 :scale 5.0)
      (send (send self :get-robot-interface) :angle-vector-raw
            (send self :ik->bin-center arm target-bin-
                  :offset (send self :decide-bin-return-area arm target-bin-)
                  :rotation-axis :z :move-palm-end t)
            :fast (send (send self :get-robot-interface) :get-arm-controller arm :gripper nil)
            0 :scale 5.0))
    (send (send self :get-robot-interface) :wait-interpolation)
    (send (send self :get-robot-interface) :stop-grasp arm)
    (when (eq grasp-style- :pinch)
      (send (send self :get-robot-interface) :stop-grasp arm :pinch)
      (send *baxter* :rotate-gripper arm 90 :relative nil))
    (send self :spin-off-by-wrist arm :times 3)
    (send (send self :get-robot-interface) :wait-interpolation)
    (ros::ros-info "[main] ~a, return object in shelf" arm)
    (send (send self :get-robot-interface) :angle-vector-raw
          (send *baxter* arm :move-end-pos #f(0 0 200) :world :rotation-axis :z)
          :fast (send (send self :get-robot-interface) :get-arm-controller arm) 0 :scale 5.0)
    (send (send self :get-robot-interface) :wait-interpolation)
    (when moveit-p- (send self :add-bin-scene target-bin-))
    (send self :arc-reset-pose arm)
    (send (send self :get-robot-interface) :wait-interpolation)
    (if (eq grasp-style- :pinch)
      (send self :resolve-collision-between-fingers arm)))
  (:place-object (arm)
    (let (dropped end-coords-name orientation path-constraints orientation-constraint place-z)
      ;; FIXME: moveit cannot solve path
      ;; moveit path constraints
      ;; (setq end-coords-name
      ;;       (if (eq arm :larm) "left_gripper_pad_with_base" "right_gripper_pad_with_base"))
      ;; (setq orientation (instance geometry_msgs::Quaternion :init :x 0 :y 0 :z 0 :w 1))
      ;; (setq orientation-constraint
      ;;       (instance moveit_msgs::OrientationConstraint :init
      ;;                 :link_name end-coords-name
      ;;                 :orientation orientation))
      ;; (send orientation-constraint :header :frame_id end-coords-name)
      ;; (setq path-constraints (instance moveit_msgs::Constraints :init
      ;;                                  :orientation_constraints (list orientation-constraint)))

      (send (send self :get-robot-interface) :speak
            (format nil "~a arm is placing ~a into box ~a."
                    (arm2str arm) (underscore-to-space target-obj-) target-cardboard-))
      (ros::ros-info "[main] ~a, place object in bin ~a" arm target-cardboard-)
      ;; (when moveit-p- (send self :add-object-in-hand-scene arm target-obj-))

      ;; FIXME: moveit cannot solve path
      ;; (send (send self :get-robot-interface) :angle-vector
      ;;       (send self :ik->cardboard-center arm target-cardboard-
      ;;             :offset #f(0 0 200) :rotation-axis :z :use-gripper t)
      ;;       4000 (send (send self :get-robot-interface) :get-arm-controller arm) 0
      ;;       :path-constraints path-constraints)
      (let (avs (offsets (list #f(-50 0 500) #f(-50 0 200) #f(0 0 200) #f(30 0 200))))
        (dolist (offset offsets)
          (pushback
            (send self :ik->cardboard-center arm target-cardboard-
                  :offset offset
                  :rotation-axis (if (eq grasp-style- :suction) :z nil)
                  :use-gripper (if (eq grasp-style- :suction) t nil)
                  :move-palm-end (if (eq grasp-style- :suction) nil t))
            avs))
        (setq avs (remove nil avs))
        (setq place-z 50)
        (if (eq grasp-style- :suction)
          (send (send self :get-robot-interface) :angle-vector-sequence-raw avs :fast
                (send (send self :get-robot-interface) :get-arm-controller arm
                      :gripper t :head t) 0 :scale 7.0)
          (send (send self :get-robot-interface) :angle-vector-sequence-raw avs :fast
                (send (send self :get-robot-interface) :get-arm-controller arm
                      :gripper nil :head t) 0 :scale 7.0))
        (send (send self :get-robot-interface) :wait-interpolation))
      (setq dropped (not (send (send self :get-robot-interface) :graspingp arm grasp-style-)))
      ;; (when moveit-p- (send self :delete-object-in-hand-scene arm))
      (if dropped
        (progn
          (send (send self :get-robot-interface) :speak
                (format nil "~a arm dropped ~a to somewhere."
                        (arm2str arm) (underscore-to-space target-obj-)))
          (ros::ros-error "[main] arm ~a: dropped object" arm)
          (send (send self :get-robot-interface) :stop-grasp arm)
          (when (eq grasp-style- :pinch)
            (send (send self :get-robot-interface) :stop-grasp arm :pinch)
            (send *baxter* :rotate-gripper arm 90 :relative nil))
          ;; we assume object is dropped, but it can be just a mis-detection of grasp
          (send self :spin-off-by-wrist arm :times 3)
          (send (send self :get-robot-interface) :wait-interpolation)
          (send self :add-postponed-object arm target-obj- target-bin-))
        (let (av)
          (ros::ros-info-green "[main] arm ~a: place object ~a in cardboard ~a" arm target-obj- target-cardboard-)
          (send self :update-json target-obj-
                :src (cons :bin target-bin-) :dst (cons :cardboard target-cardboard-))
          (send self :add-finished-object arm target-obj- target-bin-)
          (when moveit-p- (send self :delete-cardboard-scene target-cardboard-))
          (setq av (if (eq grasp-style- :suction)
                     (send *baxter* arm :move-end-pos (float-vector 0 0 (- place-z)) :world)
                     (send *baxter* arm :move-end-pos
                       (float-vector 0 0 (- place-z)) :world :rotation-axis nil)))
          (when av
            (send (send self :get-robot-interface) :angle-vector-raw av
                  2000 (send (send self :get-robot-interface) :get-arm-controller arm) 0)
            (send (send self :get-robot-interface) :wait-interpolation))
          (send (send self :get-robot-interface) :stop-grasp arm) ;; release object
          (when (eq grasp-style- :pinch)
            (send (send self :get-robot-interface) :stop-grasp arm :pinch)
            (send *baxter* :rotate-gripper arm 90 :relative nil)) ;; release object
          (send self :spin-off-by-wrist arm :times 5)
          (send (send self :get-robot-interface) :wait-interpolation)
          (when av
            (setq av (if (eq grasp-style- :suction)
                       (send *baxter* arm :move-end-pos (float-vector 0 0 place-z) :world
                             :rotation-axis :z :use-gripper t)
                       (send *baxter* arm :move-end-pos (float-vector 0 0 place-z) :world
                             :rotation-axis nil)))
            (send (send self :get-robot-interface) :angle-vector-raw av
                  1000 (send (send self :get-robot-interface) :get-arm-controller arm) 0)
            (send (send self :get-robot-interface) :wait-interpolation))
          (when moveit-p- (send self :add-cardboard-scene target-cardboard-))))))
  (:return-from-place-object (arm)
    (let (avs)
      (setq avs (list (send *baxter* :avoid-shelf-pose arm (if (eq arm :larm) :d :f))))
      (pushback (send *baxter* :arc-reset-pose arm) avs)
      (send (send self :get-robot-interface) :angle-vector-sequence avs 5000
            (send (send self :get-robot-interface) :get-arm-controller arm) 0)
      (send (send self :get-robot-interface) :wait-interpolation)
      (if (eq grasp-style- :pinch)
        (send self :resolve-collision-between-fingers arm))
      (setq grasp-style- nil))))


(defun jsk_arc2017_baxter::pick-init (&key (ctype :default-controller) (moveit nil))
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
      (setq *ti* (instance jsk_arc2017_baxter::pick-interface :init :moveit moveit))
      (send *ti* :set-robot-interface *ri*))
    (send *baxter* :angle-vector (send (send *ti* :get-robot-interface) :state :potentio-vector))
    (send (send *ti* :get-robot-interface) :calib-grasp :arms)))
