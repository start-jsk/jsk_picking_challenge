;; -*- mode: lisp;-*-

(require "package://jsk_2015_05_baxter_apc/euslisp/lib/util.l")
(require "package://jsk_2016_01_baxter_apc/euslisp/lib/util.l")
(require "package://jsk_arc2017_baxter/euslisp/lib/baxter-interface.l")

(ros::load-ros-manifest "jsk_arc2017_baxter")

(unless (find-package "JSK_ARC2017_BAXTER")
  (make-package "JSK_ARC2017_BAXTER"))

(defclass jsk_arc2017_baxter::arc-interface
  :super propertied-object
  :slots (attached-objects-
          bbox-volume-threshold-
          bin-cubes-
          bin-movable-regions-
          bin-object-volumes-
          cardboard-cubes-
          finished-objects-
          object-boxes-
          object-coords-
          orders-
          postponed-objects-
          rack-leg-cubes-
          robot-
          robot-interface-
          tote-cubes-
          tote-movable-regions-
          unfinished-objects-
          ))

(defmethod jsk_arc2017_baxter::arc-interface
  (:init ()
    ;; initialize slots
    (setq bbox-volume-threshold- 0.01) ;; [m^3]
    (setq attached-objects- (make-hash-table))
    (setq bin-cubes- (make-hash-table))
    (setq bin-movable-regions- (make-hash-table))
    (setq bin-object-volumes- (make-hash-table))
    (dolist (bin (list :a :b :c))
      (sethash bin bin-object-volumes- 0))
    (setq object-boxes- (make-hash-table))
    (setq object-coords- (make-hash-table))
    (setq cardboard-cubes- (make-hash-table))
    (setq tote-cubes- (make-hash-table))
    (setq tote-movable-regions- (make-hash-table))
    (setq postponed-objects- (make-hash-table))
    (setq rack-leg-cubes- (make-hash-table))
    (setq finished-objects- (make-hash-table))
    (setq unfinished-objects- (make-hash-table)))
  (:set-robot-interface (ri) (setq robot-interface- ri))
  (:get-robot-interface ()
    (if robot-interface- robot-interface- (if (boundp '*ri*) *ri* nil)))
  (:set-robot (rb) (setq robot- rb))
  (:get-robot () (if robot- robot- (if (boundp '*baxter*) *baxter* nil)))
  (:arc-reset-pose
    (&optional (arm :arms) &rest args)
    (dolist (tmp-arm (if (eq arm :arms) (list :rarm :larm) (list arm)))
      (send (send self :get-robot) :arc-reset-pose tmp-arm))
    (send* (send self :get-robot-interface) :angle-vector
           (send (send self :get-robot) :angle-vector) args))
  (:fold-pose-back
    (&optional (arm :arms) &rest args)
    (dolist (tmp-arm (if (eq arm :arms) (list :rarm :larm) (list arm)))
      (send (send self :get-robot) :fold-pose-back tmp-arm))
    (send* (send self :get-robot-interface) :angle-vector
           (send (send self :get-robot) :angle-vector) args))
  (:bbox->cube
    (bbox)
    (let* ((dims (ros::tf-point->pos (send bbox :dimensions)))
           (bx (make-cube (elt dims 0) (elt dims 1) (elt dims 2))))
      (send bx :newcoords
            (send (send self :get-robot-interface) :tf-pose->coords
                  (send bbox :header :frame_id)
                  (send bbox :pose)))
      bx))
  (:cube->movable-region
    (cube &key (offset (list 0 0 0)))
    (let (movable-region vertex-values)
      (send cube :worldcoords)
      (dotimes (i 3)
        (setq vertex-values (mapcar #'(lambda (x) (aref x i)) (send cube :vertices)))
        (pushback (list
                    (+ (apply #'min vertex-values) (elt offset i))
                    (- (apply #'max vertex-values) (elt offset i)))
                  movable-region))
      movable-region))
  (:set-movable-region-for-bin
    (&key (offset (list 0 0 0)))
    (let (cube)
        (progn
          (dolist (bin (list :a :b :c))
               (setq cube (gethash bin bin-cubes-))
               (sethash bin bin-movable-regions-
                        (send self :cube->movable-region cube :offset offset))))))
  (:set-movable-region-for-tote
    (arm &key (offset (list 0 0 0)))
    (let (cube)
      (setq cube (gethash arm tote-cubes-))
      (sethash arm tote-movable-regions- (send self :cube->movable-region cube :offset offset))))
  (:get-next-target-bin (arm &key (stamp (ros::time-now)))
    (when (eq (length orders-) 0)
      (ros::ros-error "[:get-next-target-bin] no work-order, try :get-work-orders first")
      (return-from :get-next-target-bin nil))
    (let (bin-contents target-bins postponed-bins msg)
      (setq msg (one-shot-subscribe "/json_saver/output/bin_contents"
                                    jsk_arc2017_common::ContentArray
                                    :timeout 10000
                                    :after-stamp stamp))
      (setq bin-contents
            (mapcar #'(lambda (content)
                        (cons (str2symbol (send content :bin))
                              (send content :items)))
                    (send msg :contents)))
      (ros::ros-debug "[:get-next-target-bin] get bin-contents ~A" bin-contents)
      (setq target-bins
            (remove-duplicates
              (remove nil
                      (mapcar
                        #'(lambda (order)
                            (let ((item (send order :item))
                                  (bin (str2symbol (send order :bin))))
                              (ros::ros-debug "[:get-next-target-bin] item ~A, bin ~A" item bin)
                              (when (find item (cdr (assoc bin bin-contents)) :test #'string=)
                                (unless
                                  (or (find item (gethash bin finished-objects-) :test #'string=)
                                      (find item (gethash bin postponed-objects-) :test #'string=))
                                  bin))))
                        orders-))))
      ;; 66% random target
      ;; 33% select unfinished-objects (never tried to recognize)
      (when (< (random 3) 2) (setq target-bins nil))
      (ros::ros-debug "[:get-next-target-bin] get target-bins ~A" target-bins)
      (when target-bins
        (return-from :get-next-target-bin
                     (elt target-bins (random (length target-bins)))))
      (setq postponed-bins
            (remove-duplicates
              (remove nil
                      (mapcar
                        #'(lambda (order)
                            (let ((item (send order :item))
                                  (bin (str2symbol (send order :bin))))
                              (ros::ros-debug "[:get-next-target-bin]   item ~A, bin ~A" item bin)
                              (when (find item (cdr (assoc bin bin-contents)) :test #'string=)
                                (unless
                                  (find item (gethash bin finished-objects-) :test #'string=)
                                  bin))))
                        orders-))))

      (ros::ros-debug "[:get-next-target-bin] get postponed bins ~A" postponed-bins)
      (when postponed-bins
        (return-from :get-next-target-bin
                     (elt postponed-bins (random (length postponed-bins)))))
      nil))
  (:get-work-orders
    (arm)
    (let (msg)
      (setq msg
        (one-shot-subscribe
          (format nil "/strategic_work_order/~a_hand" (arm2str arm))
          jsk_arc2017_common::WorkOrderArray))
      (setq orders- (send msg :orders))))
  (:get-next-work-order (arm bin)
    (when (eq (length orders-) 0)
      (ros::ros-error "[:get-next-work-order] There is no order")
      (return-from :get-next-work-order nil))
    ;; return never tried object
    (dolist (skipping-objects
              (list (append (gethash bin finished-objects-) (gethash bin postponed-objects-))
                    (gethash bin finished-objects-)))
      (dolist (order orders-)
        (unless (find (send order :item) skipping-objects :test #'string=)
          (return-from :get-next-work-order order))))
    nil)
  (:get-certain-work-order
    (arm bin)
    (when (eq (length orders-) 0)
      (ros::ros-error "[:get-certain-work-order] There is no order")
      (return-from :get-certain-work-order nil))
    (when (null bin) (return-from :get-certain-work-order (elt orders- 0)))
    (dotimes (i (length orders-))
      (when (string= (send (elt orders- i) :bin)
                     (symbol-string bin))
        (return-from :get-certain-work-order (elt orders- i)))))
  (:select-work-order-in-bin
    (arm bin &key (stamp (ros::time-now)) (timeout 10))
    (let ((box-topic (format nil "~a_hand_camera/cluster_indices_decomposer_label/boxes" (arm2str arm)))
          (candidates-topic (format nil "/~a_hand_camera/candidates_publisher/output/candidates" (arm2str arm)))
          box-msg box-objects box-labels bin-orders
          candidates-msg candidate-labels label-names
          unfinished-orders)
      (setq bin-orders
            (remove nil (mapcar
                          #'(lambda (order)
                              (when (string= (send order :bin) (symbol-string bin)) order))
                          orders-)))
      (setq label-names
            (ros::get-param (format nil "/~a_hand_camera/label_names"
                                    (arm2str arm))))
      (ros::subscribe box-topic jsk_recognition_msgs::BoundingBoxArray
                      #'(lambda (msg)
                          (let ((st (send msg :header :stamp)))
                            (when (> (send st :to-sec) (send stamp :to-sec))
                              (setq box-msg msg)))))
      (ros::subscribe candidates-topic jsk_recognition_msgs::LabelArray
                      #'(lambda (msg)
                          (let ((st (send msg :header :stamp)))
                            (when (> (send st :to-sec) (send stamp :to-sec))
                              (setq candidates-msg msg)))))
      (while (and (not (and box-msg candidates-msg))
                  (> (+ (send stamp :to-sec) timeout)
                     (send (ros::time-now) :to-sec)))
             (unix::usleep (* 50 1000))
             (ros::spin-once))
      (ros::unsubscribe box-topic)
      (ros::unsubscribe candidates-topic)

      (when (and box-msg candidates-msg)
        (setq box-labels
              (remove nil (mapcar
                            #'(lambda (box)
                                (let (volume)
                                  (setq volume (* (send box :dimensions :x)
                                                  (send box :dimensions :y)
                                                  (send box :dimensions :z)))
                                  (when (> volume 0.0)
                                    (send box :label))))
                            (send box-msg :boxes))))
        (setq candidate-labels
              (mapcar #'(lambda (label-msg) (send label-msg :id)) (send candidates-msg :labels)))
        (setq box-labels
              (remove nil (mapcar
                            #'(lambda (label)
                                (when (find label candidate-labels) label))
                            box-labels)))
        (unless box-labels (return-from :select-work-order-in-bin nil))
        (setq box-objects (mapcar #'(lambda (label) (elt label-names label)) box-labels))
        (ros::ros-debug "arm: ~A, box-objects: ~A" (arm2str arm) box-objects)
        (dolist (order bin-orders)
          (let ((object-name (send order :item)))
            (when (and (find object-name box-objects :test #'string=)
                       (not (find object-name (append (gethash bin finished-objects-)
                                                      (gethash bin postponed-objects-))
                                  :test #'string=)))
              (return-from :select-work-order-in-bin order))))
        (setq unfinished-orders
              (remove nil
                      (mapcar #'(lambda (order)
                                  (let ((object-name (send order :item)))
                                    (when (and (find object-name box-objects :test #'string=)
                                               (not (find object-name
                                                          (gethash bin finished-objects-)
                                                          :test #'string=)))
                                      order)))
                      bin-orders)))
        (when unfinished-orders
          (return-from :select-work-order-in-bin
                       (elt unfinished-orders (random (length unfinished-orders))))))
      nil))
  (:check-exist-in-order (obj)
    (let (index)
      (setq index
            (position obj
                      (mapcar #'(lambda (order) (send order :item)) orders-) :test #'string=))
      (if index
        (elt orders- index)
        nil)))
  (:check-bin-exist (bin) (if (gethash bin bin-cubes-) t nil))
  (:recognize-bin-boxes
    (&key (stamp (ros::time-now)))
    (let ((box-topic (format nil "transformable_bin_markers/output/boxes"))
          box-msg box-list bin-list)
      (setq box-msg (one-shot-subscribe box-topic
                                        jsk_recognition_msgs::BoundingBoxArray
                                        :timeout 10000
                                        :after-stamp stamp))
      (if box-msg
        (progn
          (ros::ros-info "[~a] [:recognize-bin-boxes] recognize bin boxes" (ros::get-name))
          (setq box-list (send box-msg :boxes))
          (setq bin-list (list :a :b :c))
          (dolist (bin bin-list)
            (sethash bin bin-cubes- (send self :bbox->cube (car box-list)))
            (setq box-list (cdr box-list))))
        (ros::ros-fatal "[:recognize-bin-boxes] cannot recognize bin boxes"))))
  (:recognize-cardboard-boxes
    (&key (stamp (ros::time-now)))
    (let ((box-topic (format nil "transformable_cardboard_markers/output/boxes"))
          box-msg box-list cardboard-list)
      (setq box-msg (one-shot-subscribe box-topic
                                        jsk_recognition_msgs::BoundingBoxArray
                                        :timeout 10000
                                        :after-stamp stamp))
      (if box-msg
        (progn
          (ros::ros-info "[~a] [:recognize-cardboard-boxes] recognize cardboard boxes" (ros::get-name))
          (setq box-list (send box-msg :boxes))
          (setq cardboard-list (list :a :b :c))
          (dolist (cardboard cardboard-list)
            (sethash cardboard cardboard-cubes- (send self :bbox->cube (car box-list)))
            (setq box-list (cdr box-list))))
        (ros::ros-fatal "[:recognize-cardboard-boxes] cannot recognize cardboard boxes"))))
  (:recognize-tote-boxes
    (&key (stamp (ros::time-now)))
    (let ((box-topic (format nil "transformable_tote_markers/output/boxes"))
                     box-msg box-list)
      (setq box-msg (one-shot-subscribe box-topic
                                        jsk_recognition_msgs::BoundingBoxArray
                                        :timeout 10000
                                        :after-stamp stamp))
      (if box-msg
        (progn
          (ros::ros-info "[~a] [:recognize-tote-boxes] recognize tote bbox" (ros::get-name))
          (setq box-list (send box-msg :boxes))
          (dolist (arm '(:larm :rarm))
            (sethash arm tote-cubes- (send self :bbox->cube (pop box-list)))))
        (ros::ros-fatal "[:recognize-tote-boxes] cannot recognize tote bbox"))))
  (:visualize-bins ()
    (ros::ros-warn ":visualize-bins was deprecated. Please use :visualize-boxes.")
    (send self :visualize-boxes :box-type :bin))
  (:visualize-boxes (&key (box-type :bin))
    "Visualize boxes
     Arguments
     ---------
     box-type: symbol
       :bin, :tote or :cardboard"
    (let (boxes box-color box-cubes)
      (cond
        ((eq box-type :bin) (setq box-cubes bin-cubes- box-color :blue))
        ((eq box-type :tote) (setq box-cubes tote-cubes- box-color :red))
        ((eq box-type :cardboard) (setq box-cubes cardboard-cubes- box-color :yellow))
        (t (ros::ros-error "Unsupported box-type ~a" box-type))
        )
      (dolist (box-inside (send box-cubes :list-values))
        (let ((box-outside (make-cube (+ (x-of-cube box-inside) 30)
                                      (+ (y-of-cube box-inside) 30)
                                      (z-of-cube box-inside)))
              (box-model))
          (send box-outside :newcoords (send box-inside :copy-worldcoords))
          (send box-outside :translate (float-vector 0 0 15) :world)
          (setq box-model (body- box-outside box-inside))
          (send box-model :set-color box-color 0.5)
          (pushback box-model boxes)))
      boxes))
  (:recognize-target-object
    (arm &key (stamp (ros::time-now)) (timeout 10))
    (let ((box-topic (format nil "~a_hand_camera/cluster_indices_decomposer_target/boxes" (arm2str arm)))
          box-msg
          (com-topic (format nil "~a_hand_camera/cluster_indices_decomposer_target/centroid_pose_array"
                        (arm2str arm)))
          com-msg obj-box obj-coords is-recognized)
      (when (ros::get-param "~data_collection" nil)
        (ros::ros-info-green "Requesting data collection for shelf-bin/tote.")
        (let ((res (ros::service-call (format nil "~a_hand_camera/data_collection_server/save_request" (arm2str arm))
                           (instance std_srvs::TriggerRequest :init))))
          (if (send res :success)
            (ros::ros-info-green "~a" (send res :message))
            (ros::ros-info-red "~a" (send res :message))
            )))
      (ros::subscribe box-topic jsk_recognition_msgs::BoundingBoxArray
                      #'(lambda (msg)
                          (let ((st (send msg :header :stamp)))
                            (when (> (send st :to-sec) (send stamp :to-sec))
                              (setq box-msg msg)))))
      (ros::subscribe com-topic geometry_msgs::PoseArray
                      #'(lambda (msg)
                          (let ((st (send msg :header :stamp)))
                            (when (> (send st :to-sec) (send stamp :to-sec))
                              (setq com-msg msg)))))
      (while (and (not (and box-msg com-msg))
                  (> (+ (send stamp :to-sec) timeout) (send (ros::time-now) :to-sec)))
             (unix::usleep (* 50 1000))
             (ros::spin-once))
      (ros::unsubscribe box-topic)
      (ros::unsubscribe com-topic)
      (cond
        ((and box-msg com-msg)
         (ros::ros-info "[:recognize-target-object] arm: ~a get cpi msg" arm)
         (setq obj-box (send box-msg :boxes))
         (setq obj-coords
               (mapcar #'(lambda (obj-pose)
                           (send (send self :get-robot-interface) :tf-pose->coords
                                 (send com-msg :header :frame_id) obj-pose))
                       (send com-msg :poses)))
         (if (and (> (length obj-box) 0) (> (length obj-coords) 0))
           (progn
             (sethash arm object-boxes- obj-box)
             (sethash arm object-coords- obj-coords)
             (setq is-recognized (send box-msg :header :stamp)))
           (progn
             (ros::ros-error "[:recognize-target-object] arm: ~a obj-box length ~a" arm (length obj-box))
             (ros::ros-error "[:recognize-target-object] arm: ~a obj-coords length ~a" arm (length obj-coords))
             (setq is-recognized nil))))
        (t
         (ros::ros-error "[:recognize-target-object] arm: ~a failed to get cpi msg" arm)
         (setq is-recognized nil)))
      is-recognized))
  (:get-largest-object-index
    (arm bin &key (stamp (ros::time-now)) (n-random nil))
    (let ((box-topic (format nil "~a_hand_camera/cluster_indices_decomposer_label/boxes" (arm2str arm)))
          box-msg box-labels label-names)
      (setq label-names
            (ros::get-param (format nil "/~a_hand_camera/label_names"
                                    (arm2str arm))))
      (setq box-msg (one-shot-subscribe box-topic
                                        jsk_recognition_msgs::BoundingBoxArray
                                        :timeout 10000
                                        :after-stamp stamp))
      (when box-msg
        (setq box-labels
              (remove nil (mapcar
                            #'(lambda (box)
                                (let (volume)
                                  (setq volume (* (send box :dimensions :x)
                                                  (send box :dimensions :y)
                                                  (send box :dimensions :z)))
                                  (when (> volume 0.0)
                                    (send box :label))))
                            (send box-msg :boxes))))
        (unless box-labels (return-from :get-largest-object-index nil))
        (dolist (skipping-objects
                  (list (append (gethash bin finished-objects-) (gethash bin postponed-objects-))
                        (gethash bin finished-objects-)))
          (let (label-indices target-indices target-index)
            (setq label-indices
                  (remove nil (mapcar
                                #'(lambda (label-index)
                                    (let ((obj-name (elt label-names label-index)))
                                      (unless (find obj-name skipping-objects :test #'string=)
                                        label-index)))
                                box-labels)))
            (when label-indices
              (if n-random
                (progn
                  (setq target-indices (subseq label-indices 0 n-random))
                  (setq target-index (elt target-indices (random (length target-indices)))))
                (setq target-index (car label-indices)))
              (return-from :get-largest-object-index target-index)))))
      nil))
  (:resolve-collision-between-fingers (arm)
    (send (send self :get-robot-interface) :angle-vector-raw
          (send (send self :get-robot) :slide-gripper arm 120 :relative nil)
          1000 (send (send self :get-robot-interface) :get-arm-controller arm) 0)
    (send (send self :get-robot-interface) :move-hand arm
          (send (send self :get-robot) :hand-grasp-pre-pose arm :cylindrical) 1000 :wait nil)
    (send (send self :get-robot-interface) :wait-interpolation)
    (unix::sleep 1)
    (send (send self :get-robot-interface) :angle-vector-raw
          (send (send self :get-robot) :slide-gripper arm 0 :relative nil)
          1000 (send (send self :get-robot-interface) :get-arm-controller arm) 0)
    (send (send self :get-robot-interface) :move-hand arm
          (send (send self :get-robot) :hand-grasp-pre-pose arm :opposed) 1000 :wait nil)
    (send (send self :get-robot-interface) :wait-interpolation))
  (:pick-object-in-bin
    (arm bin &rest args)
    (let (pick-result movable-region)
      (setq movable-region (gethash bin bin-movable-regions-))
      (unless movable-region
        (ros::ros-error "[:pick-object-in-bin] No movable region for bin. Call :set-movable-region-for-bin first.")
        (return-from :pick-object-in-bin nil))
      (setq pick-result (send* self :pick-object-with-movable-region arm movable-region args))
      pick-result))
  (:pick-object-in-tote
    (arm &rest args)
    (let (pick-result movable-region)
      (setq movable-region (gethash arm tote-movable-regions-))
      (unless movable-region
        (ros::ros-error "[:pick-object-in-tote] No movable region for tote. Call :set-movable-region-for-tote first.")
        (return-from :pick-object-in-tote nil))
      (setq pick-result (send* self :pick-object-with-movable-region arm movable-region args))
      pick-result))
  (:pick-object-with-movable-region
    (arm movable-region &key (n-trial 1) (n-trial-same-pos 1)
         (do-stop-grasp nil) (grasp-style :suction) (object-index 0))
    "Return value: :grasp-succeeded, :grasp-failed or :ik-failed"
    (send (send self :get-robot-interface) :stop-grasp arm :pinch)
    (send (send self :get-robot-interface) :calib-proximity-threshold arm)
    (let (pick-result avs obj-pos obj-cube pinch-yaw suction-yaw near-walls)
      ;; pick largest bounding box when object-index is 0
      (setq obj-pos
            (send self :get-object-position arm movable-region :object-index object-index))
      (setq obj-cube (send self :bbox->cube (elt (gethash arm object-boxes-) object-index)))
      (setq near-walls (send self :check-near-walls obj-pos movable-region :range 100))
      (if (< (length near-walls) 2)
        (cond
          ((eq (car near-walls) :rear) (setq pinch-yaw 0) (setq suction-yaw 0))
          ((eq (car near-walls) :front) (setq pinch-yaw 0) (setq suction-yaw pi))
          ((eq (car near-walls) :right) (setq pinch-yaw -pi/2) (setq suction-yaw pi/2))
          ((eq (car near-walls) :left) (setq pinch-yaw pi/2) (setq suction-yaw -pi/2))
          (t
            (if (> (x-of-cube obj-cube) (y-of-cube obj-cube))
              (setq pinch-yaw
                    (+ (if (eq arm :rarm) pi/2 -pi/2)
                       (caar (send (send obj-cube :worldcoords) :rpy-angle))))
              (setq pinch-yaw (caar (send (send obj-cube :worldcoords) :rpy-angle))))
            (while (> pinch-yaw pi/2)
                   (setq pinch-yaw (- pinch-yaw pi)))
            (while (< pinch-yaw -pi/2)
                   (setq pinch-yaw (+ pinch-yaw pi)))))
        (cond
          ((and (member :rear near-walls) (member :right near-walls))
           (setq pinch-yaw (/ pi 4)) (setq suction-yaw (/ pi 4)))
          ((and (member :rear near-walls) (member :left near-walls))
           (setq pinch-yaw (/ -pi 4)) (setq suction-yaw (/ -pi 4)))
          ((and (member :front near-walls) (member :right near-walls))
           (setq pinch-yaw (/ (* 3 pi) 4)) (setq suction-yaw (/ (* 3 pi) 4)))
          ((and (member :front near-walls) (member :left near-walls))
           (setq pinch-yaw (/ (* 3 -pi) 4)) (setq suction-yaw (/ (* 3 -pi) 4)))
          (t nil)))
      (ros::ros-info "[:pick-object-with-movable-region] arm:~a approach to the object" arm)
      (ros::ros-info "[:pick-object-with-movable-region] arm:~a obj-pos: ~a" arm obj-pos)
      (send (send self :get-robot-interface) :gripper-servo-on arm)
      ;; Setup arm for picking
      (let (next-avs target-poss)
        (send (send self :get-robot) :rotate-gripper arm 0 :relative nil)
        (send (send self :get-robot) :slide-gripper arm 0 :relative nil)
        (let ((target-pos (copy-object obj-pos)))
          ;; movable-region: ((min-of-x max-of-x) (min-of-y max-of-y) (min-of-z max-of-z))
          (setf (elt target-pos 2) (+ (cadr (elt movable-region 2)) 100))
          (pushback target-pos target-poss)  ;; location above of the target object in z axis
          ;; Go to a little inside of the bin/tote, 300 (30cm) is the z length of bins
          ;; and smaller than that of tote. TODO: Fix the hard coded offset.
          (setf (elt target-pos 2) (+ (car (elt movable-region 2)) 300))
          (pushback target-pos target-poss))
        (if (eq grasp-style :suction)
          ;; suction
          (progn
            (dolist (target-pos target-poss)
              (pushback
                (send (send self :get-robot) :rotate-wrist-ik arm
                      (make-coords :pos target-pos
                                   :rpy (float-vector (or suction-yaw 0) 0 0))
                      :move-palm-end nil
                      :rotation-axis (if (null suction-yaw) :z t))
                next-avs)
              )
            ;; Fold fingers to avoid collision to the bin walls
            (send (send self :get-robot-interface) :move-hand arm
                  (send (send self :get-robot) :hand-grasp-pre-pose arm :spherical) 1000
                  :wait nil)
            (send (send self :get-robot-interface) :move-hand arm
                  (send (send self :get-robot) :hand-grasp-pose arm :spherical :angle 90) 1000
                  :wait t))
          ;; pinch
          (progn
            (dolist (target-pos target-poss)
              (pushback
                (send (send self :get-robot) :rotate-wrist-ik arm
                      (make-coords :pos target-pos
                                  :rpy (float-vector pinch-yaw 0 0))
                      :move-palm-end t
                      :rotation-axis t)
                next-avs))
            (send (send self :get-robot-interface) :move-hand arm
                  (send (send self :get-robot) :hand-grasp-pose arm :cylindrical :angle 90) 1000)))
        (dolist (av next-avs)
          (unless av
            (ros::ros-error "[:pick-object-with-movable-region] arm:~a IK before trying grasp fails. Abort picking" arm)
            (return-from :pick-object-with-movable-region :ik-failed)))
        ;; Move whole arm to location above of the target pose
        (send (send self :get-robot-interface) :angle-vector-sequence-raw next-avs
              :fast (send (send self :get-robot-interface) :get-arm-controller arm :head t)
              0 :scale 10.0)
        (send (send self :get-robot-interface) :wait-interpolation))
      ;; Now hand is inside of the bin, so we can use cylindrical
      (send (send self :get-robot-interface) :move-hand arm
            (send (send self :get-robot) :hand-grasp-pre-pose arm :cylindrical) 1000)
      ;; Picking trials in the bin
      (dotimes (i n-trial)
        (dotimes (j n-trial-same-pos)
          (when (or (null pick-result) (eq pick-result :grasp-failed))
            (setq pick-result
                  (send self :try-to-pick-object arm obj-pos
                        :offset (float-vector 0 0 (- (* i -30) 30))
                        :grasp-style grasp-style
                        :suction-yaw suction-yaw
                        :pinch-yaw pinch-yaw)))))
      (when do-stop-grasp
        (unless (eq pick-result :grasp-succeeded)
          (send (send self :get-robot-interface) :stop-grasp arm)
          (if (eq grasp-style :pinch)
            (send (send self :get-robot-interface) :stop-grasp arm :pinch))))
      (if (eq grasp-style :suction)
        (progn
          ;; Move underactuated fingers to initial pose
          (send (send self :get-robot-interface) :move-hand arm
                (send (send self :get-robot) :hand-grasp-pre-pose arm :opposed) 1000 :wait nil)))
      (send (send self :get-robot-interface) :gripper-servo-on arm)
      pick-result))
  (:get-object-position
    (arm movable-region &key (object-index 0))
    (let (obj-box obj-coords obj-pos)
      (setq obj-box (elt (gethash arm object-boxes-) object-index))
      (setq obj-coords (elt (gethash arm object-coords-) object-index))
      (setq obj-pos (send obj-coords :worldpos))
      (dolist (i (list 0 1))
        (cond ((> (elt (elt movable-region i) 0) (aref obj-pos i))
               (ros::ros-error "[:get-object-position] object is out of movable region. ~a > ~a < ~a"
                               (elt (elt movable-region i) 1)
                               (elt (send obj-coords :pos) i)
                               (elt (elt movable-region i) 0))
               (setf (aref obj-pos i) (elt (elt movable-region i) 0)))
              ((> (aref obj-pos i) (elt (elt movable-region i) 1))
               (ros::ros-error "[:get-object-position] object is out of movable region. ~a < ~a > ~a"
                               (elt (elt movable-region i) 1)
                               (elt (send obj-coords :pos) i)
                               (elt (elt movable-region i) 0))
               (setf (aref obj-pos i) (elt (elt movable-region i) 1)))
              (t nil)))
      obj-pos))
  (:check-near-walls
    (obj-pos movable-region &key (range 100))
    (let (near-walls)
      (when (< (- (aref obj-pos 0) (elt (elt movable-region 0) 0)) range)
        (pushback :rear near-walls)
        (ros::ros-info-cyan "[:check-near-walls] Object is near rear wall"))
      (when (< (- (elt (elt movable-region 0) 1) (aref obj-pos 0)) range)
        (pushback :front near-walls)
        (ros::ros-info-cyan "[:check-near-walls] Object is near front wall"))
      (when (< (- (aref obj-pos 1) (elt (elt movable-region 1) 0)) range)
        (pushback :right near-walls)
        (ros::ros-info-cyan "[:check-near-walls] Object is near right wall"))
      (when (< (- (elt (elt movable-region 1) 1) (aref obj-pos 1)) range)
        (pushback :left near-walls)
        (ros::ros-info-cyan "[:check-near-walls] Object is near left wall"))
      near-walls))
  (:try-to-pick-object
    (arm obj-pos &key (offset #f(0 0 0)) (grasp-style :suction) suction-yaw
         (pinch-yaw (if (eq grasp-style :pinch) 0)))
      (if (eq grasp-style :suction)
        (send self :try-to-suction-object arm obj-pos suction-yaw :offset offset)
        (send self :try-to-pinch-object arm obj-pos pinch-yaw :offset offset)))
  (:try-to-suction-object
    (arm obj-pos suction-yaw &key (offset #f(0 0 0)))
    (send self :try-to-suction-object-with-gripper-v6 arm obj-pos suction-yaw :offset offset))
  (:try-to-suction-object-with-gripper-v4
    (arm obj-pos &key (offset #f(0 0 0)))
    "Return value: :grasp-succeeded or :grasp-failed"
    (let (graspingp)
      (send (send self :get-robot-interface) :angle-vector-raw
            (send (send self :get-robot) arm :inverse-kinematics
                  (make-coords :pos (v+ obj-pos offset)
                               :rpy #f(0 0 0))
                  :use-gripper t
                  :rotation-axis :z)
            3000 (send (send self :get-robot-interface) :get-arm-controller arm) 0)
      (send (send self :get-robot-interface) :wait-interpolation)
      ;; start the vacuum gripper after approaching to the object
      (ros::ros-info "[:try-to-suction-object-with-gripper-v4] arm:~a start vacuum gripper" arm)
      (send (send self :get-robot-interface) :start-grasp arm)
      (unix::sleep 1)
      (send (send self :get-robot-interface) :angle-vector-raw
            (send (send self :get-robot) arm :inverse-kinematics
                  (make-coords :pos obj-pos
                               :rpy #f(0 0 0))
                  :use-gripper t
                  :rotation-axis :z)
            3000 (send (send self :get-robot-interface) :get-arm-controller arm) 0)
      (send (send self :get-robot-interface) :wait-interpolation-until-grasp arm)
      (setq graspingp (send (send self :get-robot-interface) :graspingp arm :suction))
      (ros::ros-info "[:try-to-suction-object-with-gripper-v4] arm:~a graspingp: ~a" arm graspingp)
      (unless graspingp
        (ros::ros-info "[:try-to-suction-object-with-gripper-v4] arm:~a again approach to the object" arm)
        (let ((temp-av (send (send self :get-robot) :angle-vector)))
          ;; only if robot can solve IK
          (if (send (send self :get-robot) arm :move-end-pos #f(0 0 -50) :local)
            (send (send self :get-robot-interface) :angle-vector-raw
                  (send (send self :get-robot) :angle-vector)
                  3000 (send (send self :get-robot-interface) :get-arm-controller arm) 0))
          (send (send self :get-robot-interface) :wait-interpolation-until-grasp arm)
          ;; revert baxter
          (send (send self :get-robot-interface) :angle-vector-raw
                (send (send self :get-robot) :angle-vector temp-av)
                3000 (send (send self :get-robot-interface) :get-arm-controller arm) 0)
          (send (send self :get-robot-interface) :wait-interpolation-until-grasp arm)))
      ;; lift object
      (ros::ros-info "[:try-to-suction-object-with-gripper-v4] arm:~a lift the object" arm)
      (send (send self :get-robot-interface) :gripper-servo-off arm)
      (send (send self :get-robot-interface) :angle-vector-raw
            (send (send self :get-robot) arm :move-end-pos #f(0 0 200) :world)
            3000 (send (send self :get-robot-interface) :get-arm-controller arm) 0)
      (send (send self :get-robot-interface) :wait-interpolation)
      (unix::sleep 1)  ;; wait for arm to follow
      (setq graspingp (send (send self :get-robot-interface) :graspingp arm :suction))
      (ros::ros-info "[:try-to-suction-object-with-gripper-v4] arm:~a graspingp: ~a" arm graspingp)
      (if graspingp :grasp-succeeded :grasp-failed)))
  (:try-to-suction-object-with-gripper-v6
    (arm obj-pos suction-yaw &key (offset #f(0 0 0)))
    "Return value: :grasp-succeeded or :grasp-failed"
    (let (graspingp av
          (coords-before-approach (send (send self :get-robot) arm :end-coords :copy-worldcoords)))
      (send coords-before-approach :locate (v+ obj-pos #f(0 0 300)) :world)
      (ros::ros-info "[:try-to-suction-object-with-gripper-v6] arm:~a suction-yaw: ~a" arm suction-yaw)
      ;; start the vacuum gripper before approaching to the object
      (ros::ros-info "[:try-to-suction-object-with-gripper-v6] arm:~a start vacuum gripper" arm)
      (send (send self :get-robot-interface) :start-grasp arm)
      ;; suction: prismatic-based approach
      (send (send self :get-robot) :slide-gripper arm 120 :relative nil)
      (send (send self :get-robot) arm :inverse-kinematics
            (make-coords :pos (v+ obj-pos offset)
                         :rpy (float-vector (or suction-yaw 0) 0 0))
            :use-gripper t
            :rotation-axis (if suction-yaw t :z))
      (let ((prismatic-angle (send (send self :get-robot) arm :gripper-x :joint-angle)))
        (send (send self :get-robot) :slide-gripper arm 0 :relative nil)
        (send (send self :get-robot-interface) :angle-vector-raw
              (send (send self :get-robot) :angle-vector)
              3000 (send (send self :get-robot-interface) :get-arm-controller arm) 0)
        (send (send self :get-robot-interface) :wait-interpolation)  ;; move down only the hand palm

        (send (send self :get-robot) :slide-gripper arm prismatic-angle :relative nil)
        (send (send self :get-robot-interface) :angle-vector-raw
              (send (send self :get-robot) :angle-vector)
              3000 (send (send self :get-robot-interface) :get-arm-controller arm) 0)
        ;; FIXME: :wait-interpolation-until using :prismatic-loaded sometimes ends too fast,
        ;; so currently we only check :grasp (suction pressure).
        (send (send self :get-robot-interface) :wait-interpolation-until arm :grasp)

        ;; maximum angle of prismatic joint
        (send (send self :get-robot) :slide-gripper arm 120 :relative nil)
        (send (send self :get-robot-interface) :angle-vector-raw
              (send (send self :get-robot) :angle-vector)
              3000 (send (send self :get-robot-interface) :get-arm-controller arm) 0)
        (send (send self :get-robot-interface) :wait-interpolation-until arm
              :grasp :prismatic-loaded)
        )
      (setq graspingp (send (send self :get-robot-interface) :graspingp arm :suction))
      (ros::ros-info "[:try-to-suction-object-with-gripper-v6] arm:~a graspingp: ~a" arm graspingp)
      (unless graspingp
        (ros::ros-info "[:try-to-suction-object-with-gripper-v6] arm:~a again approach to the object" arm)
        (let ((temp-av (send (send self :get-robot) :angle-vector)))
          ;; only if robot can solve IK
          (if (send (send self :get-robot) arm :move-end-pos #f(0 0 -100) :local)
            (send (send self :get-robot-interface) :angle-vector-raw
                  (send (send self :get-robot) :angle-vector)
                  3000 (send (send self :get-robot-interface) :get-arm-controller arm) 0))
          (send (send self :get-robot-interface) :wait-interpolation-until arm
                :grasp :prismatic-loaded)
          ;; revert baxter
          (send (send self :get-robot-interface) :angle-vector-raw
                (send (send self :get-robot) :angle-vector temp-av)
                3000 (send (send self :get-robot-interface) :get-arm-controller arm) 0)
          (send (send self :get-robot-interface) :wait-interpolation-until arm
                :grasp :prismatic-loaded)))
      ;; Open fingers in bin
      (send (send self :get-robot-interface) :move-hand arm
            (send (send self :get-robot) :hand-grasp-pre-pose arm :spherical) 1000 :wait nil)
      (send (send self :get-robot) :angle-vector
            (send (send self :get-robot-interface) :state :potentio-vector :wait-until-update t))
      (send (send self :get-robot-interface) :angle-vector-raw
            (send (send self :get-robot) :rotate-gripper arm 30 :relative nil)
            1000 (send (send self :get-robot-interface) :get-arm-controller arm) 0)
      (send (send self :get-robot-interface) :wait-interpolation)
      ;; suction: prismatic-based approach
      (send (send self :get-robot) :slide-gripper arm 0 :relative nil)
      (send (send self :get-robot) :rotate-gripper arm 0 :relative nil)
      (send (send self :get-robot-interface) :angle-vector-raw
            (send (send self :get-robot) :angle-vector)
            3000 (send (send self :get-robot-interface) :get-arm-controller arm) 0)
      ;; suction: prismatic-based approach
      ;; lift object
      (ros::ros-info "[:try-to-suction-object-with-gripper-v6] arm:~a lift the object" arm)
      (send (send self :get-robot-interface) :gripper-servo-off arm)
      (let ((tc (send (send self :get-robot) arm :end-coords :copy-worldcoords)))
        ;; overwrite only world-z
        (setf (aref (send tc :worldpos) 2) (elt (send coords-before-approach :worldpos) 2))
        (send (send self :get-robot) arm :inverse-kinematics tc :rotation-axis :z)
        )
      (send (send self :get-robot-interface) :angle-vector-raw
            (send (send self :get-robot) :angle-vector)
            3000 (send (send self :get-robot-interface) :get-arm-controller arm) 0)
      (send (send self :get-robot-interface) :wait-interpolation)
      (setq graspingp (send (send self :get-robot-interface) :graspingp arm :suction))
      (ros::ros-info "[:try-to-suction-object-with-gripper-v6] arm:~a graspingp: ~a" arm graspingp)
      (if graspingp :grasp-succeeded :grasp-failed)))
  (:try-to-pinch-object
    (arm obj-pos pinch-yaw &key (offset #f(0 0 0)))
    "Return value: :grasp-succeeded, :grasp-failed or :ik-failed"
    (let (av graspingp pre-coords)
      (ros::ros-info "[:try-to-pinch-object] arm:~a pinch-yaw: ~a" arm pinch-yaw)
      ;; Initialize finger
      (send (send self :get-robot-interface) :move-hand arm
            (send (send self :get-robot) :hand-grasp-pose arm :cylindrical :angle 40) 1000)
      ;; start the vacuum gripper before approaching to the object
      (ros::ros-info "[:try-to-pinch-object] arm:~a start vacuum gripper" arm)
      (send (send self :get-robot-interface) :start-grasp arm)
      (setq av
            (send (send self :get-robot) arm :inverse-kinematics
                  (make-coords :pos (v+ obj-pos offset)
                               :rpy (float-vector pinch-yaw 0 0))
                  :move-palm-end t
                  :rotation-axis t))
      (if (null av)
        (setq av
              (send (send self :get-robot) arm :inverse-kinematics
                    (make-coords :pos (v+ obj-pos offset)
                                 :rpy (float-vector pinch-yaw 0 0))
                    :move-palm-end t
                    :rotation-axis :z)))
      (unless av
        (ros::ros-error "[:try-to-pinch-object] arm:~a IK to object fails. Abort picking" arm)
        (return-from :try-to-pinch-object :ik-failed))
      (send (send self :get-robot-interface) :update-robot-state :wait-until-update t)
      (unless (send (send self :get-robot-interface) :angle-vector-raw av
                    3000 (send (send self :get-robot-interface) :get-arm-controller arm)
                    0 :end-coords-interpolation t)
        (send (send self :get-robot-interface) :angle-vector-raw av 3000
              (send (send self :get-robot-interface) :get-arm-controller arm) 0))
      ;; Wait until grasp or finger touch
      (send (send self :get-robot-interface) :wait-interpolation-until arm
            :grasp :finger-flexion :finger-loaded :prismatic-loaded :finger-proximity)
      (setq av
            (send (send self :get-robot) arm :inverse-kinematics
                  (make-coords :pos obj-pos
                               :rpy (float-vector pinch-yaw 0 0))
                  :move-palm-end t
                  :rotation-axis t))
      (if (null av)
        (setq av
              (send (send self :get-robot) arm :inverse-kinematics
                    (make-coords :pos obj-pos
                                 :rpy (float-vector pinch-yaw 0 0))
                    :move-palm-end t
                    :rotation-axis :z)))
      (unless av
        (ros::ros-error "[:try-to-pinch-object] arm:~a IK to object fails. Abort picking" arm)
        (return-from :try-to-pinch-object :ik-failed))
      (send (send self :get-robot-interface) :angle-vector-raw av 3000
            (send (send self :get-robot-interface) :get-arm-controller arm) 0)
      (send (send self :get-robot-interface) :wait-interpolation-until arm
            :grasp :finger-flexion :finger-loaded :prismatic-loaded :finger-proximity)
      (send (send self :get-robot) :angle-vector
            (send (send self :get-robot-interface) :state :potentio-vector
                  :wait-until-update t))
      (send (send self :get-robot) arm :move-end-pos #f(0 0 -20) :world)
      (send (send self :get-robot-interface) :angle-vector-raw
            (send (send self :get-robot) :slide-gripper arm 120 :relative nil)
            3000 (send (send self :get-robot-interface) :get-arm-controller arm) 0)
      (send (send self :get-robot-interface) :wait-interpolation-until arm :grasp :prismatic-loaded)
      (send (send self :get-robot) :angle-vector
            (send (send self :get-robot-interface) :state :potentio-vector :wait-until-update t))
      (send (send self :get-robot-interface) :stop-grasp arm)
      (send (send self :get-robot-interface) :start-grasp arm :pinch)
      (send (send self :get-robot-interface) :start-grasp arm)
      (send (send self :get-robot) :angle-vector
            (send (send self :get-robot-interface) :state :potentio-vector :wait-until-update t))
      (setq graspingp (send (send self :get-robot-interface) :graspingp arm :pinch))
      (ros::ros-info "[:try-to-pinch-object] arm:~a graspingp: ~a" arm graspingp)
      ;; lift object
      (ros::ros-info "[:try-to-pinch-object] arm:~a lift the object" arm)
      (setq pre-coords (send (send (send self :get-robot) arm :end-coords) :copy-worldcoords))
      (send pre-coords :translate #f(0 0 200) :world)
      (setq av (send (send self :get-robot) arm :inverse-kinematics pre-coords :rotation-axis t))
      (if (null av)
        (setq av
              (send (send self :get-robot) arm :inverse-kinematics pre-coords :rotation-axis :z)))
      (send (send self :get-robot-interface) :angle-vector-raw av 3000
            (send (send self :get-robot-interface) :get-arm-controller arm :gripper nil) 0)
      (send (send self :get-robot-interface) :wait-interpolation)
      ;; slide gripper to grasp tightly
      (send (send self :get-robot) :rotate-gripper arm -90 :relative nil)
      (send (send self :get-robot) :slide-gripper arm 0 :relative nil)
      (send (send self :get-robot-interface) :angle-vector-raw
            (send (send self :get-robot) :angle-vector)
            :fast (send (send self :get-robot-interface) :get-arm-controller arm) 0 :scale 3.0)
      (send (send self :get-robot-interface) :wait-interpolation)
      (send (send self :get-robot) :angle-vector
            (send (send self :get-robot-interface) :state :potentio-vector :wait-until-update t))
      (setq graspingp (send (send self :get-robot-interface) :graspingp arm :pinch))
      (ros::ros-info "[:try-to-pinch-object] arm:~a graspingp: ~a" arm graspingp)
      (if graspingp :grasp-succeeded :grasp-failed)))
  (:ik->bin-center
    (arm bin &key (offset #f(0 0 0)) (rpy #f(0 0 0))
         (rotation-axis t) (use-gripper nil) (move-palm-end nil))
    (let (bin-cube bin-coords)
      (setq bin-cube (gethash bin bin-cubes-))
      (setq bin-coords (send bin-cube :copy-worldcoords))
      (send bin-coords :translate
            (float-vector 0.0 0.0 (/ (z-of-cube bin-cube) 2.0))
            :local)
      (send bin-coords :translate offset :world)
      (send bin-coords :rotate (aref rpy 0) :z)
      (send bin-coords :rotate (aref rpy 1) :y)
      (send bin-coords :rotate (aref rpy 2) :x)
      (send (send self :get-robot) arm :inverse-kinematics bin-coords
            :rotation-axis rotation-axis
            :use-gripper use-gripper
            :move-palm-end move-palm-end)))
  (:ik->cardboard-center
    (arm cardboard &key (offset #f(0 0 0)) (rpy #f(0 0 0))
         (rotation-axis t) (use-gripper nil) (move-palm-end nil))
    (let (cardboard-cube cardboard-coords)
      (setq cardboard-cube (gethash cardboard cardboard-cubes-))
      (setq cardboard-coords (send cardboard-cube :copy-worldcoords))
      (send cardboard-coords :translate
            (float-vector 0.0 0.0 (/ (z-of-cube cardboard-cube) 2.0))
            :local)
      (send cardboard-coords :translate offset :world)
      (send cardboard-coords :rotate (aref rpy 0) :z)
      (send cardboard-coords :rotate (aref rpy 1) :y)
      (send cardboard-coords :rotate (aref rpy 2) :x)

      ;; use prismatic joint as much as possible
      (let ((jnt (send (send self :get-robot) :joint (format nil "~a_gripper_prismatic_joint" (arm2str arm)))))
        (send jnt :joint-angle (send jnt :max-angle))
        )
      (send (send self :get-robot) arm :inverse-kinematics cardboard-coords
            :rotation-axis nil :use-gripper nil)
      (send (send self :get-robot) arm :inverse-kinematics cardboard-coords
            :rotation-axis rotation-axis
            :use-gripper use-gripper
            :move-palm-end move-palm-end)))
  (:ik->tote-center
    (arm &key (offset #f(0 0 0)) (rpy #f(0 0 0))
         (rotation-axis t) (use-gripper nil) (move-palm-end nil))
    (let (tote-cube tote-coords)
      (setq tote-cube (gethash arm tote-cubes-))
      (setq tote-coords (send tote-cube :copy-worldcoords))
      (send tote-coords :translate
            (float-vector 0.0 0.0 (/ (z-of-cube tote-cube) 2.0))
            :local)
      (send tote-coords :translate offset :world)
      (send tote-coords :rotate (aref rpy 0) :z)
      (send tote-coords :rotate (aref rpy 1) :y)
      (send tote-coords :rotate (aref rpy 2) :x)
      (send (send self :get-robot) arm :inverse-kinematics tote-coords
            :rotation-axis rotation-axis
            :use-gripper use-gripper
            :move-palm-end move-palm-end)))
  (:move-arm-body->bin-overlook-pose
    (arm bin &key (gripper-angle 90))
    (let (avs offset rpy
              (offset-x (if (eq arm :larm) 30 -50))
              (offset-y (if (eq arm :larm) 280 -250)))
      (setq avs
            (list (send (send self :get-robot) :avoid-shelf-pose arm (if (eq arm :larm) :d :f))))
      (send (send self :get-robot) :reset-pose arm)
      (send (send self :get-robot) :rotate-gripper arm gripper-angle :relative nil)
      (setq offset (float-vector offset-x offset-y 250))
      (setq rpy
            (float-vector 0 pi/2 (if (eq arm :larm) pi/2 -pi/2)))
      (send self :ik->bin-center arm bin
            :offset offset :rpy rpy :use-gripper nil)
      (send (send self :get-robot-interface) :angle-vector-raw
            (send (send self :get-robot) :angle-vector) 3000
            (send (send self :get-robot-interface) :get-arm-controller arm) 0)))
  (:move-arm-body->tote-overlook-pose
    (arm &key (gripper-angle 90))
    (let (avs offset rpy
              (offset-x (if (eq arm :larm) 0 -20))
              (offset-y (if (eq arm :larm) 390 -320)))
      (setq avs
            (list (send (send self :get-robot) :avoid-shelf-pose arm (if (eq arm :larm) :d :f))))
      (send (send self :get-robot) :reset-pose arm)
      (send (send self :get-robot) :rotate-gripper arm gripper-angle :relative nil)
      (setq offset (float-vector offset-x offset-y 250))
      (setq rpy (float-vector 0 pi/2 (if (eq arm :larm) pi/2 -pi/2)))
      (pushback
        (send self :ik->tote-center arm
              :offset offset :rpy rpy :use-gripper nil)
        avs)
      (send (send self :get-robot-interface) :angle-vector-raw
            (send (send self :get-robot) :angle-vector) 3000
            (send (send self :get-robot-interface) :get-arm-controller arm) 0)))
  (:wait-for-user-input-to-start (arm)
    (let (can-start)
      (ros::ros-info "[:wait-for-user-input-to-start] wait for user input to start: ~a" arm)
      (ros::wait-for-service "/rviz/yes_no_button")
      (while
        (not can-start)
        (setq can-start (send (ros::service-call
                                "/rviz/yes_no_button" (instance jsk_gui_msgs::YesNoRequest)) :yes)))
      (ros::ros-info "[:wait-for-user-input-to-start] received user input: ~a" arm)))
  (:set-object-segmentation-candidates (arm candidates)
    (let ((req (instance jsk_recognition_msgs::SetLabelsRequest :init)))
      (send req :labels candidates)
      (ros::service-call
        (format nil "/~a_hand_camera/apply_context_to_label_proba/update_candidates"
                (arm2str arm))
        req)))
  (:set-target-location (arm location)
    (if (consp location)
      (setq location (format nil "~a_~a" (symbol2str (car location)) (symbol-string (cdr location))))
      (setq location (symbol2str location)))
    (ros::set-dynparam
      (format nil "/~a_hand_camera/candidates_publisher" (arm2str arm))
      (cons "target_location" location)))
  (:set-arm-state-param (arm state)
    (ros::set-param (format nil "~a_hand/state" (arm2str arm)) (symbol2str state)))
  (:get-bin-contents (bin)
    (ros::get-param
      (format nil "/bin_contents/~A" (symbol-string bin))))
  (:add-workspace-scene
    ()
    (let ((base-name (send (send (send self :get-robot) :base_lk) :name))
          (dimensions (list '(2500 10 2000) '(2500 10 2000)))
          (positions (list (float-vector 500 1200 0) (float-vector 500 -1200 0)))
          (names (list "workspace_left" "workspace_right")))
      (dotimes (i (length positions))
        (let ((cube (apply #'make-cube (elt dimensions i)))
              (pos (elt positions i))
              (name (elt names i)))
          (send cube :locate pos :world)
          (send *co* :add-object cube :frame-id base-name
                :relative-pose (send cube :copy-worldcoords)
                :object-id name)
          )
        )
      ))
  (:add-bin-scene (bin)
    (let ((cube (gethash bin bin-cubes-))
          (base-name (send (send (send self :get-robot) :base_lk) :name)))
      (send *co* :add-object cube :frame-id base-name
            :relative-pose (send cube :copy-worldcoords)
            :object-id (format nil "bin_~A" (symbol-string bin)))))
  (:delete-bin-scene (bin)
    (send *co* :delete-object (gethash bin bin-cubes-)))
  (:add-shelf-scene ()
    (dolist (bin (list :a :b :c))
      (send self :add-bin-scene bin)))
  (:delete-shelf-scene ()
    (dolist (bin (list :a :b :c))
      (send self :delete-bin-scene bin)))
  (:add-cardboard-scene (cardboard)
    (let (cardboard-cube (base-name (send (send (send self :get-robot) :base_lk) :name)))
      (setq cardboard-cube (gethash cardboard cardboard-cubes-))
      (send *co* :add-object cardboard-cube :frame-id base-name
            :relative-pose (send cardboard-cube :copy-worldcoords)
            :object-id (format nil "cardboard_~A" (symbol-string cardboard)))))
  (:delete-cardboard-scene (cardboard)
    (let (cardboard-cube)
      (setq cardboard-cube (gethash cardboard cardboard-cubes-))
      (send *co* :delete-object cardboard-cube)))
  (:add-cardboad-rack-leg-scene (side)
    (let* ((cardboard-c-cube (gethash :c cardboard-cubes-))
           (cardboard-c-coords (send cardboard-c-cube :copy-worldcoords))
           (base-name (send (send (send self :get-robot) :base_lk) :name))
           rack-leg-cube offset)
      (setq offset
            (float-vector
              (elt (send cardboard-c-cube :worldpos) 0)
              (* (if (eq side :right) -1 1) (/ (y-of-cube cardboard-c-cube) 2.0))
              (elt (send cardboard-c-coords :worldpos) 2)))
      (setq rack-leg-cube (make-cube 300 20 1200))
      (send rack-leg-cube :translate offset :world)
      (send *co* :add-object rack-leg-cube :frame-id base-name
            :relative-pose (send rack-leg-cube :copy-worldcoords)
            :object-id (format nil "rack_leg_~a" (symbol2str side)))
      (sethash side rack-leg-cubes- rack-leg-cube)))
  (:delete-cardboard-rack-leg-scene (side)
    (send *co* :delete-object (gethash side rack-leg-cubes-)))
  (:add-cardboard-rack-scene ()
    (dolist (cardboard (list :a :b :c))
      (send self :add-cardboard-scene cardboard))
    (dolist (side (list :left :right))
      (send self :add-cardboad-rack-leg-scene side)))
  (:delete-cardboard-rack-scene ()
    (dolist (cardboard (list :a :b :c))
      (send self :delete-cardboard-scene cardboard))
    (dolist (side (list :left :right))
      (send self :delete-cardboad-rack-leg-scene side)))
  (:add-tote-scene (arm)
    (dolist (tmp-arm (if (eq arm :arms) (list :rarm :larm) (list arm)))
      (let ((cube (gethash tmp-arm tote-cubes-))
            (base-name (send (send (send self :get-robot) :base_lk) :name)))
        (send *co* :add-object cube :frame-id base-name
              :relative-pose (send cube :copy-worldcoords)
              :object-id (format nil "~A_tote" (arm2str tmp-arm))))))
  (:delete-tote-scene (arm)
    (dolist (tmp-arm (if (eq arm :arms) (list :rarm :larm) (list arm)))
      (send *co* :delete-object (gethash tmp-arm tote-cubes-))))
  (:add-object-in-hand-scene (arm target-obj)
    ;; add object as attached collision object
    (let ((end-coords-name (format nil "~a_gripper_pad_with_base" (arm2str arm))) obj-cube)
      ;; attach largest bounding box
      (setq obj-cube (send self :bbox->cube (elt (gethash arm object-boxes-) 0)))
      (send *co* :add-attached-object obj-cube end-coords-name
            :frame-id end-coords-name
            :object-id target-obj
            :relative-pose (make-coords :pos (float-vector 0 0 (/ (z-of-cube obj-cube) 2)))
            :touch-links (list
                           (format nil "~a_gripper_base" (arm2str arm))
                           (format nil "~a_gripper_tube" (arm2str arm))
                           (format nil "~a_gripper_pad_with_base" (arm2str arm))
                           (format nil "~a_gripper_l_finger_base" (arm2str arm))
                           (format nil "~a_gripper_r_finger_base" (arm2str arm))
                           (format nil "~a_gripper_l_finger" (arm2str arm))
                           (format nil "~a_gripper_r_finger" (arm2str arm))
                           (format nil "~a_gripper_palm_endpoint" (arm2str arm))))
      (sethash arm attached-objects- obj-cube)))
  (:delete-object-in-hand-scene (arm)
    ;; delete object as attached collision object
    (let (obj-cube)
      (setq obj-cube (gethash arm attached-objects-))
      (send *co* :delete-attached-object obj-cube)
      (send *co* :delete-object obj-cube)))
  (:wipe-all-scene ()
    (send *co* :wipe-all))
  (:spin-off-by-wrist
    (arm &key (times 10))
    (send (send self :get-robot-interface) :angle-vector-sequence-raw
          (send (send self :get-robot) :spin-off-by-wrist arm :times times)
          :fast (send (send self :get-robot-interface) :get-arm-controller arm) 0))
  (:send-av
    (&optional (tm 3000) (ctype nil))
    (send (send self :get-robot-interface) :angle-vector
          (send (send self :get-robot) :angle-vector) tm ctype))
  (:check-can-start (arm start-state wait-state)
    (let ((service-name
           (format nil "/state_server/~a_hand/check_can_start" (arm2str arm)))
          req can-start)
      (ros::wait-for-service service-name)
      (setq req (instance jsk_arc2017_baxter::CheckCanStartRequest :init))
      (send req :start_state (symbol2str start-state))
      (send req :wait_state (symbol2str wait-state))
      (setq can-start
            (send (ros::service-call service-name req) :can_start))
      can-start))
  (:update-state (arm state)
    (let ((service-name
           (format nil "/state_server/~a_hand/update_state" (arm2str arm)))
          req updated)
      (ros::wait-for-service service-name)
      (setq req (instance jsk_arc2017_baxter::UpdateStateRequest :init))
      (send req :state (symbol2str state))
      (setq updated (send (ros::service-call service-name req) :updated))
      updated))
  (:get-state (arm)
    (let ((service-name
           (format nil "/state_server/~a_hand/get_state" (arm2str arm)))
          req state)
      (ros::wait-for-service service-name)
      (setq req (instance jsk_arc2017_baxter::GetStateRequest :init))
      (setq state (send (ros::service-call service-name req) :state))
      state))
  (:update-json (object &key src dst)
    (let ((service-name (format nil "/json_saver/update_json"))
          (req (instance jsk_arc2017_common::UpdateJSONRequest :init)))
      (ros::wait-for-service service-name)
      (if (consp src)
        (setq src (format nil "~a_~a" (symbol2str (car src)) (symbol-string (cdr src))))
        (setq src (symbol2str src)))
      (if (consp dst)
        (setq dst (format nil "~a_~a" (symbol2str (car dst)) (symbol-string (cdr dst))))
        (setq dst (symbol2str dst)))
      (send req :src src)
      (send req :dst dst)
      (send req :item object)
      (send (ros::service-call service-name req) :updated)))
  (:save-json ()
    (let ((service-name (format nil "/json_saver/save_json"))
          (req (instance std_srvs::TriggerRequest :init)))
      (ros::wait-for-service service-name)
      (send (ros::service-call service-name req) :success)))
  (:get-scale-candidates (arm action &key (stamp (ros::time-now)))
    (let ((candidates-topic
            (format nil "~a_hand_camera/weight_candidates_refiner/output/candidates/~a"
                    (arm2str arm) (symbol2str action)))
          (changed-topic
            (format nil "~a_hand_camera/weight_candidates_refiner/output/changed_from_reset"
                    (arm2str arm)))
          candidates-msg changed-msg)
      (ros::subscribe candidates-topic jsk_recognition_msgs::LabelArray
                      #'(lambda (msg)
                          (let ((st (send msg :header :stamp)))
                            (when (> (send st :to-sec) (send stamp :to-sec))
                              (setq candidates-msg msg)))))
      (ros::subscribe changed-topic jsk_recognition_msgs::BoolStamped
                      #'(lambda (msg)
                          (let ((st (send msg :header :stamp)))
                            (when (> (send st :to-sec) (send stamp :to-sec))
                              (setq changed-msg msg)))))
      (while (and (not (and candidates-msg changed-msg))
                  (> (+ (send stamp :to-sec) 10)
                     (send (ros::time-now) :to-sec)))
        (unix::usleep (* 50 1000))
        (ros::spin-once))
      (ros::unsubscribe candidates-topic)
      (ros::unsubscribe changed-topic)

      (if (and candidates-msg changed-msg)
        (if (send changed-msg :data)
          (mapcar #'(lambda (label-msg) (send label-msg :name)) (send candidates-msg :labels))
          :no-change)
        :timeout)))
  (:reset-scale (arm)
    (let ((service-name (format nil "/~a_hand_camera/weight_candidates_refiner/reset" (arm2str arm)))
          (req (instance std_srvs::TriggerRequest :init)))
      (ros::wait-for-service service-name)
      (send (ros::service-call service-name req) :success)))
  (:update-object-tables (obj bin &key object-type)
    (cond
      ((eq object-type :unfinished-objects)
       (sethash bin unfinished-objects-
                (remove-duplicates (append (gethash bin unfinished-objects-) (list obj))
                                   :test #'string=)))
      ((eq object-type :postponed-objects)
       (sethash bin unfinished-objects-
                (remove-duplicates (remove obj (gethash bin unfinished-objects-) :test #'string=)
                                   :test #'string=))
       (sethash bin postponed-objects-
                (remove-duplicates (append (gethash bin postponed-objects-) (list obj))
                                   :test #'string=)))
      ((eq object-type :finished-objects)
       ;; first, add into postponed-objects-
       (send self :update-object-tables obj bin :object-type :postponed-objects)
       ;; then move to finished-objects-
       ;; this is for the case that targets are same object.
       (sethash bin postponed-objects-
                (remove-duplicates (remove obj (gethash bin postponed-objects-) :test #'string=)
                                   :test #'string=))
       (sethash bin finished-objects-
                (remove-duplicates (append (gethash bin finished-objects-) (list obj))
                                   :test #'string=)))
      (t nil)))
  (:set-objects-param (arm bin &key object-type)
    (cond
      ((eq object-type :unfinished-objects)
       (ros::set-param
         (format nil "/~a_hand/unfinished_objects/~a" (arm2str arm)
                 (if (eq bin :tote) (symbol2str bin) (format nil "bin_~a" (symbol-string bin))))
         (gethash bin unfinished-objects-)))
      ((eq object-type :finished-objects)
       (ros::set-param
         (format nil "/~a_hand/finished_objects/~a" (arm2str arm)
                 (if (eq bin :tote) (symbol2str bin) (format nil "bin_~a" (symbol-string bin))))
         (gethash bin finished-objects-)))
      ((eq object-type :postponed-objects)
       (ros::set-param
         (format nil "/~a_hand/postponed_objects/~a" (arm2str arm)
                 (if (eq bin :tote) (symbol2str bin) (format nil "bin_~a" (symbol-string bin))))
         (gethash bin postponed-objects-)))
      ((eq object-type :all)
       (send self :set-objects-param arm bin :object-type :unfinished-objects)
       (send self :set-objects-param arm bin :object-type :finished-objects)
       (send self :set-objects-param arm bin :object-type :postponed-objects))
      (t nil)))
  (:reset-object-tables (location-type)
    (cond
      ((eq location-type :bins)
       (when (eq (length orders-) 0)
         (ros::ros-error "[:set-unfinished-objects] There is no order, :get-work-orders first.")
         (return-from :set-unfinished-objects nil))
       (dolist (order orders-)
         (let ((bin (str2symbol (send order :bin)))
               (obj (send order :item)))
           (send self :update-object-tables obj bin :object-type :unfinished-objects)))
       (dolist (bin (list :a :b :c))
         (sethash bin finished-objects- nil)
         (sethash bin postponed-objects- nil)))
      ((eq location-type :tote)
       (let (tote-contents)
         (setq tote-contents (ros::get-param "/tote_contents"))
         (sethash :tote unfinished-objects- tote-contents)
         (sethash :tote finished-objects- nil)
         (sethash :tote postponed-objects- nil)))
      (t nil)))
  (:add-finished-object (arm obj bin)
    (send self :update-object-tables obj bin :object-type :finished-objects)
    (send self :set-objects-param arm bin :object-type :unfinished-objects)
    (send self :set-objects-param arm bin :object-type :postponed-objects)
    (send self :set-objects-param arm bin :object-type :finished-objects))
  (:add-postponed-object (arm obj bin)
    (send self :update-object-tables obj bin :object-type :postponed-objects)
    (send self :set-objects-param arm bin :object-type :unfinished-objects)
    (send self :set-objects-param arm bin :object-type :postponed-objects))
  (:get-grasp-style (arm obj trial-time)
    (let ((service-name
           (format nil "/grasp_style_server/get_grasp_style"))
          req res succeeded)
      (ros::wait-for-service service-name)
      (setq req (instance jsk_arc2017_common::GetGraspStyleRequest :init))
      (send req :item obj)
      (send req :trial_time trial-time)
      (setq res (ros::service-call service-name req))
      (setq succeeded (send res :success))
      (if succeeded
        (str2symbol (send res :style))
        (progn
          (ros::ros-warn "[:get-grasp-style] arm: ~a, cannot get grasp-style" (arm2str arm))
          (ros::ros-warn "[:get-grasp-style] message: ~a" (send res :message))
          succeeded))))
  (:decide-bin-return-area (arm bin)
    (let (bin-cube region region-x index)
      (setq bin-cube (gethash bin bin-cubes-))
      (setq region (send self :cube->movable-region bin-cube))
      (setq region-x (- (cadr (elt region 0)) (car (elt region 0))))
      (setq index (random 2))
      (cond
       ((eq index 0)
        (float-vector (/ (- region-x) 5.0)
                      0.0
                      0.0))
       ((eq index 1)
        (float-vector (/ region-x 5.0)
                      0.0
                      0.0)))))
  (:decide-tote-return-area (arm)
    (let (tote-cube region region-y)
      (setq tote-cube (gethash arm tote-cubes-))
      (setq region (send self :cube->movable-region tote-cube))
      (setq region-y (- (cadr (elt region 1)) (car (elt region 1))))
      (if (eq arm :larm)
        (float-vector 0 (/ region-y 8.0) 0)
        (float-vector 0 (/ region-y -8.0) 0))))
  (:decide-bin-place-area (arm bin grasp-style)
    (let (bin-cube region x-range index)
      (setq bin-cube (gethash bin bin-cubes-))
      (setq region (send self :cube->movable-region bin-cube))
      (setq x-range (- (cadr (elt region 0)) (car (elt region 0))))
      (if (eq grasp-style :suction)
        (progn
          (setq index (random 3))
          (cond
           ((eq index 0)
            (float-vector 0 0 0))
           ((eq index 1)
            (float-vector (/ x-range 5.0) 0 0))
           ((eq index 2)
            (float-vector (/ x-range -5.0) 0 0))))
        (float-vector 0 0 0))))
  (:decide-cardboard-place-area (arm cardboard)
    (let (cardboard-cube region region-y)
      (setq cardboard-cube (gethash cardboard cardboard-cubes-))
      (setq region (send self :cube->movable-region cardboard-cube))
      (setq region-y (- (cadr (elt region 1)) (car (elt region 1))))
      (if (= (random 2) 0)
        (float-vector -50.0 (/ region-y -7.0) 0.0)
        (float-vector -50.0 (/ region-y 7.0) 0.0))))
  (:get-stowing-target-bin (arm)
    (let ((bins (if (eq arm :larm) (list :a :b) (list :b :c)))
          obj-bbox target-bin obj-bbox-volume)
      (setq object-volumes
            (mapcar #'(lambda (bin) (gethash bin bin-object-volumes-)) bins))
      (setq target-bin (elt bins (argmin object-volumes)))
      (setq obj-bbox (elt (gethash arm object-boxes-) 0))
      (setq obj-bbox-volume (* (send obj-bbox :dimensions :x)
                               (send obj-bbox :dimensions :y)
                               (send obj-bbox :dimensions :z)))
      (sethash target-bin bin-object-volumes-
               (+ (gethash target-bin bin-object-volumes-) obj-bbox-volume))
      target-bin)))

(defun jsk_arc2017_baxter::arc-init (&key (ctype :default-controller) (moveit nil))
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
      (setq *ti* (instance jsk_arc2017_baxter::arc-interface :init))
      (send *ti* :set-robot-interface *ri*))
    (send (send *ti* :get-robot) :angle-vector
          (send (send *ti* :get-robot-interface) :state :potentio-vector))
    (send (send *ti* :get-robot-interface) :calib-grasp :arms)))
