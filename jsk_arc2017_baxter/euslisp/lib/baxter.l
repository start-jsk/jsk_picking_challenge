#!/usr/bin/env roseus

;; vim: set ft=lisp:

(require "package://jsk_2015_05_baxter_apc/euslisp/lib/util.l")
(require :baxter "package://jsk_arc2017_baxter/robots/baxter.l")
(require "package://baxtereus/baxter-util.l")

(unless (find-package "JSK_ARC2017_BAXTER")
  (make-package "JSK_ARC2017_BAXTER"))


(defclass jsk_arc2017_baxter::baxter-robot
  :super baxter-robot
  :slots (view-hand-pos- ik-prepared-poses- hand-joints-))

(defmethod jsk_arc2017_baxter::baxter-robot
  (:init
    (&rest args)
    (send-super* :init args)
    ;; initialize view-hand-pos-
    (setq view-hand-pos- (make-hash-table))
    (sethash :rarm view-hand-pos- (make-hash-table))
    (sethash :larm view-hand-pos- (make-hash-table))
    ;; for larm
    (sethash :a (gethash :larm view-hand-pos-) #f(785.344 229.224 762.48))
    (sethash :b (gethash :larm view-hand-pos-) #f(813.742 10.946 828.431))
    (sethash :d (gethash :larm view-hand-pos-) #f(807.059 275.852 633.668))
    (sethash :e (gethash :larm view-hand-pos-) #f(770.657 57.875 550.976))
    (sethash :g (gethash :larm view-hand-pos-) #f(781.985 244.363 368.102))
    (sethash :h (gethash :larm view-hand-pos-) #f(819.589 13.426 415.42))
    (sethash :j (gethash :larm view-hand-pos-) #f(811.875 312.473 156.184))
    (sethash :k (gethash :larm view-hand-pos-) #f(780.791 15.733 137.103))
    ;; for rarm
    (sethash :b (gethash :rarm view-hand-pos-) #f(813.742 -10.946 828.431))
    (sethash :c (gethash :rarm view-hand-pos-) #f(785.344 -180 762.48))
    (sethash :e (gethash :rarm view-hand-pos-) #f(770.657 -57.875 550.976))
    (sethash :f (gethash :rarm view-hand-pos-) #f(807.059 -186 633.668))
    (sethash :h (gethash :rarm view-hand-pos-) #f(819.589 -13.426 415.42))
    (sethash :i (gethash :rarm view-hand-pos-) #f(781.985 -184 368.102))
    (sethash :k (gethash :rarm view-hand-pos-) #f(780.791 -15.733 137.103))
    (sethash :l (gethash :rarm view-hand-pos-) #f(811.875 -180 156.184))
    ;; pos of :c, :f, :i, :l is not symmetrical to :a, :d, :g, :j because torso can't see
    ;; initialize ik-prepared-poses-
    (setq ik-prepared-poses- (list :untuck-pose))
    ;; initialize hand-joints-
    (setq hand-joints- (make-hash-table))
    (sethash :rarm hand-joints-
             (list :right_gripper_finger_yaw_joint
                   :right_gripper_finger_roll_joint
                   ;; We have to move mimic joints, too
                   :right_gripper_r_finger_yaw_joint
                   :right_gripper_r_finger_roll_joint))
    (sethash :larm hand-joints-
             (list :left_gripper_finger_yaw_joint
                   :left_gripper_finger_roll_joint
                   ;; We have to move mimic joints, too
                   :left_gripper_r_finger_yaw_joint
                   :left_gripper_r_finger_roll_joint))
    ;; translate palm end for good cylindrical grasp
    (send self :rarm-palm-endpoint :translate #f(30 0 0) :local)
    (send self :larm-palm-endpoint :translate #f(30 0 0) :local)
    )

  (:arc-reset-pose
    (&optional (arm :arms))
    (let ((arms (if (eq arm :arms) (list :larm :rarm) (list arm))))
      (dolist (arm arms)
        (send self :untuck-pose arm)
        (send self arm :move-end-pos (float-vector 0 (if (eq arm :larm) 300 -300) 0) :world)
        (send self :rotate-wrist arm (if (eq arm :larm) 90 -90) :relative nil)
        ))
    (send self :angle-vector))

  (:rarm-palm-endpoint
    (&rest args)
    (send* self :right_gripper_palm_endpoint_lk args))
  (:larm-palm-endpoint
    (&rest args)
    (send* self :left_gripper_palm_endpoint_lk args))
  (:set-ik-prepared-poses
    (poses)
    (if (listp poses)
      (setq ik-prepared-poses- poses))
    )
  (:ik-prepared-poses () ik-prepared-poses-)
  (:inverse-kinematics
    (target-coords
      &rest args
      &key move-target
      (link-list
        (if (atom move-target)
          (send self :link-list (send move-target :parent))
          (mapcar #'(lambda (mt) (send self :link-list (send mt :parent))) move-target)))
      (use-gripper nil) (move-palm-end nil) (rthre (deg2rad 10))
      &allow-other-keys)
    ;; currently works only if move-target is not list
    ;; set endpoint of palm as move-target
    (if move-palm-end
      (dolist (limb (list :rarm :larm))
        (when (eq move-target (send self limb :end-coords))
          (setq move-target (send self limb :palm-endpoint))
          (setq link-list (send self :link-list (send move-target :parent))))))
    ;; if the last link of link-list is in gripper, remove gripper links
    (if (null use-gripper)
      (cond ((equal (send (car (last link-list)) :name) "right_gripper_pad_with_base")
             (setq link-list (butlast link-list 2)))
            ((equal (send (car (last link-list)) :name) "left_gripper_pad_with_base")
             (setq link-list (butlast link-list 2))))
      nil)
    (send-super* :inverse-kinematics target-coords
                 :move-target move-target :link-list link-list :rthre rthre args)
    )
  (:rotate-wrist
    (arm angle &key (relative t))
    (send self arm :wrist-r :joint-angle angle :relative relative)
    (send self :angle-vector)
    )
  (:rotate-gripper
    (arm angle &key (relative t))
    (send self arm :gripper-p :joint-angle angle :relative relative)
    (send self :angle-vector)
    )
  (:slide-gripper
    (arm pos &key (relative t))
    (send self arm :gripper-x :joint-angle pos :relative relative)
    (send self :angle-vector))
  (:hand
    (arm &rest args)
    (case (car args)
      ((:angle-vector)
       (let ((av (cadr args)) (joints (gethash arm hand-joints-)))
         (if (and (null (eq (length av) 0)) (null (eq (length av) 2)))
           (progn (ros::ros-error "length of angle-vector must be 0 or 2.~%") (exit)))
         (dotimes (i (length av))
           (send self (elt joints i) :joint-angle (elt av i))
           (send self (elt joints (+ i (length av))) :joint-angle (elt av i)))
         (mapcar
           #'(lambda (j) (send self j :joint-angle))
           (subseq joints 0 (/ (length joints) 2)))
         ))
      (t (ros::ros-error ":hand first arg is invalid. args: ~A~%" args) (exit))
      ))
  (:hand-grasp-pre-pose
    (arm style)
    (case style
      (:opposed (send self :hand arm :angle-vector #f(0 0)))
      (:spherical (send self :hand arm :angle-vector #f(30 0)))
      (:cylindrical (send self :hand arm :angle-vector #f(90 0)))
      (t (ros::ros-error ":hand-grasp-pre-pose no such style ~A~%" style) (exit))
    ))
  (:hand-grasp-pose
    (arm style &key (angle 180))
    (case style
      (:opposed (send self :hand arm :angle-vector (float-vector 0 angle)))
      (:spherical (send self :hand arm :angle-vector (float-vector 30 angle)))
      (:cylindrical (send self :hand arm :angle-vector (float-vector 90 angle)))
      (t (ros::ros-error ":hand-grasp-pose no such style ~A~%" style) (exit))
    ))
  (:avoid-shelf-pose
    (arm bin)
    (case bin
      (:a (send self :avoid-shelf-pose-a arm))
      (:b (send self :avoid-shelf-pose-b arm))
      (:c (send self :avoid-shelf-pose-c arm))
      (:d (send self :avoid-shelf-pose-d arm))
      (:e (send self :avoid-shelf-pose-e arm))
      (:f (send self :avoid-shelf-pose-f arm))
      (:g (send self :avoid-shelf-pose-g arm))
      (:h (send self :avoid-shelf-pose-h arm))
      (:i (send self :avoid-shelf-pose-i arm))
      (:j (send self :avoid-shelf-pose-j arm))
      (:k (send self :avoid-shelf-pose-k arm))
      (:l (send self :avoid-shelf-pose-l arm)))
    )
  (:view-hand-pose
    (arm bin)
    (case bin
      (:a (send self :view-hand-pose-a arm))
      (:b (send self :view-hand-pose-b arm))
      (:c (send self :view-hand-pose-c arm))
      (:d (send self :view-hand-pose-d arm))
      (:e (send self :view-hand-pose-e arm))
      (:f (send self :view-hand-pose-f arm))
      (:g (send self :view-hand-pose-g arm))
      (:h (send self :view-hand-pose-h arm))
      (:i (send self :view-hand-pose-i arm))
      (:j (send self :view-hand-pose-j arm))
      (:k (send self :view-hand-pose-k arm))
      (:l (send self :view-hand-pose-l arm)))
    (let ((pos (gethash bin (gethash arm view-hand-pos-))))
      (if pos
        (send self arm :inverse-kinematics
              (make-coords :pos pos)
              :rotation-axis nil
              :revert-if-fail nil)
        )
      )
    )
  (:visualize-path
    (avs &key (limb))
    (let (robots coords-list)
      (dotimes (i (length avs))
        (let ((robot (instance (class self) :init)))
          (send robot :angle-vector (elt avs i))
          (when (not (eq i (1- (length avs))))
            (send-all (send robot :bodies) :set-color nil 0.3)
            )
          (pushback robot robots)
          (when limb
            (pushback (send robot limb :end-coords) coords-list)
            )
          )
        )
      (objects (append robots coords-list))
      )
    )
  (:view-opposite-hand-pose
    (arm &key (dy 300))
    (let (opposite-arm gripper-coords theta)
      (setq opposite-arm (if (eq arm :larm) :rarm :larm))
      (setq gripper-coords
            (send (send self opposite-arm :end-coords)
                  :copy-worldcoords))
      (send gripper-coords :orient 0 :z :world)
      (if (eq arm :larm)
        (setq theta -1.57)
        (progn
          (setq dy (- dy))
          (setq theta 1.57)))
      (send gripper-coords :translate (float-vector 0 dy -200))
      (send gripper-coords :rotate theta :z :world)
      (send self :rotate-gripper arm 70 :relative nil)
      (send self arm
            :inverse-kinematics gripper-coords
            :use-gripper nil
            :rotation-axis t
            :revert-if-fail nil)))
  (:get-support-arm-coords
    (arm &key (dx 0) (dy 0) (dz 0))
    (let (support-arm-coords)
      (setq support-arm-coords
            (send (send self arm :end-coords)
                  :copy-worldcoords))
      (send support-arm-coords :orient 0 :z :world)
      (send support-arm-coords :rotate 3.14 :y :world)
      (send support-arm-coords :translate (float-vector dx dy dz) :world)
      support-arm-coords))
  (:ik->nearest-pose
    (arm target-coords pose-candidates &rest args)
    (let ((opposite-arm (if (eq arm :larm) :rarm :larm))
          current-opposite-av current-av pose-and-next-avs chosen-pose)
      (unless pose-candidates
        (setq pose-candidates (list (send self :angle-vector))))
      (setq current-opposite-av (send self opposite-arm :angle-vector))
      (setq current-av (send self arm :angle-vector))
      ;; exclude poses which IK fail from
      (dolist (pose pose-candidates)
        (send self :angle-vector pose)
        (when (send* self arm :inverse-kinematics target-coords args)
          (pushback (cons pose (send self arm :angle-vector)) pose-and-next-avs)))
      (unless pose-and-next-avs
        (ros::ros-error "[:ik->nearest-pose] Cannot solve IK from poses")
        (send self opposite-arm :angle-vector current-opposite-av)
        (send self arm :angle-vector current-av)
        (return-from :ik->nearest-pose nil))
      (setq chosen-pose
            (car (car (sort pose-and-next-avs #'<
                            #'(lambda (pose-and-next-av)
                                (norm (v- current-av (cdr pose-and-next-av))))))))
      (format t "[:ik->nearest-pose] arm:~a midpose: ~a~%" arm chosen-pose)
      (send self :angle-vector chosen-pose)
      (send self opposite-arm :angle-vector current-opposite-av)
      (send* self arm :inverse-kinematics target-coords args)))
  (:spin-off-by-wrist
    (arm &key (times 10))
    (let (avs robot)
      (pushback (send self :angle-vector) avs)
      (dotimes (i times)
        (pushback (send self :rotate-wrist arm 10) avs)
        (pushback (send self :rotate-wrist arm -20) avs)
        (pushback (send self :rotate-wrist arm 20) avs)
        (pushback (send self :rotate-wrist arm -10) avs))
      avs))
  (:rotate-wrist-ik
    (arm target-coords &rest args)
    (let (pose-candidates min-av max-av)
      (setq pose-candidates (list (send self :angle-vector)))
      (send self arm :wrist-r :joint-angle (send self arm :wrist-r :min-angle))
      (setq min-av (send self :angle-vector))
      (send self arm :wrist-r :joint-angle (send self arm :wrist-r :max-angle))
      (setq max-av (send self :angle-vector))
      (let ((sampling-ratios (list 0 0.25 0.5 0.75 1)))
        (when (eq arm :rarm) (nreverse sampling-ratios))
        (dolist (p sampling-ratios)
          (pushback (midpoint p min-av max-av) pose-candidates)
          )
        )
      (send self :angle-vector (car pose-candidates))
      (send* self :ik->nearest-pose arm target-coords pose-candidates args))))


(defclass jsk_arc2017_baxter::baxter-robot-safe
  :super jsk_arc2017_baxter::baxter-robot
  :slots (collision-ignore-pairs))

(defmethod jsk_arc2017_baxter::baxter-robot-safe
  (:init (&rest args)
    (send-super* :init args)
    ;; collision-avoidance-links
    (setq collision-avoidance-links
          (list (send self :screen_lk)
                (send self :head_lk)
                (send self :larm :wrist-r :child-link)
                (send self :larm :wrist-p :child-link)
                (send self :larm :elbow-p :child-link)
                (send self :larm :elbow-r :child-link)
                (send self :larm :shoulder-r :child-link)
                (send self :larm :shoulder-p :child-link)
                (send self :larm :shoulder-y :child-link)
                (send self :left_gripper_r_finger_base_lk)
                (send self :left_gripper_palm_endpoint_lk)
                (send self :left_gripper_l_finger_base_lk)
                (send self :left_gripper_tube_lk)
                (send self :left_gripper_r_finger_lk)
                (send self :left_gripper_l_finger_lk)
                (send self :left_gripper_pad_with_base_lk)
                (send self :rarm :wrist-r :child-link)
                (send self :rarm :wrist-p :child-link)
                (send self :rarm :elbow-p :child-link)
                (send self :rarm :elbow-r :child-link)
                (send self :rarm :shoulder-r :child-link)
                (send self :rarm :shoulder-p :child-link)
                (send self :rarm :shoulder-y :child-link)
                (send self :right_gripper_r_finger_base_lk)
                (send self :right_gripper_palm_endpoint_lk)
                (send self :right_gripper_l_finger_base_lk)
                (send self :right_gripper_tube_lk)
                (send self :right_gripper_r_finger_lk)
                (send self :right_gripper_l_finger_lk)
                (send self :right_gripper_pad_with_base_lk)))
         (setq collision-ignore-pairs
               (list (cons (send self :larm :shoulder-r :child-link)
                           (send self :larm :elbow-r :child-link))
                     (cons (send self :rarm :shoulder-r :child-link)
                           (send self :rarm :elbow-r :child-link))
                     (cons (send self :left_gripper_l_finger_lk)
                           (send self :left_gripper_r_finger_base_lk))
                     (cons (send self :left_gripper_l_finger_lk)
                           (send self :left_gripper_r_finger_base_lk))
                     (cons (send self :left_gripper_l_finger_lk)
                           (send self :left_gripper_pad_with_base_lk))
                     (cons (send self :left_gripper_l_finger_lk)
                           (send self :left_gripper_l_finger_base_lk))
                     (cons (send self :left_gripper_l_finger_lk)
                           (send self :left_gripper_r_finger_lk))
                     (cons (send self :left_gripper_l_finger_lk)
                           (send self :left_gripper_r_finger_base_lk))
                     (cons (send self :left_gripper_l_finger_base_lk)
                           (send self :left_gripper_r_finger_lk))
                     (cons (send self :left_gripper_l_finger_base_lk)
                           (send self :left_gripper_r_finger_base_lk))
                     (cons (send self :right_gripper_pad_with_base_lk)
                           (send self :right_gripper_r_finger_lk))
                     (cons (send self :right_gripper_l_finger_lk)
                           (send self :right_gripper_r_finger_base_lk))
                     (cons (send self :right_gripper_l_finger_lk)
                           (send self :right_gripper_r_finger_base_lk))
                     (cons (send self :right_gripper_l_finger_lk)
                           (send self :right_gripper_pad_with_base_lk))
                     (cons (send self :right_gripper_l_finger_lk)
                           (send self :right_gripper_l_finger_base_lk))
                     (cons (send self :right_gripper_l_finger_lk)
                           (send self :right_gripper_r_finger_lk))
                     (cons (send self :right_gripper_l_finger_lk)
                           (send self :right_gripper_r_finger_base_lk))
                     (cons (send self :right_gripper_l_finger_base_lk)
                           (send self :right_gripper_r_finger_lk))
                     (cons (send self :right_gripper_l_finger_base_lk)
                           (send self :right_gripper_r_finger_base_lk))
                     (cons (send self :right_gripper_pad_with_base_lk)
                           (send self :right_gripper_r_finger_lk))))
    (setq larm-collision-avoidance-links collision-avoidance-links)
    (setq rarm-collision-avoidance-links collision-avoidance-links))
  (:collision-ignore-pairs (&optional pairs)
    (when pairs (setq collision-ignore-pairs pairs))
    collision-ignore-pairs)
  (:remove-collision-pairs (pairs remove-pairs)
    (dolist (pair remove-pairs)
      (delete-if #'(lambda (x)
                     (or (and (eq (car x) (car pair))
                              (eq (cdr x) (cdr pair)))
                         (and (eq (car x) (cdr pair))
                              (eq (cdr x) (car pair)))))
                 pairs))
    pairs)
  (:collision-check-pairs
    (&key (links collision-avoidance-links) (ignore-pairs (send self :collision-ignore-pairs)))
    (send self :remove-collision-pairs
          (send-super :collision-check-pairs :links links) ignore-pairs))
  (:self-collision-check
    (&rest args &key (pairs (send self :collision-check-pairs :links collision-avoidance-links))
           &allow-other-keys)
    (send-super* :self-collision-check :pairs pairs args)))
