#!/usr/bin/env roseus

(ros::load-ros-manifest "roseus")
(ros::load-ros-manifest "force_proximity_ros")

(defclass proximity-calibrator
  :super propertied-object
  :slots (proximities-calibrated-
          proximities-init-
          proximities-param-a-
          topic-list-))

(defmethod proximity-calibrator
  (:init (&rest args &key topic-list)
    (ros::roseus "proximity-calibrator")
    ;; create subscribers and publishers
    (ros::advertise-service "/set_init_proximities" std_srvs::Empty #'send self :set-init-proximities)
    (dolist (topic-name topic-list)
      (ros::subscribe topic-name force_proximity_ros::ProximityArray #'send self :cb topic-name)
      (ros::advertise (format nil "~A/init" topic-name) std_msgs::Float64MultiArray 1)
      (ros::advertise (format nil "~A/calibrated" topic-name) std_msgs::Float64MultiArray 1)
      (ros::advertise (format nil "~A/distance" topic-name) std_msgs::Float64MultiArray 1))
    ;; make hash tables
    (setq proximities-calibrated- (make-hash-table :test #'equal))
    (setq proximities-init- (make-hash-table :test #'equal))
    (setq proximities-param-a- (make-hash-table :test #'equal))
    ;; get default proximity parameters from rosparam
    (dolist (topic-name topic-list)
      (sethash topic-name
               proximities-param-a-
               (ros::get-param (format nil "~A/a" topic-name)))
      (sethash topic-name
               proximities-init-
               (ros::get-param (format nil "~A/b" topic-name))))
    (ros::spin-once)
    self)
  (:cb (topic-name msg)
    ;; set calibrated sensor value to hash table
    (sethash topic-name
             proximities-calibrated-
             (mapcar #'- (mapcar #'(lambda (x) (send x :proximity)) (send msg :proximities))
                     (gethash topic-name proximities-init-)))
    ;; publish distance calculated from proximtiy values
    (send self :publish-proximities topic-name)
    )
  (:publish-proximities (topic-name)
    (let ((float-array (instance std_msgs::Float64MultiArray :init)))
      (send float-array :data (gethash topic-name proximities-init-))
      (ros::publish (format nil "~A/init" topic-name) float-array)
      (send float-array :data (gethash topic-name proximities-calibrated-))
      (ros::publish (format nil "~A/calibrated" topic-name) float-array)
      (send float-array :data
            (mapcar #'(lambda (a I-b)
                        (if (> I-b 0)
                          (sqrt (/ a I-b))
                          *inf*))
                    (gethash topic-name proximities-param-a-)
                    (gethash topic-name proximities-calibrated-)))
      (ros::publish (format nil "~A/distance" topic-name) float-array)))
  (:set-init-proximities (req)
    (dolist (topic-name topic-list-)
      (sethash topic-name
               proximities-init-
               (gethash topic-name proximities-raw-)))
    (instance std_srvs::EmptyResponse :init))
  )


(defun init (&optional (topic-list (list "/proximity_sensor_topic1" "/proximity_sensor_topic2")))
  (when (not (boundp '*pc*))
    (setq *pc* (instance proximity-calibrator :init :topic-list topic-list)))
  (ros::spin-once))


(init)
(ros::spin)
