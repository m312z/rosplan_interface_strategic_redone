;;--------------------------------------;;
;; This file contains the initial state ;;
;; that is loaded into the tactical     ;;
;; knowledge base. It does not contain  ;;
;; goals.                               ;;
;;--------------------------------------;;

(define (problem tactical_template)
(:domain redone_tactical)
(:objects
    sky_wp0 sky_wp1 sky_wp2 sky_wp3 - sky
    ground_wp0 ground_wp1 ground_wp2 ground_wp3 ground_wp4 ground_wp5 ground_wp6 - ground
    uav01 - uav
    dock1 dock2 - dock
)
(:init

    ;;----------------------------;;
    ;; template UAV initial state ;;
    ;;----------------------------;;

    (uav_at uav01 sky_wp0)
    (docked_at uav01 dock1)
    (not_recharging uav01)

    ;; DOCK
    (dock_at dock1 ground_wp1)
    (dock_at dock2 ground_wp1)
    (dock_free dock2)

    ;;--------------;;
    ;; static state ;;
    ;;--------------;;

    (= (distance sky_wp0 ground_wp0) 5.0)
    (= (distance ground_wp0 sky_wp0) 5.0)

    (= (distance ground_wp0 ground_wp1) 5.0)
    (= (distance ground_wp1 ground_wp0) 5.0)
    (= (distance ground_wp1 ground_wp2) 5.0)
    (= (distance ground_wp1 ground_wp3) 5.0)
    (= (distance ground_wp2 ground_wp1) 5.0)
    (= (distance ground_wp2 ground_wp3) 5.0)
    (= (distance ground_wp3 ground_wp1) 5.0)
    (= (distance ground_wp3 ground_wp2) 5.0)
    (= (distance ground_wp3 ground_wp4) 5.0)
    (= (distance ground_wp3 ground_wp5) 5.0)
    (= (distance ground_wp4 ground_wp3) 5.0)
    (= (distance ground_wp4 ground_wp5) 5.0)
    (= (distance ground_wp5 ground_wp3) 5.0)
    (= (distance ground_wp5 ground_wp4) 5.0)
    (= (distance ground_wp5 ground_wp6) 5.0)
    (= (distance ground_wp6 ground_wp3) 5.0)
    (= (distance ground_wp6 ground_wp4) 5.0)
    (= (distance ground_wp6 ground_wp5) 5.0)

    (= (distance sky_wp0 ground_wp0) 5.0)
    (= (distance ground_wp0 sky_wp0) 5.0)

    (= (distance sky_wp0 sky_wp1) 5.0)
    (= (distance sky_wp0 sky_wp2) 5.0)
    (= (distance sky_wp0 sky_wp3) 5.0)
    (= (distance sky_wp1 sky_wp0) 5.0)
    (= (distance sky_wp1 sky_wp2) 5.0)
    (= (distance sky_wp1 sky_wp3) 5.0)
    (= (distance sky_wp2 sky_wp0) 5.0)
    (= (distance sky_wp2 sky_wp1) 5.0)
    (= (distance sky_wp2 sky_wp3) 5.0)
    (= (distance sky_wp3 sky_wp0) 5.0)
    (= (distance sky_wp3 sky_wp1) 5.0)
    (= (distance sky_wp3 sky_wp2) 5.0)
)
(:goal (and
)))
