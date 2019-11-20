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
)
(:init

    ;;----------------------------;;
    ;; template UAV initial state ;;
    ;;----------------------------;;

    (uav_at uav01 sky_wp0)

    ;;--------------;;
    ;; static state ;;
    ;;--------------;;

    (can_observe sky_wp0 ground_wp0)
    (can_observe sky_wp1 ground_wp1)
    (can_observe sky_wp1 ground_wp2)
    (can_observe sky_wp2 ground_wp3)
    (can_observe sky_wp2 ground_wp4)
    (can_observe sky_wp3 ground_wp5)
    (can_observe sky_wp3 ground_wp6)

    ;;(connected ground_wp0 ground_wp1)
    ;;(connected ground_wp1 ground_wp0)
    (connected ground_wp1 ground_wp2)
    (connected ground_wp1 ground_wp3)
    (connected ground_wp2 ground_wp1)
    (connected ground_wp2 ground_wp3)
    (connected ground_wp3 ground_wp1)
    (connected ground_wp3 ground_wp2)
    (connected ground_wp3 ground_wp4)
    (connected ground_wp3 ground_wp5)
    (connected ground_wp3 ground_wp6)
    (connected ground_wp4 ground_wp3)
    (connected ground_wp4 ground_wp5)
    (connected ground_wp4 ground_wp6)
    (connected ground_wp5 ground_wp3)
    (connected ground_wp5 ground_wp4)
    (connected ground_wp5 ground_wp6)
    (connected ground_wp6 ground_wp3)
    (connected ground_wp6 ground_wp4)
    (connected ground_wp6 ground_wp5)

    (connected sky_wp0 ground_wp0)
    (connected ground_wp0 sky_wp0)

    (connected sky_wp0 sky_wp1)
    (connected sky_wp0 sky_wp2)
    (connected sky_wp0 sky_wp3)
    (connected sky_wp1 sky_wp0)
    (connected sky_wp1 sky_wp2)
    (connected sky_wp1 sky_wp3)
    (connected sky_wp2 sky_wp0)
    (connected sky_wp2 sky_wp1)
    (connected sky_wp2 sky_wp3)
    (connected sky_wp3 sky_wp0)
    (connected sky_wp3 sky_wp1)
    (connected sky_wp3 sky_wp2)

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
