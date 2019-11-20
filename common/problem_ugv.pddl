;;--------------------------------------;;
;; This file contains the initial state ;;
;; that is loaded into the strategic    ;;
;; knowledge base.                      ;;
;;--------------------------------------;;

(define (problem redone_strategic)
(:domain redone_strategic)
(:objects
    sky_wp0 sky_wp1 sky_wp2 sky_wp3 - sky
    ground_wp0 ground_wp1 ground_wp2 ground_wp3 ground_wp4 ground_wp5 ground_wp6 - ground
    ugv1 ugv2 ugv0 - ugv
    uav01 - uav
    dock1 dock2 - dock
)
(:init

    ;;---------------;;
    ;; initial state ;;
    ;;---------------;;

    ;; UGV
    (ugv0_at ground_wp1)
    (ugv1_at ground_wp5)
    (ugv2_at ground_wp6)

    (visited ground_wp1)
    (visited ground_wp5)
    (visited ground_wp6)


    ;; UAV
    (uav_not_on_mission uav01)
    (docked_at uav01 dock1)
    (docked uav01)
    (not_recharging uav01)

    (= (charge uav01) 7200)

    ;; DOCK
	(dock_at dock1 ground_wp1)
	(dock_at dock2 ground_wp1)
    (dock_free dock2)

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
    (docked uav01)
    (visited ground_wp2)
    (visited ground_wp3)
    (visited ground_wp4)
)))
