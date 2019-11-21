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
    (visited ground_wp1)
    (visited ground_wp5)
    (visited ground_wp6)


    ;; UAV
    (uav_not_on_mission uav01)
    (docked_at uav01 dock1)
    (not_recharging uav01)

    (= (charge uav01) 7200)

    ;; DOCK
	(dock_at dock1 ground_wp1)
	(dock_at dock2 ground_wp1)
    (dock_free dock2)

    ;;--------------;;
    ;; static state ;;
    ;;--------------;;

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
