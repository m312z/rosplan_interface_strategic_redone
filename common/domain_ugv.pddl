(define (domain redone_strategic)

(:requirements :strips :typing :fluents :disjunctive-preconditions :durative-actions :equality :duration-inequalities :timed-initial-literals)

(:types
	ground sky - waypoint 
	uav ugv - robot
    dock
    mission
)

(:predicates

    ;; statics (in both)
    (can_observe ?wp1 - sky ?wp2 - ground)

    ;; UGV predicates
	(ugv0_at ?wp - ground)
    (ugv1_at ?wp - ground)
    (ugv2_at ?wp - ground)
	(visited ?wp - ground)
    (not_occupied ?wp - ground)

    ;; strategic UAV predicates
	(uav_at ?u - uav ?wp - sky)
	(docked ?u - uav)
    (flying ?u - uav)
    (not_recharging ?u - uav)
    (uav_not_on_mission ?u - uav)

    ;; strategic dock predicates
	(dock_at ?d - dock ?wp - ground)
	(docked_at ?u - uav ?d - dock)
	(dock_free ?d - dock)

    ;; strategic predicates
    (mission_complete ?m - mission)
    (mission_complete_waypoint ?m - mission ?wp - sky)
)

(:functions

    ;; static functions
    (distance ?wp1 ?wp2 - waypoint)

    ;; strategic functions
    (mission_duration ?m - mission)
    (charge ?u - uav)
)

;;-------------;;
;; UGV ACTIONS ;;
;;-------------;;

;; Move between any two waypoints, avoiding terrain
(:durative-action goto_waypoint0
	:parameters (?v - ugv ?from ?to - ground)
	:duration ( = ?duration (* 3 (distance ?from ?to)))
	:condition (and
        (at start (not_occupied ?to))
		(at start (ugv0_at ?from))
        )
	:effect (and
        (at start (not_occupied ?from))
        (at start (not (not_occupied ?to)))
		(at start (not (ugv0_at ?from)))
		(at end (visited ?to))
		(at end (ugv0_at ?to)))
    )

;; Move between any two waypoints, avoiding terrain
(:durative-action goto_waypoint1
    :parameters (?v - ugv ?from ?to - ground)
    :duration ( = ?duration (* 3 (distance ?from ?to)))
    :condition (and
        (at start (ugv1_at ?from))
        )
    :effect (and
        (at start (not (ugv1_at ?from)))
        (at end (visited ?to))
        (at end (ugv1_at ?to)))
    )

;; Move between any two waypoints, avoiding terrain
(:durative-action goto_waypoint2
    :parameters (?v - ugv ?from ?to - ground)
    :duration ( = ?duration (* 3 (distance ?from ?to)))
    :condition (and
        (at start (ugv2_at ?from))
        )
    :effect (and
        (at start (not (ugv2_at ?from)))
        (at end (visited ?to))
        (at end (ugv2_at ?to)))
    )

;;-------------;;
;; UAV ACTIONS ;;
;;-------------;;

;; Move between any two waypoints
(:durative-action flyto_waypoint
	:parameters (?v - uav ?from ?to - sky)
	:duration ( = ?duration 6)
	:condition (and
		(at start (uav_at ?v ?from))
        (at start (uav_not_on_mission ?v))
        (over all (uav_not_on_mission ?v))
		(over all (flying ?v))
        )
	:effect (and
		(at start (not (uav_at ?v ?from)))
		(at end (uav_at ?v ?to)))
    )

(:durative-action take_off
    :parameters (?v - uav ?d - dock ?ground_wp - ground ?sky_wp - sky)
	:duration (= ?duration 6)
    :condition (and
        (at start (can_observe ?sky_wp ?ground_wp))
        (at start (dock_at ?d ?ground_wp))
        (at start (docked_at ?v ?d))
        (at start (docked ?v))
        (at start (uav_not_on_mission ?v))
        (over all (uav_not_on_mission ?v))
        )
    :effect(and
        (at start (not (docked_at ?v ?d)))
        (at start (not (docked ?v)))
        (at end (uav_at ?v ?sky_wp))
        (at end (flying ?v))
		(at end (dock_free ?d))
        )
    )

(:durative-action land
    :parameters (?v - uav ?d - dock ?ground_wp - ground ?sky_wp - sky)
	:duration (= ?duration 10)
    :condition (and
        (at start (flying ?v))
        (at start (can_observe ?sky_wp ?ground_wp))
        (at start (uav_at ?v ?sky_wp))
		(at start (dock_at ?d ?ground_wp))
		(at start (dock_free ?d))
        (at start (uav_not_on_mission ?v))
        (over all (uav_not_on_mission ?v))
        )
    :effect(and
		(at start (not (dock_free ?d)))
        (at start (not (uav_at ?v ?sky_wp)))
        (at start (not (flying ?v)))
        (at end (docked_at ?v ?d))
        (at end (docked ?v))
        )
    )

(:durative-action recharge
    :parameters (?v - uav ?d - dock ?ground_wp - ground ?sky_wp - sky)
	:duration (and (>= ?duration 0) (<= ?duration 7200))
    :condition (and
        (at start (not_recharging ?v))
        (over all (docked ?v))
        (at start (uav_not_on_mission ?v))
        (over all (uav_not_on_mission ?v))
        (over all (<= (charge ?v) 7200))
        )
    :effect(and
        (at start (not (not_recharging ?v)))
        (at end (not_recharging ?v))
        (increase (charge ?v) (* 1 #t))
        )
    )

;;-------------------;;
;; STRATEGIC ACTIONS ;;
;;-------------------;;

(:durative-action complete_mission
    :parameters (?u - uav ?m - mission ?from ?to - sky)
    :duration ( = ?duration (mission_duration ?m))
    :condition (and
        (at start (uav_not_on_mission ?u))
        (at start (mission_complete_waypoint ?m ?to))
        (at start (uav_at ?u ?from))
        (over all (flying ?u))
        (at start (> (charge ?u) (* (mission_duration ?m) 6)))
	    )
    :effect (and
        (at start (not (uav_not_on_mission ?u)))
        (at start (not (uav_at ?u ?from)))
        (at end (uav_not_on_mission ?u))
	    (at end (mission_complete ?m))
        (at end (uav_at ?u ?to))
        (decrease (charge ?u) (* 6 #t))
	    )
    )
)
