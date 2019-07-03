(define (domain redone_tactical)

(:requirements :strips :typing :fluents :disjunctive-preconditions :durative-actions :equality :duration-inequalities :timed-initial-literals)

(:types
	ground sky - waypoint 
	uav ugv - robot
    dock
    mission
)

(:predicates

    ;; common predicates ;;
    (connected ?wp1 ?wp2 - waypoint)
    (can_observe ?wp1 - sky ?wp2 - ground)

    ;; UGV predicates ;;
	(ugv_at ?v - ugv ?wp - ground)
	(visited ?wp - ground)

    ;; UAV predicates ;;
	(uav_at ?u - uav ?wp - sky)
	(docked ?u - uav)
    (flying ?u - uav)
    (observed ?u - ugv)

    ;; dock predicates
	(dock_at ?d - dock ?wp - ground)
	(docked_at ?u - uav ?d - dock)
	(dock_free ?d - dock)

    ;; strategic predicates ;;
    (mission_complete ?m - mission)
    (uav_not_on_mission ?u - uav)
)

(:functions

    (distance ?wp1 ?wp2 - waypoint)

    ;; strategic functions ;;
    (mission_duration ?m - mission)
)

;;-------------;;
;; UGV ACTIONS ;;
;;-------------;;

;; Move between any two waypoints, avoiding terrain
(:durative-action goto_waypoint
	:parameters (?v - ugv ?from ?to - ground)
	:duration ( = ?duration (* 3 (distance ?from ?to)))
	:condition (and
		(at start (ugv_at ?v ?from))
        (at start (connected ?from ?to))
        )
	:effect (and
		(at start (not (ugv_at ?v ?from)))
		(at end (visited ?to))
		(at end (ugv_at ?v ?to)))
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
        )
	:effect (and
		(at start (not (uav_at ?v ?from)))
		(at end (uav_at ?v ?to)))
    )

(:durative-action observe_ugv
	:parameters (?v - uav ?obs_wp - sky ?wp - ground ?o - ugv)
	:duration (= ?duration 10)
	:condition (and
		(at start (ugv_at ?o ?wp))
		(at start (can_observe ?obs_wp ?wp))
		(over all (uav_at ?v ?obs_wp))
		(over all (flying ?v))
        )
	:effect (and
		(at end (observed ?o))
        )
    )

(:durative-action take_off
    :parameters (?v - uav ?d - dock ?ground_wp - ground ?sky_wp - sky)
	:duration (= ?duration 6)
    :condition (and
        (at start (can_observe ?sky_wp ?ground_wp))
        (at start (dock_at ?d ?ground_wp))
        (at start (docked_at ?v ?d))
        (at start (docked ?v))
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
        (at start (can_observe ?sky_wp ?ground_wp))
        (at start (flying ?v))
        (at start (uav_at ?v ?sky_wp))
		(at start (dock_at ?d ?ground_wp))
		(at start (dock_free ?d))
        )
    :effect(and
		(at start (not (dock_free ?d)))
        (at start (not (uav_at ?v ?sky_wp)))
        (at start (not (flying ?v)))
        (at end (docked_at ?v ?d))
        (at end (docked ?v))
        )
    )


;;-------------------;;
;; STRATEGIC ACTIONS ;;
;;-------------------;;

(:durative-action complete_mission
    :parameters (?uav - uav ?m - mission)
    :duration ( = ?duration (mission_duration ?m))
    :condition (and
	    (at start (docked ?uav))
        (at start (uav_not_on_mission ?uav))
	    )
    :effect (and
        (at start (not (uav_not_on_mission ?uav)))
        (at end (uav_not_on_mission ?uav))
	    (at end (mission_complete ?m))
	    )
    )
)
