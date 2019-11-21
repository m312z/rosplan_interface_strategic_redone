(define (domain redone_tactical)

(:requirements :strips :typing :fluents :disjunctive-preconditions :durative-actions :equality :duration-inequalities :timed-initial-literals)

(:types
	ground sky - waypoint
	uav - robot
	dock
)

(:predicates

    ;; statics (in both)
    (can_observe ?wp1 - sky ?wp2 - ground)

	(docked ?u - uav)
    (flying ?u - uav)
    (not_recharging ?u - uav)

    ;; tactical domain only
    (observed ?wp - ground)
	(uav_at ?u - uav ?wp - sky)

	;; dock predicates
	(dock_at ?d - dock ?wp - ground)
	(docked_at ?u - uav ?d - dock)
	(dock_free ?d - dock)
)

(:functions

    ;; static functions
    (distance ?wp1 ?wp2 - waypoint)
)

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
	:parameters (?v - uav ?obs_wp - sky ?wp - ground)
	:duration (= ?duration 10)
	:condition (and
		(at start (can_observe ?obs_wp ?wp))
		(over all (uav_at ?v ?obs_wp))
        )
	:effect (and
		(at end (observed ?wp))
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
        (at start (flying ?v))
        (at start (can_observe ?sky_wp ?ground_wp))
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
)
