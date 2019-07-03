(define (domain redone_tactical)

(:requirements :strips :typing :fluents :disjunctive-preconditions :durative-actions :equality :duration-inequalities :timed-initial-literals)

(:types
	ground sky - waypoint 
	uav ugv - robot
    mission
)

(:predicates
	(ugv_at ?v - ugv ?wp - ground)
	(uav_at ?u - uav ?wp - sky)
	(docked_at ?u - uav ?wp - ground)

	(docked ?u - uav)
    (flying ?u - uav)

    (observed ?u - ugv)

    (connected ?wp1 ?wp2 - waypoint)
    (can_observe ?wp1 - sky ?wp2 - ground)
)

(:functions
    (distance ?wp1 ?wp2 - waypoint)
)

;; Move between any two waypoints, avoiding terrain
(:durative-action flyto_waypoint
	:parameters (?v - uav ?from ?to - sky)
	;;:duration ( = ?duration (distance ?from ?to))
	:duration ( = ?duration 1)
	:condition (and
		(at start (uav_at ?v ?from))
        (at start (connected ?from ?to))
        )
	:effect (and
		(at start (not (uav_at ?v ?from)))
		(at end (uav_at ?v ?to)))
    )

(:durative-action observe_ugv
	:parameters (?v - uav ?obs_wp - sky ?wp - ground ?o - ugv)
	:duration ( = ?duration 1)
	:condition (and
		(at start (uav_at ?v ?obs_wp))
		(at start (ugv_at ?o ?wp))
		(at start (can_observe ?obs_wp ?wp))
        )
	:effect (and
		(at end (observed ?o))
        )
    )

(:durative-action take_off
    :parameters (?v - uav ?start_wp - ground ?sky_wp - sky)
	:duration (= ?duration 1)
    :condition (and
        (at start (can_observe ?sky_wp ?start_wp))
        (at start (docked_at ?v ?start_wp))
        (at start (docked ?v))
        )
    :effect(and
        (at start (not (docked_at ?v ?start_wp)))
        (at start (not (docked ?v)))
        (at end (uav_at ?v ?sky_wp))
        (at end (flying ?v))
        )
    )

(:durative-action land
    :parameters (?v - uav ?wp - ground ?sky_wp - sky)
	:duration (= ?duration 1)
    :condition (and
        (at start (can_observe ?sky_wp ?wp))
        (at start (flying ?v))
        (at start (uav_at ?v ?sky_wp))
        )
    :effect(and
        (at start (not (uav_at ?v ?sky_wp)))
        (at start (not (flying ?v)))
        (at end (docked_at ?v ?wp))
        (at end (docked ?v))
        )
    )
)
