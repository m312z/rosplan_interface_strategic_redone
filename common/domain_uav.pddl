(define (domain redone_tactical)

(:requirements :strips :typing :fluents :disjunctive-preconditions :durative-actions :equality :duration-inequalities :timed-initial-literals)

(:types
	ground sky - waypoint
	uav - robot
)

(:predicates

    ;; statics (in both)
    (connected ?wp1 ?wp2 - waypoint)
    (can_observe ?wp1 - sky ?wp2 - ground)

    ;; tactical domain only
    (observed ?wp - ground)
	(uav_at ?u - uav ?wp - sky)
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

)
