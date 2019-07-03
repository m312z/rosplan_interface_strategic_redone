(define (domain redone_ugv)

(:requirements :strips :typing :fluents :disjunctive-preconditions :durative-actions :equality :duration-inequalities :timed-initial-literals)

(:types
	ground sky - waypoint 
	uav ugv - robot
)

(:predicates
	(ugv_at ?v - ugv ?wp - ground)
    (connected ?wp1 ?wp2 - waypoint)
	(visited ?wp - waypoint)
)

(:functions
    (distance ?wp1 ?wp2 - waypoint)
)

;; Move between any two waypoints, avoiding terrain
(:durative-action goto_waypoint
	:parameters (?v - ugv ?from ?to - ground)
	;;:duration ( = ?duration (distance ?from ?to))
	:duration ( = ?duration 1)
	:condition (and
		(at start (ugv_at ?v ?from))
        (at start (connected ?from ?to))
        )
	:effect (and
		(at start (not (ugv_at ?v ?from)))
		(at end (visited ?to))
		(at end (ugv_at ?v ?to)))
    )
)
