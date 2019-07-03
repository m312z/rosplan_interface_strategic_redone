(define (problem task)
(:domain turtlebot)
(:objects
    sky_wp0 sky_wp9 - sky
    ground_wp1 ground_wp2 ground_wp3 ground_wp4 ground_wp5 ground_wp6 ground_wp7 ground_wp8 - ground
    ugv01 ugv02 ugv03 - ugv
    uav01 - uav
)
(:init
    (ugv_at ugv01 ground_wp1)
    (ugv_at ugv02 ground_wp5)
    (ugv_at ugv03 ground_wp8)

    (docked_at uav01 ground_wp1)
    (docked uav01)

    (can_observe sky_wp0 ground_wp1)
    (can_observe sky_wp0 ground_wp2)
    (can_observe sky_wp0 ground_wp3)
    (can_observe sky_wp0 ground_wp4)
    (can_observe sky_wp0 ground_wp5)

    (can_observe sky_wp9 ground_wp4)
    (can_observe sky_wp9 ground_wp5)
    (can_observe sky_wp9 ground_wp6)
    (can_observe sky_wp9 ground_wp7)
    (can_observe sky_wp9 ground_wp8)

    (connected ground_wp1 ground_wp2)
    (connected ground_wp2 ground_wp1)
    (connected ground_wp2 ground_wp3)
    (connected ground_wp3 ground_wp2)
    (connected ground_wp3 ground_wp4)
    (connected ground_wp4 ground_wp3)
    (connected ground_wp4 ground_wp5)
    (connected ground_wp5 ground_wp4)
    (connected ground_wp5 ground_wp6)
    (connected ground_wp6 ground_wp5)
    (connected ground_wp3 ground_wp4)
    (connected ground_wp4 ground_wp3)
    (connected ground_wp4 ground_wp5)
    (connected ground_wp5 ground_wp4)
    (connected ground_wp5 ground_wp6)
    (connected ground_wp6 ground_wp5)
    (connected ground_wp6 ground_wp7)
    (connected ground_wp7 ground_wp6)
    (connected ground_wp7 ground_wp8)
    (connected ground_wp8 ground_wp7)
    (connected sky_wp0 ground_wp3)
    (connected ground_wp3 sky_wp0)
    (connected ground_wp3 ground_wp4)
    (connected ground_wp4 ground_wp3)

    (connected sky_wp0 sky_wp9)
    (connected sky_wp9 sky_wp0)
)
(:goal (and
    (observed ugv01)
    (observed ugv02)
    (observed ugv03)
    (docked uav01)
)))
