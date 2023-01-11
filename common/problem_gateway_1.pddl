(define (problem task)
(:domain gateway)
(:objects
    wp0 wp1 wp2 wp3 wp4 - waypoint
    bumble - robot
)
(:init
    (= (charge bumble) 0)
    (robot_at bumble wp0)
    (docked bumble)
    (dock_at wp0)
)
(:goal (and
    (visited wp0)
    (visited wp1)
    (visited wp2)
    (visited wp3)
    (visited wp4)
    (docked bumble)
    (> (charge bumble) 0)
)))
