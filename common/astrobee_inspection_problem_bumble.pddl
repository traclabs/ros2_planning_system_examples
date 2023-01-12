(define (problem task)
(:domain astrobee_inspection)
(:objects
    bumble - robot
    wp0 wp1 wp2 wp3 wp4 wp5 bumble_dock - location
)
(:init
    (robot_available bumble)
    (robot_at bumble bumble_dock)

)
(:goal (and
    (location_inspected wp0)
    (location_inspected wp1)
    (location_inspected wp2)
    (location_inspected wp3)
    (location_inspected wp4)
    (location_inspected wp5)
))
)
