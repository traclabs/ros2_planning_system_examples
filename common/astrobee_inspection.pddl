(define (domain astrobee_inspection)
(:requirements :strips :typing :adl :fluents :durative-actions)

;; Types ;;;;;;;;;;;;;;;;;;;;;;;;;;;
(:types
robot
location
)

;; Predicates ;;;;;;;;;;;;;;;;;;;;;;;;
(:predicates

(robot_available ?r - robot)
(location_inspected ?loc - location)
(robot_at ?r - robot ?loc - location)

)




;; Functions ;;;;;;;;;;;;;;;;;;;;;;;;;;
(:functions
)

;; Actions ;;;;;;;;;;;;;;;;;;;;;;;;;;;;


(:durative-action move_to_inspect
  :parameters (?r - robot ?start_loc ?goal_loc - location)
  :duration ( = ?duration 11)
  :condition (and
      (at start(robot_at ?r ?start_loc))
      (at start(robot_available ?r))
  )
  :effect (and
   	  (at start(not(robot_available ?r)))
	  (at start(not(robot_at ?r ?start_loc)))
	  (at end(robot_available ?r))
	  (at end(robot_at ?r ?goal_loc))
          (at end(location_inspected ?goal_loc))
  )
)




);; end of domain astrobee_inspection ;;;;;;;;;;;;
