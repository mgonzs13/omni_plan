(define (domain easy_plan_domain)
(:requirements :typing :durative-actions :negative-preconditions)

 (:types
  robot
  room
)

(:predicates
  (battery_full ?r0 - robot)
  (battery_low ?r0 - robot)
  (charging_point_at ?r0 - room)
  (connected ?r0 - room ?r1 - room)
  (robot_at ?r0 - robot ?r1 - room)
)

(:durative-action charge
  :parameters (?robot - robot ?room - room)
  :duration (= ?duration 10)
  :condition (and
    (at start (robot_at ?robot ?room))
    (at start (charging_point_at ?room)))
  :effect (and
    (at end (battery_low ?robot))
    (at end (battery_full ?robot)))
)
(:durative-action askcharge
  :parameters (?robot - robot ?r1 - room ?r2 - room)
  :duration (= ?duration 10)
  :condition (and
    (at start (robot_at ?robot ?r1))
    (at start (charging_point_at ?r2)))
  :effect (and
    (at end (robot_at ?robot ?r1))
    (at end (robot_at ?robot ?r2)))
)
(:durative-action move
  :parameters (?robot - robot ?r1 - room ?r2 - room)
  :duration (= ?duration 10)
  :condition (and
    (at start (robot_at ?robot ?r1))
    (over all (battery_full ?robot))
    (at start (connected ?r1 ?r2)))
  :effect (and
    (at end (robot_at ?robot ?r1))
    (at end (robot_at ?robot ?r2)))
)
)