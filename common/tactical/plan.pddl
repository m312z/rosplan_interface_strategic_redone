Number of literals: 11
Constructing lookup tables: [10%] [20%] [30%] [40%] [50%] [60%] [70%] [80%] [90%] [100%] [110%]
Post filtering unreachable actions:  [10%] [20%] [30%] [40%] [50%] [60%] [70%] [80%] [90%] [100%] [110%]
[01;34mNo analytic limits found, not considering limit effects of goal-only operators[00m
All the ground actions in this problem are compression-safe
Initial heuristic = 2.000
b (1.000 | 6.000)
; Plan found with metric 16.001
; States evaluated so far: 3
; Time 0.00
0.000: (flyto_waypoint uav01 sky_wp0 sky_wp1)  [6.000]
6.001: (observe_ugv uav01 sky_wp1 ground_wp1)  [10.000]

 * All goal deadlines now no later than 16.001

Resorting to best-first search
b (1.000 | 6.000)
Problem Unsolvable
;;;; Solution Found
; States evaluated: 12
; Cost: 16.001
; Time 0.00
0.000: (flyto_waypoint uav01 sky_wp0 sky_wp1)  [6.000]
6.001: (observe_ugv uav01 sky_wp1 ground_wp1)  [10.000]
