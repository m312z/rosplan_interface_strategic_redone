Number of literals: 11
Constructing lookup tables: [10%] [20%] [30%] [40%] [50%] [60%] [70%] [80%] [90%] [100%] [110%]
Post filtering unreachable actions:  [10%] [20%] [30%] [40%] [50%] [60%] [70%] [80%] [90%] [100%] [110%]
[01;34mNo analytic limits found, not considering limit effects of goal-only operators[00m
All the ground actions in this problem are compression-safe
Initial heuristic = 3.000
b (2.000 | 6.000)b (1.000 | 16.001)
; Plan found with metric 16.001
; States evaluated so far: 4
; Time 0.00
0.000: (flyto_waypoint uav01 sky_wp1 sky_wp3)  [6.000]
6.001: (observe_ugv uav01 sky_wp3 ground_wp5)  [10.000]
6.001: (observe_ugv uav01 sky_wp3 ground_wp6)  [10.000]

 * All goal deadlines now no later than 16.001

Resorting to best-first search
b (2.000 | 6.000)b (1.000 | 16.001)
Problem Unsolvable
;;;; Solution Found
; States evaluated: 14
; Cost: 16.001
; Time 0.00
0.000: (flyto_waypoint uav01 sky_wp1 sky_wp3)  [6.000]
6.001: (observe_ugv uav01 sky_wp3 ground_wp5)  [10.000]
6.001: (observe_ugv uav01 sky_wp3 ground_wp6)  [10.000]
