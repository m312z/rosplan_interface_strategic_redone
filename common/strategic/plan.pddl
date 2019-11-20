Number of literals: 38
Constructing lookup tables: [10%] [20%] [30%] [40%] [50%] [60%] [70%] [80%] [90%] [100%]
Post filtering unreachable actions:  [10%] [20%] [30%] [40%] [50%] [60%] [70%] [80%] [90%] [100%]
[01;34mNo analytic limits found, not considering limit effects of goal-only operators[00m
73% of the ground temporal actions in this problem are compression-safe
Initial heuristic = 8.000
b (7.000 | 15.000)b (6.000 | 15.000)b (5.000 | 15.000)b (4.000 | 22.002)b (3.000 | 22.002)b (2.000 | 38.004)b (1.000 | 38.004);;;; Solution Found
; States evaluated: 11
; Cost: 48.005
; Time 0.00
0.000: (goto_waypoint0 ugv01 ground_wp1 ground_wp2)  [15.000]
0.000: (goto_waypoint1 ugv01 ground_wp5 ground_wp3)  [15.000]
0.000: (goto_waypoint2 ugv01 ground_wp6 ground_wp4)  [15.000]
0.000: (take_off uav01 dock1 ground_wp1 sky_wp1)  [6.000]
6.001: (complete_mission uav01 mission_0)  [16.001]
22.003: (complete_mission uav01 mission_1)  [16.001]
38.005: (land uav01 dock1 ground_wp1 sky_wp1)  [10.000]
