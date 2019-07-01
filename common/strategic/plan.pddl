Warnings encountered when parsing Domain/Problem File

Errors: 0, warnings: 5
/home/q/ROSPlan/src/rosplan_interface_strategic/common/problem_strategic.pddl: line: 7: Warning: Re-declaration of symbol in same scope: black
/home/q/ROSPlan/src/rosplan_interface_strategic/common/problem_strategic.pddl: line: 7: Warning: Re-declaration of symbol in same scope: blue
/home/q/ROSPlan/src/rosplan_interface_strategic/common/problem_strategic.pddl: line: 7: Warning: Re-declaration of symbol in same scope: green
/home/q/ROSPlan/src/rosplan_interface_strategic/common/problem_strategic.pddl: line: 7: Warning: Re-declaration of symbol in same scope: red
/home/q/ROSPlan/src/rosplan_interface_strategic/common/problem_strategic.pddl: line: 7: Warning: Re-declaration of symbol in same scope: white
Number of literals: 35
Constructing lookup tables: [10%] [20%] [30%] [40%] [50%] [60%] [70%] [80%] [90%] [100%] [110%] [120%]
Post filtering unreachable actions:  [10%] [20%] [30%] [40%] [50%] [60%] [70%] [80%] [90%] [100%] [110%] [120%]
Have identified that bigger values of (carrying kenny black) are preferable
Have identified that bigger values of (carrying kenny blue) are preferable
Have identified that bigger values of (carrying kenny green) are preferable
Have identified that bigger values of (carrying kenny white) are preferable
Have identified that bigger values of (carrying kenny red) are preferable
[01;34mNo analytic limits found, not considering limit effects of goal-only operators[00m
81% of the ground temporal actions in this problem are compression-safe
Initial heuristic = 9.000
b (8.000 | 5.000)b (6.000 | 65.001)b (5.000 | 735.653)b (4.000 | 795.654)b (3.000 | 1043.659)b (2.000 | 1103.660)b (1.000 | 1768.162);;;; Solution Found
; States evaluated: 12
; Cost: 1768.162
; Time 0.00
0.000: (undock kenny wp0)  [5.000]
5.001: (localise kenny)  [60.000]
65.002: (complete_mission kenny mission_0 wp0 wp1)  [670.651]
735.654: (goto_waypoint kenny wp1 wp0)  [60.000]
795.655: (complete_mission kenny mission_1 wp0 wp1)  [248.004]
1043.660: (goto_waypoint kenny wp1 wp0)  [60.000]
1103.661: (complete_mission kenny mission_2 wp0 wp1)  [664.501]
