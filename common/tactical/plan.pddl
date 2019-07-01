Warnings encountered when parsing Domain/Problem File

Errors: 0, warnings: 5
/home/q/ROSPlan/src/rosplan_interface_strategic/common/problem_tactical.pddl: line: 7: Warning: Re-declaration of symbol in same scope: black
/home/q/ROSPlan/src/rosplan_interface_strategic/common/problem_tactical.pddl: line: 7: Warning: Re-declaration of symbol in same scope: blue
/home/q/ROSPlan/src/rosplan_interface_strategic/common/problem_tactical.pddl: line: 7: Warning: Re-declaration of symbol in same scope: green
/home/q/ROSPlan/src/rosplan_interface_strategic/common/problem_tactical.pddl: line: 7: Warning: Re-declaration of symbol in same scope: red
/home/q/ROSPlan/src/rosplan_interface_strategic/common/problem_tactical.pddl: line: 7: Warning: Re-declaration of symbol in same scope: white
Number of literals: 30
Constructing lookup tables: [10%] [20%] [30%] [40%] [50%] [60%] [70%] [80%] [90%] [100%] [110%]
Post filtering unreachable actions:  [10%] [20%] [30%] [40%] [50%] [60%] [70%] [80%] [90%] [100%] [110%]
Have identified that bigger values of (carrying kenny black) are preferable
Have identified that bigger values of (carrying kenny blue) are preferable
Have identified that bigger values of (carrying kenny green) are preferable
Have identified that bigger values of (carrying kenny white) are preferable
Have identified that bigger values of (carrying kenny red) are preferable
[01;34mNo analytic limits found, not considering limit effects of goal-only operators[00m
Pruning (complete_mission kenny mission_2 wp0 wp1) - never appeared in initial RPG
91% of the ground temporal actions in this problem are compression-safe
Initial heuristic = 12.000
b (10.000 | 10.000)b (8.000 | 190.000)b (6.000 | 260.000)b (5.000 | 320.000)
Resorting to best-first search
b (10.000 | 10.000)b (8.000 | 190.000)b (6.000 | 260.000)b (5.000 | 320.000)b (3.000 | 496.037)b (2.000 | 556.037);;;; Solution Found
; States evaluated: 118
; Cost: 559.037
; Time 0.02
0.000: (collect_black kenny wp0)  [10.000]
10.000: (goto_waypoint kenny wp0 wp1)  [60.000]
70.000: (collect_blue kenny wp1)  [120.000]
190.000: (goto_waypoint kenny wp1 wp3)  [60.000]
250.000: (collect_white kenny wp3)  [10.000]
260.000: (goto_waypoint kenny wp3 wp0)  [60.000]
486.037: (collect_red kenny wp0)  [10.000]
496.037: (goto_waypoint kenny wp0 wp1)  [60.000]
556.037: (complete_building kenny i03 wp1)  [3.000]
