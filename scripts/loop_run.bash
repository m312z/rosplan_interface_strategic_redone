killall -9 xdot;
python graph_saver.py & xdot strategic_plan.dot & xdot tactical_plan.dot &
while true; do
	rosrun rosplan_interface_strategic strategic_init.bash;
	rosrun rosplan_interface_strategic strategic_goal.bash;
	rosrun rosplan_interface_strategic strategic_plan.bash;
	rosservice call /rosplan_knowledge_base/clear;
done
