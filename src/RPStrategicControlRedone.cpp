#include "rosplan_interface_strategic/RPStrategicControlRedone.h"
#include <iostream>

/* The implementation of RPStrategicControl.h specific to the REDONE scenario */
namespace KCL_rosplan {

	/* constructor */
	RPStrategicControl::RPStrategicControl(ros::NodeHandle &nh) {

		node_handle = &nh;

		// planning interface
		std::string probTopic = "/rosplan_problem_interface/problem_generation_server";
		std::string planTopic = "/rosplan_planner_interface/planning_server";
		std::string parsTopic = "/rosplan_parsing_interface/parse_plan";
		std::string knowTopic = "/rosplan_knowledge_base";

		// knowledge interface
		node_handle->getParam("tactical_knowledge_base", knowTopic);
        std::stringstream ss;
        ss << knowTopic << "/update";
		tactical_update_knowledge_client = nh.serviceClient<rosplan_knowledge_msgs::KnowledgeUpdateService>(ss.str());
        ss.str("");
        ss << knowTopic << "/state/goals";
		tactical_current_goals_client = nh.serviceClient<rosplan_knowledge_msgs::GetAttributeService>(ss.str());

		// knowledge interface
		node_handle->getParam("strategic_knowledge_base", knowTopic);
        ss.str("");
        ss << knowTopic << "/update";
		strategic_update_knowledge_client = nh.serviceClient<rosplan_knowledge_msgs::KnowledgeUpdateService>(ss.str());
        ss.str("");
        ss << knowTopic << "/state/goals";
		strategic_current_goals_client = nh.serviceClient<rosplan_knowledge_msgs::GetAttributeService>(ss.str());

        // services
		node_handle->getParam("problem_service_topic", probTopic);
		node_handle->getParam("planning_service_topic", planTopic);
		node_handle->getParam("parsing_service_topic", parsTopic);

		problem_client = nh.serviceClient<std_srvs::Empty>(probTopic);
		planning_client = nh.serviceClient<std_srvs::Empty>(planTopic);
		parsing_client = nh.serviceClient<std_srvs::Empty>(parsTopic);
	}

	void RPStrategicControl::planCallback(const rosplan_dispatch_msgs::EsterelPlan& msg) {
		last_plan = msg;
		new_plan_recieved = true;
	}

	/*----------*/
	/* missions */
	/*----------*/

	/**
	 * add goals to form a new mission
	 * in future, could redo the decomposition here
	 */
	bool RPStrategicControl::addMissionGoals(rosplan_knowledge_msgs::KnowledgeUpdateServiceArray::Request &req, rosplan_knowledge_msgs::KnowledgeUpdateServiceArray::Response &res) {

		// new mission name
		std::stringstream ss;
        ss <<"mission_" << missions.size();

		// new mission
		ROS_INFO("KCL: (%s) Planning tactical mission: %s", ros::this_node::getName().c_str(), ss.str().c_str());
		planMission(req.knowledge, ss.str());
			
		// add new mission to KB
		ROS_INFO("KCL: (%s) Adding new mission goals.", ros::this_node::getName().c_str(), ss.str().c_str());
		storeMission(ss.str());

		return true;
	}

	/**
	 * get goals for particular mission
	 */
	bool RPStrategicControl::getMissionGoals(rosplan_knowledge_msgs::GetAttributeService::Request &req, rosplan_knowledge_msgs::GetAttributeService::Response &res) {
		std::string mission_name = req.predicate_name;
		if(missions.find(mission_name)!=missions.end()) {
			res.attributes = missions.find(mission_name)->second;
		}
		return true;
	}

	/**
	 * mission generation service method
	 */
	bool RPStrategicControl::decomposeProblem(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {

        goals.clear();

        // TODO clustering of UGV locations to form missions
        // TODO mission alternatives (requires domain support)

        // Creating tactical goal sets
  //       std::vector<std::vector<rosplan_knowledge_msgs::KnowledgeItem>> clusters;
  //       std::vector<rosplan_knowledge_msgs::KnowledgeItem> c1;
  //       std::vector<rosplan_knowledge_msgs::KnowledgeItem> c2;
  //       std::vector<rosplan_knowledge_msgs::KnowledgeItem> other_goals;


		// rosplan_knowledge_msgs::KnowledgeItem goal;
		// goal.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
		// goal.attribute_name = "observed";
		// goal.values.clear();
		// diagnostic_msgs::KeyValue pair_mission;
		// pair_mission.key = "wp";
		// pair_mission.value = "ground_wp1";
  //       goal.values.push_back(pair_mission);
  //       c1.push_back(goal);

		// goal.values.clear();
		// pair_mission.key = "wp";
		// pair_mission.value = "ground_wp5";
  //       goal.values.push_back(pair_mission);
  //       c2.push_back(goal);

		// goal.values.clear();
		// pair_mission.key = "wp";
		// pair_mission.value = "ground_wp6";
  //       goal.values.push_back(pair_mission);
  //       c2.push_back(goal);

  //       clusters.push_back(c1);
  //       clusters.push_back(c2);

		// std::vector<std::vector<rosplan_knowledge_msgs::KnowledgeItem>>::iterator cit = clusters.begin();
		// for(; cit!=clusters.end(); cit++) {

		// 	std::stringstream ss;
  //       	ss <<"mission_" << missions.size();
			
		// 	ROS_INFO("KCL: (%s) Planning tactical missions.", ros::this_node::getName().c_str());
		// 	planMission(*cit, ss.str());
			
		// 	ROS_INFO("KCL: (%s) Adding new mission goals.", ros::this_node::getName().c_str());
		// 	storeMission(ss.str());
		// }
		return true;
	}

	void RPStrategicControl::planMission(const std::vector<rosplan_knowledge_msgs::KnowledgeItem> &cluster, const std::string &mission_name) {

		// clear goals at start
        rosplan_knowledge_msgs::KnowledgeUpdateService updateSrv;
	    updateSrv.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::REMOVE_GOAL;
	    updateSrv.request.knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
	    updateSrv.request.knowledge.attribute_name = "";
	    updateSrv.request.knowledge.values.clear();
	    tactical_update_knowledge_client.call(updateSrv);

        // insert goals for tactical mission
		std::vector<rosplan_knowledge_msgs::KnowledgeItem>::const_iterator git = cluster.begin();
		for(; git!=cluster.end(); git++) {
	        updateSrv.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_GOAL;
	        updateSrv.request.knowledge = *git;
	        tactical_update_knowledge_client.call(updateSrv);
        }

	    // generate tactical problem and plan
	    ROS_INFO("KCL: (%s) Generating plan for %s.", ros::this_node::getName().c_str(), mission_name.c_str());
	    new_plan_recieved = false;

	    std_srvs::Empty empty;

	    problem_client.call(empty);
	    ros::Duration(0.5).sleep(); // sleep for a second
	    planning_client.call(empty);
	    ros::Duration(0.5).sleep(); // sleep for a second
	    parsing_client.call(empty);
	    ros::Duration(0.5).sleep(); // sleep for a second

	    while(!new_plan_recieved && ros::ok()) ros::spinOnce();

	    ROS_INFO("KCL: (%s) Recieved plan for %s.", ros::this_node::getName().c_str(), mission_name.c_str());

	    // compute plan duration and final position
	    double max_time = 0;
        std::string move_action = "flyto_waypoint";
        std::string final_position = "sky_wp0";
	    std::vector<rosplan_dispatch_msgs::EsterelPlanNode>::iterator nit = last_plan.nodes.begin();
	    for(; nit != last_plan.nodes.end(); nit++) {
		    double time = nit->action.dispatch_time + nit->action.duration;
		    if(time > max_time) max_time = time;
		    if(nit->action.name == move_action) final_position = nit->action.parameters[2].value;
	    }

		mission_completed_locations[mission_name] = final_position;
	    mission_durations[mission_name] = max_time;
	    missions[mission_name];
	    missions[mission_name] = cluster;

	    // clear goals again
	    updateSrv.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::REMOVE_GOAL;
	    updateSrv.request.knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
	    updateSrv.request.knowledge.attribute_name = "";
	    updateSrv.request.knowledge.values.clear();
	    tactical_update_knowledge_client.call(updateSrv);
	}

	void RPStrategicControl::storeMission(const std::string &mission_name) {

		std::map< std::string, std::vector<rosplan_knowledge_msgs::KnowledgeItem>>::iterator mit = missions.find(mission_name);
		if(mit == missions.end()) {
		   	ROS_WARN("KCL: (%s) Tried to store a mission that doesn't exist! %s.", ros::this_node::getName().c_str(), mission_name.c_str());
		   	return;
		}

		// mission instance
        rosplan_knowledge_msgs::KnowledgeUpdateService updateSrv;
		updateSrv.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
		updateSrv.request.knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::INSTANCE;
		updateSrv.request.knowledge.instance_type = "mission";
		updateSrv.request.knowledge.instance_name = mission_name;
		updateSrv.request.knowledge.values.clear();
		strategic_update_knowledge_client.call(updateSrv);

		// mission duration
		updateSrv.request.knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FUNCTION;
		updateSrv.request.knowledge.attribute_name = "mission_duration";
		updateSrv.request.knowledge.values.clear();
		diagnostic_msgs::KeyValue pair_mission;
		pair_mission.key = "m";
		pair_mission.value = mission_name;
		updateSrv.request.knowledge.values.push_back(pair_mission);
		updateSrv.request.knowledge.function_value = mission_durations[mission_name];
		strategic_update_knowledge_client.call(updateSrv);

		// mission final position
		updateSrv.request.knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
		updateSrv.request.knowledge.attribute_name = "mission_complete_waypoint";
		updateSrv.request.knowledge.values.clear();
		pair_mission.key = "m";
		pair_mission.value = mission_name;
		updateSrv.request.knowledge.values.push_back(pair_mission);
		pair_mission.key = "wp";
		pair_mission.value = mission_completed_locations[mission_name];
		updateSrv.request.knowledge.values.push_back(pair_mission);
		strategic_update_knowledge_client.call(updateSrv);

		// mission goal
		updateSrv.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_GOAL;
		updateSrv.request.knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
		updateSrv.request.knowledge.attribute_name = "mission_complete";
		updateSrv.request.knowledge.values.clear();
		pair_mission.key = "m";
		pair_mission.value = mission_name;
		updateSrv.request.knowledge.values.push_back(pair_mission);
		strategic_update_knowledge_client.call(updateSrv);
	}

} // close namespace

	/*-------------*/
	/* Main method */
	/*-------------*/

	int main(int argc, char **argv) {

		ros::init(argc, argv, "rosplan_interface_strategic_control");
		ros::NodeHandle nh("~");

		// params
		std::string lps_topic;
		nh.param("complete_plan_topic", lps_topic, std::string("/rosplan_plan_dispatcher/dispatch_plan"));

		// create bidder
		KCL_rosplan::RPStrategicControl rpsc(nh);

		// listen
		ros::ServiceServer service1 = nh.advertiseService("decompose_problem", &KCL_rosplan::RPStrategicControl::decomposeProblem, &rpsc);
		ros::ServiceServer service2 = nh.advertiseService("get_mission_goals", &KCL_rosplan::RPStrategicControl::getMissionGoals, &rpsc);
		ros::ServiceServer service3 = nh.advertiseService("add_mission_goal", &KCL_rosplan::RPStrategicControl::addMissionGoals, &rpsc);

		ros::Subscriber lps = nh.subscribe(lps_topic, 100, &KCL_rosplan::RPStrategicControl::planCallback, &rpsc);

		ROS_INFO("KCL: (%s) Ready to receive", ros::this_node::getName().c_str());

		ros::spin();

		return 0;
	}
