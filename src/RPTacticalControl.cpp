#include "rosplan_interface_strategic/RPTacticalControl.h"

/* The implementation of RPTacticalControl.h */
namespace KCL_rosplan {

	/* constructor */
	RPTacticalControl::RPTacticalControl(ros::NodeHandle &nh) {

		// planning interface
		std::string goalTopic = "/rosplan_interface_strategic_control/get_mission_goals";
		std::string cancTopic = "/rosplan_plan_dispatcher/cancel_dispatch";
		std::string probTopic = "/rosplan_problem_interface/problem_generation_server";
		std::string planTopic = "/rosplan_planner_interface/planning_server";
		std::string parsTopic = "/rosplan_parsing_interface/parse_plan";
		std::string dispTopic = "/rosplan_plan_dispatch/dispatch_plan";

		nh.getParam("mission_goals_topic", goalTopic);
		nh.getParam("cancel_service_topic", cancTopic);
		nh.getParam("problem_service_topic", probTopic);
		nh.getParam("planning_service_topic", planTopic);
		nh.getParam("parsing_service_topic", parsTopic);
		nh.getParam("dispatch_service_topic", dispTopic);

		mission_goals_client = nh.serviceClient<rosplan_knowledge_msgs::GetAttributeService>(goalTopic);
		cancel_client = nh.serviceClient<std_srvs::Empty>(cancTopic);
		problem_client = nh.serviceClient<std_srvs::Empty>(probTopic);
		planning_client = nh.serviceClient<std_srvs::Empty>(planTopic);
		parsing_client = nh.serviceClient<std_srvs::Empty>(parsTopic);
		dispatch_client = nh.serviceClient<rosplan_dispatch_msgs::DispatchService>(dispTopic);

        // knowledge interface
		std::string knowTopic = "rosplan_knowledge_base";
		nh.getParam("tactical_knowledge_base", knowTopic);

        std::stringstream ss;
        ss << knowTopic << "/state/goals";
		current_goals_client = nh.serviceClient<rosplan_knowledge_msgs::GetAttributeService>(ss.str());

		ss.str("");
		ss << knowTopic << "/state/propositions";
		current_props_client = nh.serviceClient<rosplan_knowledge_msgs::GetAttributeService>(ss.str());

        ss.str("");
        ss << knowTopic << "/update";
		local_update_knowledge_client = nh.serviceClient<rosplan_knowledge_msgs::KnowledgeUpdateService>(ss.str());

		std::string status_topic = "status";
		status_publisher = nh.advertise<std_msgs::String>(status_topic, 1000);
	}

	/**
	 * fetch goals for corresponding mission and update KB
	 */
	bool RPTacticalControl::initGoals(const std::string &mission, const std::string &uav_name) {

		mission_goals.clear();
		old_goals.clear();

		// fetch mission goals
		rosplan_knowledge_msgs::GetAttributeService gsrv;
		gsrv.request.predicate_name = mission;
		if(!mission_goals_client.call(gsrv)) {
			ROS_ERROR("KCL: (%s) Failed to call Strategic Interface for goals: %s.", ros::this_node::getName().c_str(), mission_goals_client.getService().c_str());
			return false;
		} else {
			mission_goals = gsrv.response.attributes;
		}

		// fetch and store old goals
		rosplan_knowledge_msgs::GetAttributeService currentGoalSrv;
		if (!current_goals_client.call(currentGoalSrv)) {
			ROS_ERROR("KCL: (%s) Failed to call Knowledge Base for goals: %s.", ros::this_node::getName().c_str(), current_goals_client.getService().c_str());
			return false;
		} else {
			old_goals = currentGoalSrv.response.attributes;
		}

		// fetch final goal position for mission
		std::string mission_goal_location = "";
		rosplan_knowledge_msgs::GetAttributeService currentPropsSrv;
		currentPropsSrv.request.predicate_name = "mission_complete_waypoint";		
		if (!current_props_client.call(currentPropsSrv)) {
			ROS_ERROR("KCL: (%s) Failed to call Knowledge Base for propositions: %s.", ros::this_node::getName().c_str(), current_goals_client.getService().c_str());
			return false;
		} else {
			std::vector<rosplan_knowledge_msgs::KnowledgeItem>::iterator kit = currentPropsSrv.response.attributes.begin();
			for(; kit!=currentPropsSrv.response.attributes.end(); kit++) {
				// (mission_complete_waypoint ?m - mission ?wp - sky)
				if(kit->values.size()<2) continue; // malformed proposition
				if(kit->values[0].value == mission) {
					mission_goal_location = kit->values[1].value;
					break;
				}
			}
		}

		// clear old goals
		rosplan_knowledge_msgs::KnowledgeUpdateService updateSrv;
		updateSrv.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::REMOVE_GOAL;
		updateSrv.request.knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
		updateSrv.request.knowledge.attribute_name = "";
		updateSrv.request.knowledge.values.clear();
		local_update_knowledge_client.call(updateSrv);

		// add mission goal to knowledge base
		rosplan_knowledge_msgs::KnowledgeUpdateService updateGoalSrv;
		updateGoalSrv.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_GOAL;
		updateGoalSrv.request.knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
		for(int i = 0; i<mission_goals.size(); i++) {
			updateGoalSrv.request.knowledge = mission_goals[i];
			if(!local_update_knowledge_client.call(updateGoalSrv)) {
				ROS_INFO("KCL: (%s) failed to update PDDL goal.", ros::this_node::getName().c_str());
				restoreGoals();
				return false;
			}
		}

		if(mission_goal_location != "") {
			// add final goal position as a tactical goal
			rosplan_knowledge_msgs::KnowledgeItem wpgoal;
			wpgoal.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
			wpgoal.attribute_name = "uav_at";
			diagnostic_msgs::KeyValue pair;
			pair.key = "u";
			pair.value = uav_name;
			wpgoal.values.push_back(pair);
			pair.key = "wp";
			pair.value = mission_goal_location;
			wpgoal.values.push_back(pair);
			updateGoalSrv.request.knowledge = wpgoal;
			if(!local_update_knowledge_client.call(updateGoalSrv)) {
				ROS_INFO("KCL: (%s) failed to update PDDL goal.", ros::this_node::getName().c_str());
				restoreGoals();
				return false;
			}
		}

		return true;

	}

	/**
	 * remove mission goals from KB and restore saved goals
	 */
	void RPTacticalControl::restoreGoals() {

		ROS_INFO("restoreGoals ##### %d" , __LINE__);

		// remove mission goal from KB
		rosplan_knowledge_msgs::KnowledgeUpdateService updateGoalSrv;
		updateGoalSrv.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::REMOVE_GOAL;
		for(int i = 0; i<mission_goals.size(); i++) {
			ROS_INFO("restoreGoals ##### %d" , __LINE__);
			updateGoalSrv.request.knowledge = mission_goals[i];
			if(!local_update_knowledge_client.call(updateGoalSrv)) {
				ROS_INFO("KCL: (%s) failed to update PDDL goal.", ros::this_node::getName().c_str());
			}
		}

		ROS_INFO("restoreGoals ##### %d" , __LINE__);

		// add old goal to knowledge base
		updateGoalSrv.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_GOAL;
		updateGoalSrv.request.knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
		for(int i = 0; i<old_goals.size(); i++) {
			updateGoalSrv.request.knowledge = old_goals[i];
			if(!local_update_knowledge_client.call(updateGoalSrv)) {
				ROS_INFO("KCL: (%s) failed to update PDDL goal.", ros::this_node::getName().c_str());
			}
		}
	}

	/**
	 * monitor mission goals in case of human intervention
	 */
	void RPTacticalControl::monitorGoals() {

        ros::Rate loop_rate(1);

        try {
            while(ros::ok()) {

                mutex.lock();

		        // fetch goals from the KB
		        rosplan_knowledge_msgs::GetAttributeService currentGoalSrv;
		        if (!current_goals_client.call(currentGoalSrv)) {
			        ROS_ERROR("KCL: (%s) Failed to call Knowledge Base for goals: %s.", ros::this_node::getName().c_str(), current_goals_client.getService().c_str());
                    mutex.unlock();
			        return;
		        }

                if(mission_goals.size() != currentGoalSrv.response.attributes.size()) {
                    // cancel any ongoing dispatch
                	ROS_INFO("KCL: (%s) aborting tactical plan dispatch; goals changed.", params.name.c_str());
                    std_srvs::Empty empty;
                    cancel_client.call(empty);
                    ros::spinOnce();

                    mission_goals = currentGoalSrv.response.attributes;
                }

                mutex.unlock();

                // check for interruption and sleep
                loop_rate.sleep();       
                boost::this_thread::interruption_point(); 
            }
        } catch(boost::thread_interrupted& ex) {
            ROS_INFO("KCL: (%s) Ending goal monitor.", params.name.c_str());
            mutex.unlock();
            return;
        }
	}

	/* action dispatch callback */
	bool RPTacticalControl::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {

		std_msgs::String status_msg;
		status_msg.data = "on_mission";
		status_publisher.publish(status_msg);
		ros::spinOnce();

		// get mission ID from action dispatch complete_mission (?r - robot ?m - mission ?wp - waypoint)
		std::string mission;
		std::string uav_name;
		bool found_mission = false;
		bool found_uav_name = false;
		for(size_t i=0; i<msg->parameters.size(); i++) {
			if(0==msg->parameters[i].key.compare("m")) {
				mission = msg->parameters[i].value;
				found_mission = true;
			}
			if(0==msg->parameters[i].key.compare("u")) {
				uav_name = msg->parameters[i].value;
				found_uav_name = true;
			}
		}
		if(!found_mission || !found_uav_name) {
			ROS_INFO("KCL: (%s) aborting action dispatch; PDDL action missing required parameter ?u ?m", params.name.c_str());

			status_msg.data = "off_mission";
			status_publisher.publish(status_msg);
			ros::spinOnce();

			return false;
		}

		if(!initGoals(mission, uav_name)) {
			status_msg.data = "off_mission";
			status_publisher.publish(status_msg);
			ros::spinOnce();
			return false;
		}

		// generate problem and plan
		ROS_INFO("KCL: (%s) Starting tactical replanning loop.", ros::this_node::getName().c_str());

        // set up monitor for new goals
        boost::thread goalMonitorThread(boost::bind(&RPTacticalControl::monitorGoals, this));


        bool dispatch_success = false;
		std_srvs::Empty empty;
        while(ros::ok() && !dispatch_success && !action_cancelled) {

		    rosplan_dispatch_msgs::DispatchService dispatch;

            // cancel any existing dispatch (TODO check why)
            mutex.lock();
		    cancel_client.call(empty);
		    ros::Duration(1).sleep(); // sleep for a second
            mutex.unlock();

            // generate the problem
		    problem_client.call(empty);
		    ros::Duration(1).sleep(); // sleep for a second


		    // send to planner
    		ROS_INFO("KCL: (%s) Starting tactical planner.", ros::this_node::getName().c_str());
		    if(planning_client.call(empty)) {
			    ros::Duration(1).sleep(); // sleep for a second

			    // parse planner output
			    parsing_client.call(empty);
			    ros::Duration(1).sleep(); // sleep for a second

			    // dispatch tactical plan
        		ROS_INFO("KCL: (%s) Starting tactical dispatcher.", ros::this_node::getName().c_str());
                dispatch_client.call(dispatch);
			    dispatch_success = dispatch.response.goal_achieved;
		    }
        }

        goalMonitorThread.interrupt();
        goalMonitorThread.join();

        ROS_INFO("KCL: (%s) Finishing strategic action by clearing tactical goals.", ros::this_node::getName().c_str());
        mutex.lock();
		restoreGoals();
        mutex.unlock();

        ROS_INFO("KCL: (%s) Finished.", ros::this_node::getName().c_str());

		status_msg.data = "off_mission";
		status_publisher.publish(status_msg);
		ros::spinOnce();

		return dispatch_success;
	}
} // close namespace

/*-------------*/
/* Main method */
/*-------------*/

int main(int argc, char **argv) {

	ros::init(argc, argv, "rosplan_interface_tactical_control");
	ros::NodeHandle nh("~");

	// create PDDL action subscriber
	KCL_rosplan::RPTacticalControl rptc(nh);

	rptc.runActionInterface();

	return 0;
}
