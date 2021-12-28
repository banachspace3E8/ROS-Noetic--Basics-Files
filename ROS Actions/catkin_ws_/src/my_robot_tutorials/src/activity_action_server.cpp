//Shishir Khanal
//Course: ROS Actions by Edouard Renard
//C++ ROS Action Server with ROS Noetic

#include <ros/ros.h>
#include <actionlib/server/action_server.h>
#include <actionlib/server/server_goal_handle.h>

#include <thread>
#include <vector>
#include <map>
#include <string>

#include <my_robot_msgs/CountUntilAction.h>
#include <my_robot_msgs/CountUntilGoal.h>
#include <my_robot_msgs/CountUntilResult.h>
#include <my_robot_msgs/CountUntilFeedback.h>


class CountUntilServer{

	protected:
	ros::NodeHandle _nh;
	actionlib::ActionServer<my_robot_msgs::CountUntilAction> _as;
	
	//pointer to standard thread to manage callbacks
	std::vector<std::thread*> _goalThreads;
	//Map Data structure to store goals
	std::map<std::string, bool> _cancelGoals;
	
	public:
	//Constructor => goal & cancel callback and auto start is set to false
	// _1 means we can pass 1 argument
	CountUntilServer():
	_as(_nh,"/count_until", boost::bind(&CountUntilServer::onGoal, this, _1), boost::bind(&CountUntilServer::onCancel, this,_1), false){
	
		_as.start();
		ROS_INFO("Action server has been started.");	
		}

	~CountUntilServer(){
		
		for(int i = 0; i  < _goalThreads.size(); i++){
			//make sure that the thread is finished and then delete the thread
			_goalThreads.at(i)->join();
			delete _goalThreads.at(i);
		}
	
	}

	void processGoal(actionlib::ServerGoalHandle<my_robot_msgs::CountUntilAction> gh){
	
		boost::shared_ptr<const my_robot_msgs::CountUntilGoal> goal = gh.getGoal();
		std::string goal_id = gh.getGoalID().id;
		int max_number = goal->max_number;
		double wait_duration = goal->wait_duration;
		ROS_INFO("Processing goal: %s", goal_id.c_str());
		ROS_INFO("Max number: %d, wait duration: %f",max_number, wait_duration);
		
		//Validate parameters
		if (max_number > 10){
			gh.setRejected();
			return;
		}
		gh.setAccepted();
		
		int counter = 0;
		ros::Rate rate(1.0/wait_duration);
		bool success = false;
		bool preempted = false;
		
		while (ros::ok()){
		
			counter++;
			ROS_INFO("%s: %d",goal_id.c_str(), counter);
			if(_cancelGoals[goal_id]){
				preempted = true;
				break;
			}
			if(counter >=max_number){
			
				success = true;
				break;
			}
			my_robot_msgs::CountUntilFeedback feedback;
			feedback.percentage = (double)counter/(double)max_number;
			gh.publishFeedback(feedback);
			rate.sleep();
		}
	
		my_robot_msgs::CountUntilResult result;
		result.count = counter;
		ROS_INFO("Send goal result to client: %s", goal_id.c_str());
		
		if(preempted){
			gh.setCanceled(result);
		}
		else if(success){
			gh.setSucceeded(result);
		}
		else{
			gh.setAborted(result);
		}
		
		_cancelGoals.erase(goal_id);
	}
	
	void onGoal(actionlib::ServerGoalHandle<my_robot_msgs::CountUntilAction> gh){
	
		ROS_INFO("Recieved new goal");
	
		//Uncomment and recompile (catkin_make) to apply goal policy change
		//if(_cancelGoals.size() != 0){
		//	my_robot_msgs::CountUntilResult result;
		//	result.count = -1000;
		//	gh.setRejected(result);
		//	return;
		//}
	
		_goalThreads.push_back(new std::thread(boost::bind(&CountUntilServer::processGoal, this, gh)));
		
		_cancelGoals[gh.getGoalID().id] = false;
	}
	
	void onCancel(actionlib::ServerGoalHandle<my_robot_msgs::CountUntilAction> gh){
	
		ROS_INFO("Cancel request has been recieved.");
		if (_cancelGoals.count(gh.getGoalID().id) > 0){
		
			ROS_INFO("Found goal to cancel");
			_cancelGoals[gh.getGoalID().id] = true;
		}	
	}
};
		
int main(int argc, char **argv){

	ros::init(argc,argv,"count_until_server");
	
	CountUntilServer server;
	ros::spin();
}
