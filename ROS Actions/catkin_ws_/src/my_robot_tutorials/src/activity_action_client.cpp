//Shishir Khanal
//Course: ROS Actions by Edouard Renard
//C++ ROS Action Client with ROS Noetic

#include <ros/ros.h>

#include <actionlib/client/action_client.h>
#include <actionlib/client/client_helpers.h>

#include <map>
#include <string>

#include <my_robot_msgs/CountUntilAction.h>
#include <my_robot_msgs/CountUntilGoal.h>

typedef actionlib::ClientGoalHandle<my_robot_msgs::CountUntilAction> GoalHandle;

class CountUntilClient{

protected:
	ros::NodeHandle _nh;
	actionlib::ActionClient<my_robot_msgs::CountUntilAction> _ac;

public:
	//create a pointer----<A1>
	std::map<int, GoalHandle*> _goalHandles;

	CountUntilClient(): _ac("/count_until"){

		_ac.waitForActionServerToStart();
		ROS_INFO("Actionserver is up, we can send goals");

	}

	void onTransition(const GoalHandle gh){

		int index = 0;
		//starts at the beginning and continues until we get to the end of _goalHandles
		std::map<int, GoalHandle*>::iterator it = _goalHandles.begin();
		while (it != _goalHandles.end()){
			if(*(it->second) == gh){
			index = it->first;
			break;
			}
			it++;
		}

		ROS_INFO("%d -- Transition callback",index);
		actionlib::CommState commState = gh.getCommState();

		ROS_INFO("%d: Comm state: %s", index, commState.toString().c_str());

		if(commState.toString() == "Active"){

			ROS_INFO("%d: Goal just went active", index);
		}
		else if(commState.toString() == "Done"){
			ROS_INFO("%d: Goal is DONE", index);
			actionlib::TerminalState state = gh.getTerminalState();
			ROS_INFO("Client Terminal State: %s", state.toString().c_str());
			my_robot_msgs::CountUntilResultConstPtr result = gh.getResult();
			ROS_INFO("Count result: %d", (int)result->count);
		}
	}

	void onFeedback(const GoalHandle gh, const my_robot_msgs::CountUntilFeedbackConstPtr &feedback){

		//ROS_INFO("--- Feedback callback")
		//ROS_INFO("percentage: %1f", feedback->percentage);

	}

	GoalHandle sendGoal(int max_number, double wait_duration){


		my_robot_msgs::CountUntilGoal goal;
		goal.max_number = max_number;
		goal.wait_duration = wait_duration;
		GoalHandle goal_handle = _ac.sendGoal(goal,boost::bind(&CountUntilClient::onTransition, this, _1),boost::bind(&CountUntilClient::onFeedback, this, _1,_2));
		ROS_INFO("Goal has been sent.");
		return goal_handle;
	}

};


int main (int argc, char **argv){


	ros::init(argc, argv, "count_until_client");
	//create ros spin on the background on a different thread
	ros::AsyncSpinner spinner(4);
	spinner.start();

	CountUntilClient client;
	GoalHandle goal_handle1 = client.sendGoal(8,0.5);
	//reference the address----<A2>
	client._goalHandles[1] = &goal_handle1;

	GoalHandle goal_handle2 = client.sendGoal(5,0.8);
	client._goalHandles[2] = &goal_handle2;

	ros::Duration(1.5).sleep();
	goal_handle1.cancel();

	//similar to ros::ok()
	ros::waitForShutdown();
}