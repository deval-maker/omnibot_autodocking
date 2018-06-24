#include "track_model.h"

#include <ros/ros.h>
#include <gazebo_msgs/GetModelState.h>
#include <string.h>
#include <geometry_msgs/Pose.h>

track_model::track_model(std::string goal_model, std::string tracker_model, ros::NodeHandle* nodeH){

	this->node = nodeH;
	this->gazebo_model_state_client = this->node->serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");;

	this->goal_model = goal_model;

	this->tracker_model = tracker_model;

	is_tracked = false;

	this->check_tracking();

	//	set tracking_thresholds;
	//
	//	set velocity_upper_thresholds;
	//	set velocity_lower_thresholds;
	//
	//	set vel_to_tracker;
	//  set zero velo to tracker

}

track_model::~track_model(){
	// send zero velocities

}

track_model_errors_e track_model::get_goal_position(){

	track_model_errors_e status = TRACK_MODEL_SUCCESS;

	if(TRACK_MODEL_SUCCESS == status){
		status = this->get_position("table", &this->tracker_position);
	}

	if(TRACK_MODEL_SUCCESS == status)
	{
		ROS_INFO("Goal Pos %f, %f", this->tracker_position.position.x,
				this->tracker_position.position.y);
	}
	else
	{
		ROS_ERROR("Failed to get the goal position.");
	}

	return status;
}

track_model_errors_e track_model::get_tracker_position(){

	track_model_errors_e status = TRACK_MODEL_SUCCESS;

	if(TRACK_MODEL_SUCCESS == status){
		status = this->get_position("omnibot", &this->tracker_position);
	}

	if(TRACK_MODEL_SUCCESS == status)
	{
		ROS_INFO("Tracker Pos %f, %f", this->tracker_position.position.x,
				this->tracker_position.position.y);
	}
	else
	{
		ROS_ERROR("Failed to get the tracker position.");
	}

	return status;
}

track_model_errors_e track_model::get_position(std::string model_name, geometry_msgs::Pose *model_pose){

	track_model_errors_e status = TRACK_MODEL_SUCCESS;

	gazebo_msgs::GetModelState getmodelstate;

	getmodelstate.request.model_name = model_name;
	getmodelstate.request.relative_entity_name = "map";

	if (this->gazebo_model_state_client.call(getmodelstate))
	{
		*model_pose = getmodelstate.response.pose;
	}
	else
	{
		status = TRACK_MODEL_ERROR_GET_POSITION;
	}

	return status;
}


track_model_errors_e track_model::check_tracking(){
	track_model_errors_e status = TRACK_MODEL_SUCCESS;

	this->get_goal_position();
	this->get_tracker_position();

	// is_tracked = true if goal-tracker < threshold
	this->is_tracked = false;

	return status;
}

track_model_errors_e track_model::set_goal_position(){

	track_model_errors_e status = TRACK_MODEL_SUCCESS;

	// get intermediate goal (eliminate one (shortest dist))
	// (if already at the intermediate pos, give the actual goal)

	return status;
}

track_model_errors_e track_model::compute_tracking_velocities(){

	track_model_errors_e status = TRACK_MODEL_SUCCESS;

	// go_to_goal (x, y, theta) (send velocities towards goal)

	return status;
}

track_model_errors_e track_model::filter_velocities(){

	track_model_errors_e status = TRACK_MODEL_SUCCESS;

	// upper thresholds and lower thresholds to the output velos


	return status;
}

track_model_errors_e track_model::send_tracker_velocities(geometry_msgs::Twist twist_to_tracker){

	track_model_errors_e status = TRACK_MODEL_SUCCESS;


	return status;
}
