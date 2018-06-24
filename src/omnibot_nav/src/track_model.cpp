#include "track_model.h"

#include <ros/ros.h>
#include <gazebo_msgs/GetModelState.h>
#include <string.h>
#include <geometry_msgs/Pose.h>

track_model::track_model(std::string goal_model, std::string tracker_model, ros::NodeHandle* nodeH)
{

	this->node = nodeH;
	this->gazebo_model_state_client = this->node->serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");;
	this->send_velo_pub = this->node->advertise<geometry_msgs::Twist>("/cmd_vel", 1000);

	this->goal_model = goal_model;

	this->tracker_model = tracker_model;

	is_tracked = false;

	this->check_tracking();

	//	set tracking_thresholds;
	//	set velocity_upper_thresholds;

	ros::Duration(1).sleep();

	this->send_tracker_velocities(this->zero_velo);

}

track_model::~track_model()
{
	this->send_tracker_velocities(this->zero_velo);
}

track_model_errors_e track_model::get_model_position()
{

	track_model_errors_e status = TRACK_MODEL_SUCCESS;

	if(TRACK_MODEL_SUCCESS == status)
	{
		status = this->get_position("table", &this->model_position);
	}

	if(TRACK_MODEL_SUCCESS == status)
	{
		ROS_INFO("Model Pos %f, %f", this->tracker_position.position.x,
				this->tracker_position.position.y);
	}
	else
	{
		ROS_ERROR("Failed to get the goal position.");
	}

	return status;
}

track_model_errors_e track_model::get_tracker_position()
{

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

track_model_errors_e track_model::get_position(std::string model_name, geometry_msgs::Pose *model_pose)
{

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


track_model_errors_e track_model::check_tracking()
{
	track_model_errors_e status = TRACK_MODEL_SUCCESS;

	if(TRACK_MODEL_SUCCESS == status)
	{
		status = this->get_model_position();
	}

	if(TRACK_MODEL_SUCCESS == status)
	{
		status = this->get_tracker_position();
	}

	// is_tracked = true if goal-tracker < threshold
	this->is_tracked = false;

	return status;
}

track_model_errors_e track_model::set_goal_position(){

	track_model_errors_e status = TRACK_MODEL_SUCCESS;

	// get intermediate goal (eliminate one (shortest dist))
	// (if already at the intermediate pos, give the actual goal)

	this->goal.position.x = 0.0;
	this->goal.position.y = 0.0;

	ROS_INFO("Goal set to Pos x:%f, y:%f", this->goal.position.x,
			this->goal.position.y);

	return status;
}

track_model_errors_e track_model::compute_tracking_velocities(){

	track_model_errors_e status = TRACK_MODEL_SUCCESS;

	// go_to_goal (x, y, theta) (send velocities towards goal)

	if(!is_tracked)
	{
		this->vel_to_tracker.linear.x = 0.5;
	}
	else
	{
		this->vel_to_tracker = this->zero_velo;
	}

	this->filter_tracking_velocities();

	return status;
}

track_model_errors_e track_model::filter_tracking_velocities(){

	track_model_errors_e status = TRACK_MODEL_SUCCESS;

	// upper thresholds and lower thresholds to the output velos

	return status;
}

track_model_errors_e track_model::send_tracker_velocities(geometry_msgs::Twist twist_to_tracker){

	track_model_errors_e status = TRACK_MODEL_SUCCESS;

	ROS_INFO("Send Tracker velocities x:%f, y:%f angZ:%f ", twist_to_tracker.linear.x, twist_to_tracker.linear.y, twist_to_tracker.angular.z);

	this->send_velo_pub.publish(twist_to_tracker);

	return status;
}
