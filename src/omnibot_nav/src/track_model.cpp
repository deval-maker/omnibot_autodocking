#include "track_model.h"

track_model::track_model(std::string goal_model, std::string tracker_model, ros::NodeHandle* nodeH)
{

	this->node = nodeH;
	this->gazebo_model_state_client = this->node->serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");;
	this->send_velo_pub = this->node->advertise<geometry_msgs::Twist>("/cmd_vel", 1000);

	this->goal_model = goal_model;

	this->tracker_model = tracker_model;

	this->is_tracked = false;

	this->check_tracking();

	this->tracking_thresholds.position.x = 0.01;
	this->tracking_thresholds.position.y = 0.01;
	this->tracking_thresholds.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 2*M_PI/180);

	this->velocity_upper_thresholds.linear.x = 1.5;
	this->velocity_upper_thresholds.linear.y = 1.5;
	this->velocity_upper_thresholds.angular.z = 1.5;

	this->velocity_lower_thresholds.linear.x = 0.1;
	this->velocity_lower_thresholds.linear.y = 0.1;
	this->velocity_lower_thresholds.angular.z = 0.1;

	ros::Duration(1).sleep();

	this->vel_to_tracker = this->zero_velo;
	this->send_tracker_velocities();

}

track_model::~track_model()
{
	this->vel_to_tracker = this->zero_velo;
	this->send_tracker_velocities();
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
//		ROS_INFO("Model Pos %f, %f", this->model_position.position.x,
//				this->model_position.position.y);
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
//		ROS_INFO("Tracker Pos %f, %f", this->tracker_position.position.x,
//				this->tracker_position.position.y);
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

	if(TRACK_MODEL_SUCCESS == status)
	{
		tf::Pose pose;
		tf::poseMsgToTF(this->tracker_position, pose);
		double yaw_tracker = tf::getYaw(pose.getRotation());

		tf::poseMsgToTF(this->model_position, pose);
		double yaw_model = tf::getYaw(pose.getRotation());

		tf::poseMsgToTF(this->tracking_thresholds, pose);
		double yaw_threshold = tf::getYaw(pose.getRotation());

		if((fabs(yaw_model - yaw_tracker) > yaw_threshold) ||
				(fabs(this->model_position.position.x - this->tracker_position.position.x) > this->tracking_thresholds.position.x) ||
				(fabs(this->model_position.position.y - this->tracker_position.position.y) > this->tracking_thresholds.position.y) )
		{
			this->is_tracked = false;
		}

		else
		{
			this->is_tracked = true;
		}
	}

	return status;
}

track_model_errors_e track_model::get_goal_position(){

	track_model_errors_e status = TRACK_MODEL_SUCCESS;

	if(!is_tracked)
	{
		this->goal = this->model_position;

		ROS_INFO("[Goal] [Pos] [x:%f] [y:%f]", this->goal.position.x, this->goal.position.y);
	}

	return status;
}

track_model_errors_e track_model::compute_tracking_velocities(){

	track_model_errors_e status = TRACK_MODEL_SUCCESS;

	this->get_goal_position();

	if(!is_tracked)
	{
		geometry_msgs::Pose error;

		error.position.x = this->goal.position.x - this->tracker_position.position.x;
		error.position.y = this->goal.position.y - this->tracker_position.position.y;

		this->vel_to_tracker.linear.x = error.position.x * 0.8;

		ROS_INFO("[Controller] [Error.x:%f] ", error.position.x);
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

	if(fabs(this->vel_to_tracker.linear.x) > this->velocity_upper_thresholds.linear.x)
	{
		if(this->vel_to_tracker.linear.x < 0)
		{
			this->vel_to_tracker.linear.x = -1 * this->velocity_upper_thresholds.linear.x;
		}
		else
		{
			this->vel_to_tracker.linear.x = this->velocity_upper_thresholds.linear.x;
		}
	}

	if(fabs(this->vel_to_tracker.linear.x) < this->velocity_lower_thresholds.linear.x &&
			this->vel_to_tracker.linear.x != 0.0)
	{
		if(this->vel_to_tracker.linear.x < 0)
		{
			this->vel_to_tracker.linear.x = -1 * this->velocity_lower_thresholds.linear.x;
		}
		else
		{
			this->vel_to_tracker.linear.x = this->velocity_lower_thresholds.linear.x;
		}
	}

	return status;
}

track_model_errors_e track_model::send_tracker_velocities(){

	track_model_errors_e status = TRACK_MODEL_SUCCESS;

	ROS_INFO("[Tracker] [Twist] [lin_x:%f] [lin_y:%f] [lin_z:%f]", this->vel_to_tracker.linear.x, this->vel_to_tracker.linear.y,
			this->vel_to_tracker.angular.z);

	this->send_velo_pub.publish(this->vel_to_tracker);

	return status;
}
