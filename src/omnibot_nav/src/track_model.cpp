#include "track_model.h"

track_model::track_model(std::string goal_model, std::string tracker_model, ros::NodeHandle* nodeH)
{

	this->node = nodeH;
	this->gazebo_model_state_client = this->node->serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");;
	this->send_velo_pub = this->node->advertise<geometry_msgs::Twist>("/cmd_vel", 1000);

	this->goal_model = goal_model;

	this->tracker_model = tracker_model;

	this->tracker_state.x = 0.0;
	this->tracker_state.y = 0.0;
	this->tracker_state.theta = 0.0;

	this->model_state.x = 0.0;
	this->model_state.y = 0.0;
	this->model_state.theta = 0.0;

	this->goal_state.x = 0.0;
	this->goal_state.y = 0.0;
	this->goal_state.theta = 0.0;

	this->get_all_positions();

	this->tracking_thresholds.x = 0.01;
	this->tracking_thresholds.y = 0.01;
	this->tracking_thresholds.theta = (2*M_PI/180);

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
		this->pose_to_state(&(this->model_position), &(this->model_state));

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
		this->pose_to_state(&(this->tracker_position), &(this->tracker_state));

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

track_model_errors_e track_model::get_all_positions()
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

	return status;
}

track_model_errors_e track_model::get_goal_position(){

	track_model_errors_e status = TRACK_MODEL_SUCCESS;

	this->goal = this->model_position;

	this->pose_to_state(&(this->goal), &(this->goal_state));

	ROS_INFO("[Goal] [State] [x:%f] [y:%f] [theta:%f]", this->goal_state.x, this->goal_state.y, this->goal_state.theta);

	return status;
}

track_model_errors_e track_model::compute_tracking_velocities(){

	track_model_errors_e status = TRACK_MODEL_SUCCESS;
	state_var_s error;

	//	compute_error and velocities
	error.x = this->goal_state.x - this->tracker_state.x;
	error.y = this->goal_state.y - this->tracker_state.y;
	error.theta = this->goal_state.theta -this->tracker_state.theta;

	if((fabs(error.x) > this->tracking_thresholds.x))
	{
		this->vel_to_tracker.linear.x = error.x * 0.8;
	}

	else
	{
		this->vel_to_tracker.linear.x = this->zero_velo.linear.x;
	}

	if((fabs(error.y) > this->tracking_thresholds.y))
	{
		this->vel_to_tracker.linear.y = error.y * 1.5;
	}

	else
	{
		this->vel_to_tracker.linear.y = this->zero_velo.linear.y;
	}

	ROS_INFO("[Controller] [State] [Error.x:%f] [Error.y:%f] [Error.theta:%f]", error.x, error.y, error.theta);

	this->filter_tracking_velocities();

	return status;
}

track_model_errors_e track_model::filter_tracking_velocities(){

	track_model_errors_e status = TRACK_MODEL_SUCCESS;

	// X direction
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


	// Y direction
	if(fabs(this->vel_to_tracker.linear.y) > this->velocity_upper_thresholds.linear.y)
	{
		if(this->vel_to_tracker.linear.y < 0)
		{
			this->vel_to_tracker.linear.y = -1 * this->velocity_upper_thresholds.linear.y;
		}
		else
		{
			this->vel_to_tracker.linear.y = this->velocity_upper_thresholds.linear.y;
		}
	}

	if(fabs(this->vel_to_tracker.linear.y) < this->velocity_lower_thresholds.linear.y &&
			this->vel_to_tracker.linear.y != 0.0)
	{
		if(this->vel_to_tracker.linear.y < 0)
		{
			this->vel_to_tracker.linear.y = -1 * this->velocity_lower_thresholds.linear.y;
		}
		else
		{
			this->vel_to_tracker.linear.y = this->velocity_lower_thresholds.linear.y;
		}
	}

	return status;
}

track_model_errors_e track_model::send_tracker_velocities(){

	track_model_errors_e status = TRACK_MODEL_SUCCESS;

	ROS_INFO("[Tracker] [Twist] [lin_x:%f] [lin_y:%f] [ang_z:%f]", this->vel_to_tracker.linear.x, this->vel_to_tracker.linear.y,
			this->vel_to_tracker.angular.z);

	this->send_velo_pub.publish(this->vel_to_tracker);

	return status;
}

track_model_errors_e track_model::pose_to_state(geometry_msgs::Pose *pose, state_var_s *state){

	track_model_errors_e status = TRACK_MODEL_SUCCESS;

	state->x = pose->position.x;
	state->y = pose->position.y;

	tf::Pose pose_tf;
	tf::poseMsgToTF( *pose, pose_tf);

	state->theta = tf::getYaw(pose_tf.getRotation());

	return status;
}


