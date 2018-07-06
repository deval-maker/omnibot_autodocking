#include "goal_pub.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "goal_publisher");
	ros::NodeHandle n;

	goal_publisher gp(&n);

	ros::Rate loop_rate(5);

	ros::Duration(1).sleep();

	ROS_INFO("Omnibot Goal Publisher node started.");

	goal_pub_e status = GOAL_PUB_SUCCESS;

	while (ros::ok())
	{
		status = GOAL_PUB_SUCCESS;

		ros::spinOnce();

		if(GOAL_PUB_SUCCESS == status)
		{
			status = gp.get_goal();
		}

		if(GOAL_PUB_SUCCESS == status)
		{
			status = gp.publish_goal();
		}

		loop_rate.sleep();
	}

	return 0;
}

goal_publisher::goal_publisher(ros::NodeHandle *nodeH)
{

	this->node= nodeH;

	this->laser_sub = node->subscribe("/laser/scan", 1, &goal_publisher::laser_data_cb, this);
	this->goal_pub = node->advertise<geometry_msgs::PoseStamped>("/goal", 1);
}

goal_publisher::~goal_publisher()
{

}

goal_pub_e goal_publisher::get_goal()
{
	goal_pub_e status = GOAL_PUB_SUCCESS;

	if(GOAL_PUB_SUCCESS == status)
	{
		status = this->get_legs();
	}

	if(GOAL_PUB_SUCCESS == status)
	{
		status = this->compute_goal_pose();
	}

	return status;
}

goal_pub_e goal_publisher::publish_goal()
{
	goal_pub_e status = GOAL_PUB_SUCCESS;

	goal_pub.publish(this->goal_pose);

	return status;
}

void goal_publisher::laser_data_cb(const sensor_msgs::LaserScanConstPtr& scan)
{
	this->laser_data = *scan;
}

goal_pub_e goal_publisher::get_legs(void)
{
	goal_pub_e status = GOAL_PUB_SUCCESS;

	int32_t i = 0;
	float_t angle = 0.0;
	int32_t previous_detection = 0;
	int32_t leg_indexes[4] = {0};
	int32_t same_leg_count = 0;
	int8_t no_of_leg_detected = 0;

	for(i = 0; i < NO_OF_SAMPLES_LASER; i++)
	{
		if((this->laser_data.ranges[i] < this->laser_data.range_max) && (this->laser_data.ranges[i] > this->laser_data.range_min))
		{
			if(i != previous_detection+1)
			{
				leg_indexes[no_of_leg_detected] = i;
				if(no_of_leg_detected != 0)
				{
					leg_indexes[no_of_leg_detected-1] = leg_indexes[no_of_leg_detected-1] + same_leg_count /2;
				}
				same_leg_count = 1;
				no_of_leg_detected++;
			}

			else
			{
				same_leg_count++;
			}

			previous_detection = i;

		}
	}

	leg_indexes[no_of_leg_detected-1] = leg_indexes[no_of_leg_detected-1] + same_leg_count /2;

	for(i = 0; i < no_of_leg_detected; i++)
	{
		angle = ((((float)leg_indexes[i]) * 2 * M_PI) / NO_OF_SAMPLES_LASER) - M_PI;
		this->leg_points[i].x = (this->laser_data.ranges[leg_indexes[i]] + LEG_RADIUS)* cos(angle);
		this->leg_points[i].y = (this->laser_data.ranges[leg_indexes[i]] + LEG_RADIUS)  * sin(angle);

		ROS_DEBUG("Leg%d (%f,%f, %f)", i,this->leg_points[i].x, this->leg_points[i].y, angle*180 / M_PI);
	}

	if(no_of_leg_detected == 3)
	{
		this->extrapolate_the_fourth_point();
	}
	else if(no_of_leg_detected < 3)
	{
		ROS_ERROR("Only %d legs detected. ", no_of_leg_detected);
		status = GOAL_PUB_ERROR_LEG_COUNT_NOT_ENOUGH;
	}

	return status;
}

goal_pub_e goal_publisher::compute_goal_pose(void)
{
	// Naming Convention: (ABCD -> leg(0,1,2,3))
	goal_pub_e status = GOAL_PUB_SUCCESS;

	if(GOAL_PUB_SUCCESS == status)
	{
		// slope of line perpendicular to the bigger side
		// get the 2 points of a bigger side
		int i = 0;
		float_t dist_of_side = 0.0;

		for(i = 1; i < 4; i++)
		{
			dist_of_side = sqrt(pow((leg_points[i].x - leg_points[0].x), 2) + pow((leg_points[i].y - leg_points[0].y), 2));
			ROS_DEBUG("dist_of_side: %f (%d)", dist_of_side, i);

			if(fabs(dist_of_side - LENGTH_BIG_SIDE) <= ERROR_THRESHOLD_COMPARE)
			{
				break;
			}
		}

		// get the angle
		float_t angle = this->get_table_pose_angle(leg_points[0], leg_points[i]);

		tf::quaternionTFToMsg(tf::createQuaternionFromYaw(angle), this->goal_pose.pose.orientation);

		// get center of the table (Invariant of the Order of the points)
		this->goal_pose.pose.position.x = (this->leg_points[0].x + this->leg_points[1].x + this->leg_points[2].x + this->leg_points[3].x) / 4.0;
		this->goal_pose.pose.position.y = (this->leg_points[0].y + this->leg_points[1].y + this->leg_points[2].y + this->leg_points[3].y) / 4.0;

		ROS_DEBUG("Before TF X:%f, Y:%f, Theta %lf", this->goal_pose.pose.position.x, this->goal_pose.pose.position.y, angle * 180 / M_PI);

		this->goal_pose.header.frame_id = "/base_link";
		this->goal_pose.header.stamp = ros::Time(0);

		// base_link to map tf
		try
		{
			this->listener.waitForTransform("/map", "/base_link", ros::Time(0), ros::Duration(1.0));
			this->listener.transformPose("/map", this->goal_pose, this->goal_pose);
		}

		catch (tf::TransformException &ex)
		{
			ROS_ERROR("%s",ex.what());
			status = GOAL_PUB_ERROR_TRANSFORM_EX;
			ros::Duration(1.0).sleep();
		}
	}

	if(GOAL_PUB_SUCCESS == status)
	{
		tf::Pose pose_tf;
		tf::poseMsgToTF(goal_pose.pose, pose_tf);

		ROS_INFO("Goal X:%f Y:%f Theta: %f", this->goal_pose.pose.position.x, this->goal_pose.pose.position.y,
				tf::getYaw(pose_tf.getRotation()) * 180 /M_PI);
	}

	return status;
}

float_t goal_publisher::get_table_pose_angle(geometry_msgs::Point p1, geometry_msgs::Point p2)
{

	double_t angle_1 = -1 * atan2((p1.x - p2.x), (p1.y - p2.y));
	double_t angle_2 = -1 * atan2((p2.x - p1.x), (p2.y - p1.y));

	ROS_DEBUG("Angles: %f %f", angle_1 * 180 /M_PI, angle_2 * 180 /M_PI);

	// get the angle closer to zero
	if(abs(angle_1) < abs(angle_2))
	{
		return angle_1;
	}
	else
	{
		return angle_2;
	}

}

void goal_publisher::extrapolate_the_fourth_point(void)
{
	float_t ab = sqrt(pow((leg_points[0].x - leg_points[1].x),2) + pow((leg_points[0].y - leg_points[1].y), 2));
	float_t bc = sqrt(pow((leg_points[1].x - leg_points[2].x),2) + pow((leg_points[1].y - leg_points[2].y), 2));
	float_t ca = sqrt(pow((leg_points[2].x - leg_points[0].x),2) + pow((leg_points[2].y - leg_points[0].y), 2));

	if((ab > bc) && (ab > ca))
	{
		// ab is largest
		this->leg_points[3].x = this->leg_points[0].x + this->leg_points[1].x - this->leg_points[2].x;
		this->leg_points[3].y = this->leg_points[0].y + this->leg_points[1].y - this->leg_points[2].y;	}

	else if((bc > ab) && (bc > ca))
	{
		// bc is largest
		this->leg_points[3].x = this->leg_points[1].x + this->leg_points[2].x - this->leg_points[0].x;
		this->leg_points[3].y = this->leg_points[1].y + this->leg_points[2].y - this->leg_points[0].y;
	}

	else
	{
		// ca is largest
		this->leg_points[3].x = this->leg_points[2].x + this->leg_points[0].x - this->leg_points[1].x;
		this->leg_points[3].y = this->leg_points[2].y + this->leg_points[0].y - this->leg_points[1].y;
	}

	ROS_INFO("Last Point x: %f, y: %f", this->leg_points[3].x, this->leg_points[3].y);

}

