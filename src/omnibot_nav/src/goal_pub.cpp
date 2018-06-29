#include "goal_pub.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "goal_publisher");
	ros::NodeHandle n;

	goal_publisher gp(&n);

	ros::Rate loop_rate(5);

	ros::Duration(1).sleep();

	ROS_INFO("Omnibot Goal Publisher node started.");

	while (ros::ok())
	{
		ros::spinOnce();

		gp.get_goal();

		gp.publish_goal();

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

void goal_publisher::get_goal()
{
	this->get_legs();
	this->compute_goal_pose();
}

void goal_publisher::publish_goal()
{
	goal_pub.publish(this->goal_pose);
}

void goal_publisher::laser_data_cb(const sensor_msgs::LaserScanConstPtr& scan)
{
	this->laser_data = *scan;
}

void goal_publisher::get_legs()
{
	int32_t i = 0,j = 0;
	float_t angle = 0.0;
	int32_t previous_detection = 0;
	int32_t leg_indexes[4] = {0};
	int32_t same_leg_count = 0;

	for(i = 0; i < NO_OF_SAMPLES_LASER; i++)
	{
		if((this->laser_data.ranges[i] < this->laser_data.range_max) && (this->laser_data.ranges[i] > this->laser_data.range_min))
		{
			if(i != previous_detection+1)
			{
				leg_indexes[j] = i;
				if(j != 0)
				{
					leg_indexes[j-1] = leg_indexes[j-1] + same_leg_count /2;
				}
				same_leg_count = 1;
				j++;
			}

			else
			{
				same_leg_count++;
			}

			previous_detection = i;

		}
	}

	leg_indexes[j-1] = leg_indexes[j-1] + same_leg_count /2;

	for(i = 0; i < 4; i++)
	{
		angle = ((((float)leg_indexes[i]) * 2 * M_PI) / NO_OF_SAMPLES_LASER) - M_PI;
		this->leg_points[i].x = (this->laser_data.ranges[leg_indexes[i]] + LEG_RADIUS)* cos(angle);
		this->leg_points[i].y = (this->laser_data.ranges[leg_indexes[i]] + LEG_RADIUS)  * sin(angle);

		ROS_DEBUG("Leg%d (%f,%f, %f)", i,this->leg_points[i].x, this->leg_points[i].y, angle*180 / M_PI);
	}

}

void goal_publisher::compute_goal_pose()
{
	// Naming Convention: (ABCD -> leg(0,1,2,3))

	// slope of line BC or AD = Theta
	double_t angle = -1 * atan2((leg_points[2].x - leg_points[1].x), (leg_points[2].y - leg_points[1].y));

	tf::quaternionTFToMsg(tf::createQuaternionFromYaw(angle), this->goal_pose.pose.orientation);

	// get center of the table (Mid point of AC or BD)
	this->goal_pose.pose.position.x = (this->leg_points[0].x + this->leg_points[2].x) / 2.0;
	this->goal_pose.pose.position.y = (this->leg_points[0].y + this->leg_points[2].y) / 2.0;

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
		ros::Duration(1.0).sleep();
	}

	tf::Pose pose_tf;
	tf::poseMsgToTF(goal_pose.pose, pose_tf);

	ROS_INFO("Goal X:%f Y:%f Theta: %f", this->goal_pose.pose.position.x, this->goal_pose.pose.position.y,
			tf::getYaw(pose_tf.getRotation()) * 180 /M_PI);

}
