#include "goal_pub.h"
#include "geometry_msgs/Point.h"

int main(int argc, char **argv)
{

	ros::init(argc, argv, "goal_publisher");
	ros::NodeHandle n;

	goal_publisher gp(&n);

	ros::Rate loop_rate(2);

	ros::Duration(3).sleep();

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
	this->goal_pub = node->advertise<geometry_msgs::Pose>("/goal", 1);

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

	for(i = 0; i < 720; i++)
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
		angle = (((((float)leg_indexes[i]) * 2 * M_PI) / 720) - M_PI);
		ROS_INFO("angle %f", angle);
		this->leg_points[i].x = (this->laser_data.ranges[leg_indexes[i]] + 0.1)* cos(angle);
		this->leg_points[i].y = -1 * (this->laser_data.ranges[leg_indexes[i]] + 0.1)  * sin(angle);

		ROS_INFO("Leg%d (%f,%f)", i,this->leg_points[i].x, this->leg_points[i].y);
	}

}

void goal_publisher::compute_goal_pose()
{

}
