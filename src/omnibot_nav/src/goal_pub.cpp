#include "goal_pub.h"
#include "geometry_msgs/Point.h"

const sensor_msgs::LaserScan scan_data;

int main(int argc, char **argv)
{

	ros::init(argc, argv, "goal_publisher");
	ros::NodeHandle n;

	goal_publisher gp(&n);

	ros::Rate loop_rate(2);

	while (ros::ok())
	{
		ros::spinOnce();
		ROS_INFO("Laser Message %f", scan_data.angle_max);

		gp.get_goal();

		gp.publish_goal();

		loop_rate.sleep();
	}

  return 0;
}

goal_publisher::goal_publisher(ros::NodeHandle *nodeH)
{

	this->node= nodeH;
	this->no_of_legs_detected = 0;

	this->laser_sub = node->subscribe("/laser/scan", 2, &goal_publisher::laser_data_cb, this);
	this->goal_pub = node->advertise<geometry_msgs::Pose>("/goal", 2);

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

}

void goal_publisher::compute_goal_pose()
{

}


