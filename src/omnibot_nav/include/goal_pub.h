#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/LaserScan.h>


class goal_publisher
{
	private:

		void laser_data_cb(const sensor_msgs::LaserScanConstPtr& scan);
		void get_legs();
		void compute_goal_pose();

		ros::NodeHandle *node;
		ros::Subscriber laser_sub;
		ros::Publisher goal_pub;

		sensor_msgs::LaserScan laser_data;
		geometry_msgs::Pose goal_pose;

		geometry_msgs::Point leg_points[4];
		int8_t no_of_legs_detected;

	public:

		void get_goal();
		void publish_goal();
		goal_publisher(ros::NodeHandle* nodeH);
		~goal_publisher();
};
