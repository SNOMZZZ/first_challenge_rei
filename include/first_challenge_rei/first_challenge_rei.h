#ifndef FIRST_CHALLENGE_REI_H
#define FIRST_CHALLENGE_REI_H

#include <ros/ros.h>
#include "nav_msgs/Odometry.h"
#include "roomba_500driver_meiji/RoombaCtrl.h"
#include "tf/transform_listener.h"
#include "sensor_msgs/LaserScan.h"

class FirstChallengeRei{
    public:
        FirstChallengeRei();
        void process();

    private:
        void get_rpy(const geometry_msgs::Quaternion &q, double &roll, double &pitch, double &yaw);
        void pose_callback(const nav_msgs::Odometry::ConstPtr &msg);
        void range_callback(const sensor_msgs::LaserScan::ConstPtr &msg);
        void go_straight();

        int hz_;
        double distance_;
        float stop_distance_;
        double start_yaw;

        double roll;
        double pitch;
        double yaw;
        double old_yaw;

        double dx;
        int sum;
        int stop_sign_;
        double sum_x_;
        double sum_theta_;
        double dtheta_;
        int range_center;
        int range_width;
        double range;
        int count;
        double pose_checker = false;


        ros::NodeHandle private_nh;
        ros::NodeHandle nh;
        ros::Subscriber sub_pose;
        ros::Subscriber sub_range;
        ros::Publisher pub_cmd_vel;
        nav_msgs::Odometry old_pose;
        nav_msgs::Odometry current_pose;
        sensor_msgs::LaserScan current_range;
};
#endif
