#include "first_challenge_rei/first_challenge_rei.h"

FirstChallengeRei::FirstChallengeRei():private_nh(""){
    private_nh.param("hz", hz_, {50});
    private_nh.param("distance", distance_, {1});
    private_nh.param("stop_distance_", stop_distance_, {0.5});
    sub_pose = nh.subscribe("/roomba/odometry", 1, &FirstChallengeRei::pose_callback, this);
    sub_range = nh.subscribe("/scan", 1, &FirstChallengeRei::range_callback, this);
    sum_x_ = 0;
    sum_theta_ = 0;
    pub_cmd_vel = nh.advertise<roomba_500driver_meiji::RoombaCtrl>("/roomba/control", 1);
}

void FirstChallengeRei::get_rpy(const geometry_msgs::Quaternion &q, double &roll, double &pitch, double &yaw){
    tf::Quaternion quat(q.x, q.y, q.z, q.w);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
}

void FirstChallengeRei::pose_callback(const nav_msgs::Odometry::ConstPtr &msg){
    old_pose = current_pose;
    current_pose = *msg;
    if(!pose_checker) old_pose = current_pose;
    pose_checker = true;

    dx = current_pose.pose.pose.position.x - old_pose.pose.pose.position.x;
    sum_x_ += dx;

    old_yaw = yaw;
    get_rpy(current_pose.pose.pose.orientation, roll, pitch, yaw);
    if(yaw*old_yaw < 0) dtheta_ = 0.0;
    else dtheta_ = fabs(yaw - old_yaw);

    if(sum_x_ >= 0.5){
        stop_sign_ = 1;
        sum_theta_ += dtheta_;
        if(sum_theta_ >= 2*M_PI - 0.1){
            stop_sign_ = 3;
            sum_x_ = 0;
        }
    }
}

void FirstChallengeRei::range_callback(const sensor_msgs::LaserScan::ConstPtr &msg){
    current_range = *msg;

    int range_center = current_range.ranges.size()/2;
    int range_width = 10;
    double range = 0.0;
    int count = 0;
    int sum = 0;
    if(current_range.ranges.size() > 0){
        if(stop_sign_ == 3){
            for(int i=range_center-range_width; i<range_center+range_width; i++){
                range += current_range.ranges[i];
                sum += 1;
            }
            std::cout << range/sum << std::endl;
            if(range/sum <= stop_distance_){
                stop_sign_ = 2;
            }

        }
    }
}

void FirstChallengeRei::go_straight(){
    roomba_500driver_meiji::RoombaCtrl cmd_vel;

    cmd_vel.mode = 11;

    cmd_vel.cntl.linear.x = 0.2;


    switch (stop_sign_){
        case 0:
            sum_theta_ = 0.0;
            cmd_vel.cntl.linear.x = 0.2;
            cmd_vel.cntl.angular.z = 0.0;
            break;
        case 1:
            cmd_vel.cntl.linear.x = 0;
            cmd_vel.cntl.angular.z = 0.2;
            break;
        case 2:
            sum_x_ = 0;
            cmd_vel.cntl.linear.x = 0.0;
            cmd_vel.cntl.angular.z = 0.0;
            break;
        case 3:
            cmd_vel.cntl.linear.x = 0.2;
            cmd_vel.cntl.angular.z = 0.0;
            break;
    }

    pub_cmd_vel.publish(cmd_vel);
}

void FirstChallengeRei::process(){
    ros::Rate loop_rate(hz_);
    sum_x_ = 0;
    sum_theta_ = 0;
    old_yaw = 0;
    yaw = 0;
    stop_sign_ = 0;

    while(ros::ok()){
        go_straight();
        std::cout << stop_sign_ << std::endl;
        ros::spinOnce();
        loop_rate.sleep();
    }
}

int main(int argc, char** argv){
    ros::init(argc, argv, "first_challenge_rei");
    FirstChallengeRei first_challenge_rei;
    first_challenge_rei.process();
    return 0;
}
