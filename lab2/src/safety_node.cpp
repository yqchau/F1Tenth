// include ROS msg type headers and libraries
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Bool.h>
#include <ackermann_msgs/AckermannDriveStamped.h> 
#include <cmath>


void fill_vector(std::vector<float>& vec, float a, float b, float inc){
    float eps = 0.0000001f;
    for(float n = a; n <= b+eps; n += inc)
      vec.push_back(n);
}

class Safety {
// The class that handles emergency braking
private:
    ros::NodeHandle nh;
    double speed;
    //create ROS subscribers and publishers
    ros::Subscriber odom_subscriber; 
    ros::Subscriber scan_subscriber;
    ros::Publisher brake_publisher; 
    ros::Publisher brakeBool_publisher; 

    float angleMin;
    float angleMax;
    float angleInc;
    

public:
    Safety() {
        nh = ros::NodeHandle();
        speed = 0.0;
        /*
        One publisher should publish to the /brake topic with an
        ackermann_msgs/AckermannDriveStamped brake message.

        One publisher should publish to the /brake_bool topic with a
        std_msgs/Bool message.

        You should also subscribe to the /scan topic to get the
        sensor_msgs/LaserScan messages and the /odom topic to get
        the nav_msgs/Odometry messages

        The subscribers should use the provided odom_callback and 
        scan_callback as callback methods

        NOTE that the x component of the linear velocity in odom is the speed
        */

        // create ROS subscribers and publishers
        odom_subscriber = nh.subscribe("/odom", 10, &Safety::odom_callback, this);
        scan_subscriber = nh.subscribe("/scan", 10, &Safety::scan_callback, this);
        brake_publisher = nh.advertise<ackermann_msgs::AckermannDriveStamped>("/brake", 10);
        brakeBool_publisher = nh.advertise<std_msgs::Bool>("/brake_bool", 10);
        
    }
    void odom_callback(const nav_msgs::Odometry::ConstPtr &odom_msg) {
        // TODO: update current speed
        speed = odom_msg->twist.twist.linear.x;
        // ROS_INFO("Speed: %f", speed);

    }

    void scan_callback(const sensor_msgs::LaserScan::ConstPtr &scan_msg) {
        // TODO: calculate TTC
        const std::vector<float> scanRange = scan_msg -> ranges;
        std::vector<float> angles;
        fill_vector(angles, angleMin, angleMax, angleInc);

        float v, calc, ttc;
        float eps = 1e-7;
        float ttc_thres = 0.4;
        float perc_thres = 0.1;

        angleMin = scan_msg -> angle_min;
        angleMax = scan_msg -> angle_max;
        angleInc = scan_msg -> angle_increment;

        int hit = 0;
        for (int i=0; i<scanRange.size(); i++) {
            // ttc.push_back(float(speed) * std::cos(angles[i]));
            // ROS_INFO("Speed: %f, Angle: %f, Cos(Angle): %f", float(speed), angles[i], std::cos(angles[i]));
            // std::cout << speed << angles[i] << std::cos(angles[i]) << std::endl;
            v = float(speed) * std::cos(angles[i]);
            v = float(std::max(double(v), 0.)) + eps;

            ttc = float(scanRange[i]/v);
            // ROS_INFO("calc: %f", calc);
            // std::cout << calc << std::endl;

            if (ttc < ttc_thres) hit++;
        }

        // if more than 10% of points exceeds ttc_threshold, brake!! 
        if (hit/1024. > perc_thres){
            std_msgs::Bool brakeBoolMsg;
            ackermann_msgs::AckermannDriveStamped brakeMsg;

            brakeBoolMsg.data = true;
            brakeMsg.drive.speed = 0.0;

            brake_publisher.publish(brakeMsg);
            brakeBool_publisher.publish(brakeBoolMsg);

            ROS_INFO("BRAKE!!!");
        }

        // ROS_INFO("MAX TTC: %f", std::ranges::max_element(ttc.begin(), ttc.end()));
        // ttc.clear();
        // TODO: publish drive/brake message
        return;
    }

};

int main(int argc, char ** argv) {
    ros::init(argc, argv, "safety_node");
    Safety sn;
    ros::spin();
    return 0;
}