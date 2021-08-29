#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float64.h> 
#include <lab1/scan_range.h>

class Echoer {
    public:
        void callback(const sensor_msgs::LaserScanConstPtr& msg) {
            std::vector<float> ranges = msg->ranges;

            float min =  std::numeric_limits<float>::infinity();
            float max = 0.f;

            for (const float& r: ranges) {
                if (std::isnan(r) || std::isinf(r))
                    continue;
                if (r < min) min = r;
                if (r > max) max = r;
            }

            _closest.data = (min < msg->range_min) ? msg->range_min : min;
            _farthest.data = (max > msg->range_max) ? msg->range_max : max;

            _scan_range.range_min = _closest.data;
            _scan_range.range_max = _farthest.data;

            ROS_INFO("LaserScan message received: closest %f, farthest %f",
             _closest.data, _farthest.data);
        }

        std_msgs::Float64 get_closest(){
            return _closest;
        }

        std_msgs::Float64 get_farthest(){
            return _farthest;
        }

        lab1::scan_range get_scan_range(){ 
            return _scan_range;
        }
    
    private:
        std_msgs::Float64 _closest;
        std_msgs::Float64 _farthest;
        lab1::scan_range _scan_range;

};


int main(int argc, char** argv) { 
    ros::init(argc, argv, "echoer", ros::init_options::AnonymousName);
    ros::NodeHandle nh;
    ros::Rate loop_rate(10);

    Echoer echoer; 
    ros::Subscriber laserScan_subscriber = nh.subscribe("/scan", 10, &Echoer::callback, &echoer);
    ros::Publisher closestPoint_publisher = nh.advertise<std_msgs::Float64>("/closest_point", 10);
    ros::Publisher farthestPoint_publisher = nh.advertise<std_msgs::Float64>("/farthest_point", 10);
    ros::Publisher scanRange_publisher = nh.advertise<lab1::scan_range>("/scan_range", 10);

    while (ros::ok()) {
        closestPoint_publisher.publish(echoer.get_closest());
        farthestPoint_publisher.publish(echoer.get_farthest());
        scanRange_publisher.publish(echoer.get_scan_range());

        loop_rate.sleep();
        ros::spinOnce();
    }

    return 0;
}