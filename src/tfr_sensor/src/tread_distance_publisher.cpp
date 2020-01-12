#include <ros/ros.h>
#include "std_msgs/UInt32.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float64.h"
#include <boost/function.hpp>
#include "tread_distance_publisher.h"
#include <cmath>

TreadDistance::TreadDistance(const int ticksPerRevolution, const int maxTicks, const double wheelRadius, const int prevTickCount) :
    distanceTraveled{ 0 }, prevTickCount{ prevTickCount }, ticksPerRevolution{ ticksPerRevolution }, maxTicks{ maxTicks }, wheelCircumference{ wheelRadius * 2 * M_PI} {}

void TreadDistance::updateFromNewCount(const int newCount) {
    auto ticksMoved = calcTickDiff(newCount);
    distanceTraveled = (wheelCircumference * ticksMoved) / ticksPerRevolution;
    prevTickCount = newCount;
}


int TreadDistance::calcTickDiff(const int newCount) {
    //TODO: handle rollover from going past maxTicks

    // otherwise its a simple difference
    return newCount - prevTickCount;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "tread_distance_publisher");
    ros::NodeHandle n; //NodeHandle is the main access point to communications with the ROS system.

    bool requiredParamsFound = true;
    double wheelRadius, ticksPerRevolution, maxTicks;
    double rate; //rate: how quickly to publish hz.
    requiredParamsFound = requiredParamsFound && ros::param::get("~wheelRadius", wheelRadius);
    requiredParamsFound = requiredParamsFound && ros::param::get("~ticksPerRevolution", ticksPerRevolution);
    requiredParamsFound = requiredParamsFound && ros::param::get("~maxTicks", maxTicks);
    //if (!requiredParamsFound) {//throw an error or something...}
    
    ros::Publisher leftTreadPublisher = n.advertise<std_msgs::Float64>("/left_tread_distance", 15);
    ros::Publisher rightTreadPublisher = n.advertise<std_msgs::Float64>("/right_tread_distance", 15);
    TreadDistance leftTread(ticksPerRevolution, maxTicks, wheelRadius), rightTread(ticksPerRevolution, maxTicks, wheelRadius);
    boost::function<void(const std_msgs::Int32&)> leftTreadCallback = [&leftTread, &leftTreadPublisher](const std_msgs::Int32& msg) {
        std_msgs::Float64 new_msg;
        leftTread.updateFromNewCount(msg.data);
        new_msg.data = leftTread.distanceTraveled;
        leftTreadPublisher.publish(new_msg);

    };
    boost::function<void(const std_msgs::Int32&)> rightTreadCallback = [&rightTread, &rightTreadPublisher](const std_msgs::Int32& msg) {
        std_msgs::Float64 new_msg;
        rightTread.updateFromNewCount(msg.data);
        new_msg.data = rightTread.distanceTraveled;
        rightTreadPublisher.publish(new_msg);

    };
    auto leftTreadCountSub = n.subscribe<std_msgs::Int32>("/left_tread_count", 10, leftTreadCallback);
    auto rightTreadCountSub = n.subscribe<std_msgs::Int32>("/right_tread_count", 10, rightTreadCallback);
    
    ros::param::param<double>("~rate", rate, 32.0);
    ros::Rate loop_rate(rate);
    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();

    }
    return 0;
}
