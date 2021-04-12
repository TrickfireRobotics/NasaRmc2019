#ifndef POINT_BROADCASTER_H 
#define POINT_BROADCASTER_H 

#include <ros/ros.h>
#include <ros/console.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <tfr_msgs/PoseSrv.h>

/**
 *  Utility node
 *
 *  Reports information about a moveable point on command.
 *
 *  Listens for updates about the point on a specified service. The service
 *  takes in a tfr_msgs/LocalizePoint.srv
 *
 *  It will report (0 0 0) (0 0 0 0) until it is given a proper location
 *
 *  It also provides utilities for shifting a point by a set amount
 *
 *  parameters:
 *      parent_frame: the parent frame of this point (type = string default = "")
 *      point_frame: the name of the frame you want to broadcast (typed = string default = "")
 *      service_name: the name of the service you want to broadcast (typed =
 *      string default = "")
 *      hz: the frequency to pubish at. (type = doulbe default: 5.0)
 *
 * */
class PointBroadcaster 
{
    public:
        PointBroadcaster(ros::NodeHandle &n, const std::string &point_frame, const  std::string
                &parent_frame, const std::string &service, const double& height);
        ~PointBroadcaster(){};

        PointBroadcaster(const PointBroadcaster&) = delete;
        PointBroadcaster& operator=( const PointBroadcaster&) = delete;
        PointBroadcaster(PointBroadcaster&&) = delete;
        PointBroadcaster& operator=(PointBroadcaster&&) = delete;

        void broadcast();

    private:
        ros::NodeHandle &node;
        tf2_ros::TransformBroadcaster broadcaster{};
        geometry_msgs::TransformStamped transform{};
        ros::ServiceServer server;
        const std::string &service_name;
        const double& height;

        bool localizePoint(tfr_msgs::PoseSrv::Request &request,
                tfr_msgs::PoseSrv::Response &response);

};
#endif
