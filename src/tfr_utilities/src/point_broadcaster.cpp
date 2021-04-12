#include <point_broadcaster.h>

/**
 *  Utility node
 *
 *  Reports information about a moveable point on command.
 *
 *  Listens for updates about the point on a specified service. The service
 *  takes in a tfr_msgs/LocalizePoint.srv
 *
 *  It will report (0 0 0) (0 0 0 1) until it is given a proper location
 *
 *  parameters:
 *      parent_frame: the parent frame of this point (type = string default = "")
 *      point_frame: the name of the frame you want to broadcast (type = string default = "")
 *      service_name: the name of the service you want to broadcast (type = string default = "")
 *      height: height to put the point at. (type = double default: 0.0)
 *      hz: the frequency to pubish at. (type = doulbe default: 5.0)
 *
 * */

/*
 * Initializes the broadcaster and data structures
 * */
PointBroadcaster::PointBroadcaster(
        ros::NodeHandle& n, 
        const std::string &point_frame, 
        const std::string &parent_frame, 
        const std::string &service, const double& h) : 
    node{n}, 
    service_name{service}, 
    height{h}
{
    server = node.advertiseService(service_name, &PointBroadcaster::localizePoint, this);

    transform.header.frame_id = parent_frame;
    transform.child_frame_id = point_frame;
    transform.transform.rotation.w = 1;
}

/*
 * Broadcasts point across the transform network
 * */
void PointBroadcaster::broadcast()
{
    transform.header.stamp = ros::Time::now();
    broadcaster.sendTransform(transform);
}

/*
 * Gives the point a new origin
 * */
bool PointBroadcaster::localizePoint(tfr_msgs::PoseSrv::Request &request,
        tfr_msgs::PoseSrv::Response &response)
{
    
    transform.transform.translation.x = request.pose.pose.position.x;
    ROS_INFO("request.pose.pose.position.x value is %f", request.pose.pose.position.x);
    ROS_INFO("transform.transform.translation.x value is %f", transform.transform.translation.x);
    transform.transform.translation.y = request.pose.pose.position.y;
    transform.transform.translation.z = -height;
    transform.transform.rotation.x = request.pose.pose.orientation.x;
    transform.transform.rotation.y = request.pose.pose.orientation.y;
    transform.transform.rotation.z = request.pose.pose.orientation.z;
    transform.transform.rotation.w = request.pose.pose.orientation.w;
    return true;
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "point_broadcaster");
    ros::NodeHandle n;

    //get parameters
    std::string point_frame{}, parent_frame{}, service_name{};
    double height, hz;

    ros::param::param<std::string>("~parent_frame", parent_frame, "");
    ros::param::param<std::string>("~point_frame", point_frame, "");
    ros::param::param<std::string>("~service_name", service_name, "");
    ros::param::param<double>("~height", height, 0.0);
    ros::param::param<double>("~hz", hz, 5.0 );

    PointBroadcaster broadcaster{n, point_frame, parent_frame, service_name,
        height};
    geometry_msgs::PoseStamped processed_pose;
    processed_pose.pose.position.z = 0;
    processed_pose.header.stamp = ros::Time::now();
    tfr_msgs::PoseSrv::Request request{};
    request.pose = processed_pose;
    tfr_msgs::PoseSrv::Response response;

    //broadcast the point across the network
    ros::Rate rate(hz);
    //while(ros::ok())
    //{
        //broadcaster.broadcast();
        ros::spin(); //Once();
        //rate.sleep();
    //}
    broadcaster.localizePoint(request, response);
}
