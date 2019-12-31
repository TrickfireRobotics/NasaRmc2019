/* * Converts measured wheel velocities into an an odometry message for use in
 * sensor fusion.
 * 
 * Not currently configured to publish transforms, as that is the job of sensor
 * fusion right now.
 *
 * Parameters:
 *   - ~parent_frame: the frame our robot exists in (string, default: "odom")
 *   - ~child_frame: the frame of the robot (string, default: "base_footprint")
 *   - ~wheel_span: the separation of the treads of the robot. (double,
 *   default)
 *   - ~rate: how quickly to publish hz. (double, default 10)
 * Subscribed topics:
 *   - /arduino :(tfr_msgs/ArduinoReading) The most current information coming
 *   in from the sensors.
 * Published topics: 
 *   - /drivebase_odom : (nav_msgs/Odometry) the location of the
 *   base_footprint tracked by tread motion.
 * Services:
 *  - /set_drivebase_odometry : (tfr_msgs/SetOdometry) resets the basis of
 *  odometry to a new position
 * */
#include <ros/ros.h>
#include <boost/function.hpp>
#include <tfr_msgs/SetOdometry.h>
#include <tfr_msgs/PoseSrv.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Odometry.h>
#include <std_srvs/Empty.h>
#include <tf/transform_datatypes.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/convert.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Scalar.h>
#include "std_msgs/Float64.h"

class DrivebaseOdometryPublisher
{

    public:
    /********************************************
    *Constructor
    *Preconditions:
    *Postconditions:ROS masternode will have an updated registry of who is 
    *       publishing and who is subscribing to what topics
    ********************************************/
    DrivebaseOdometryPublisher(ros::NodeHandle &n, 
                const std::string& p_frame, 
                const std::string& c_frame,
                const double& wheel_sep) :
            parent_frame{p_frame},
            child_frame{c_frame},
            wheel_span{wheel_sep},
            x{},
            y{},
            angle{},
            tf_broadcaster{}
    {
        //get most current sensor infromation
        boost::function<void(const std_msgs::Float64&)> leftTreadCallback = [this](const std_msgs::Float64& msg) {this->leftTreadSpeed = msg.data; };
        boost::function<void(const std_msgs::Float64&)> rightTreadCallback = [this](const std_msgs::Float64& msg) {this->rightTreadSpeed = msg.data; };
        

	leftTreadDistanceSub = n.subscribe<std_msgs::Float64>("/left_tread_distance", 15, leftTreadCallback);
        rightTreadDistanceSub = n.subscribe<std_msgs::Float64>("/right_tread_distance", 15, rightTreadCallback);
        
        //odometry_publisher: publish to the location of the base_footprint tracked by tread motion.
        odometry_publisher = n.advertise<nav_msgs::Odometry>("/drivebase_odom", 15); 
        
        ///set_drivebase_odometry : resets the basis of odometry to a new position
        set_odometry = n.aWhdvertiseService("set_drivebase_odometry", &DrivebaseOdometryPublisher::setOdometry, this);
        reset_odometry = n.advertiseService("reset_drivebase_odometry", &DrivebaseOdometryPublisher::resetOdometry, this);
        angle.x = 0;
        angle.y = 0;
        angle.z = 0;
        angle.w = 1;
    }

    ~DrivebaseOdometryPublisher() = default;
    DrivebaseOdometryPublisher(const DrivebaseOdometryPublisher&) = delete;
    DrivebaseOdometryPublisher& operator=(const DrivebaseOdometryPublisher&) = delete;
    DrivebaseOdometryPublisher(DrivebaseOdometryPublisher&&) = delete;
    DrivebaseOdometryPublisher& operator=(DrivebaseOdometryPublisher&) = delete;

    /*****************************************************************************************
    * processOdometry: Main business logic for the node, takes in readings from the arduino,
    *       and publishes them across the network.
    * Preconditions: is subscribed to recieve information from the sensors (tfr_msgs/ArduinoReading)
    * Postconditions: Readings from the arduino are published across the network
    *****************************************************************************************/
    void processOdometry()
    {
        //first we process the data
        auto t_1 = ros::Time::now();
        double d_t = (t_1 - t_0).toSec();
        
        //if this is the first message we skip it to initialize time
        //properly
        if (!t_0.isValid())
        {
            t_0 = ros::Time::now();
            return;
        }

        //basic differential kinematics to get combined velocities
 	double v_right = 1/d_t * rightTreadSpeed; //TODO rename these to TreadDistance instead of TreadSpeed
	double v_left = 1/d_t * leftTreadSpeed;
	double v_ang = (v_right - v_left) / wheel_span;
        double v_lin = (v_right + v_left)/2;
        
        //break into xy components and increment
        double d_angle = v_ang * d_t; 
        rotateQuaternionByYaw(angle, d_angle);

        // yaw (z-axis rotation)
        auto yaw = quaternionToYaw(angle);
        double v_x = v_lin*cos(yaw);
        double v_y = v_lin*sin(yaw);


        double d_x = v_x * d_t;
        x += d_x;

        double d_y = v_y * d_t;
        y += d_y;

        t_0 = t_1;

        //let's package up the message
        nav_msgs::Odometry msg;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = parent_frame;
        msg.child_frame_id = child_frame;

        msg.pose.pose.position.x = x;
        msg.pose.pose.position.y = y;
        msg.pose.pose.position.z = 0;
        msg.pose.pose.orientation = angle;
        msg.pose.covariance = { 1e-1,    0,    0,    0,    0,    0,
            0, 1e-1,    0,    0,    0,    0,
            0,    0, 1e-1,    0,    0,    0,
            0,    0,    0, 1e-1,    0,    0,
            0,    0,    0,    0, 1e-1,    0,
            0,    0,    0,    0,    0, 1e-1 };

        msg.twist.twist.linear.x = v_x;
        msg.twist.twist.linear.y = v_y;
        msg.twist.twist.linear.z = 0;
        msg.twist.twist.angular.x = 0;
        msg.twist.twist.angular.y = 0;
        msg.twist.twist.angular.z = v_ang;
        msg.twist.covariance = { 5e-2,    0,    0,    0,    0,    0,
            0, 5e-2,    0,    0,    0,    0,
            0,    0, 5e-2,    0,    0,    0,
            0,    0,    0, 5e-2,    0,    0,
            0,    0,    0,    0, 5e-2,    0,
            0,    0,    0,    0,    0, 5e-2 };
        //publish the message
        odometry_publisher.publish(msg);
    }


    private:

        ros::Subscriber leftTreadDistanceSub, rightTreadDistanceSub; //the encoder data sub
        ros::Publisher odometry_publisher; //the pub for our processed data
        ros::ServiceServer set_odometry;
        ros::ServiceServer reset_odometry;
        tf2_ros::TransformBroadcaster tf_broadcaster;
        const std::string& parent_frame; //the parent frame of the robot
        const std::string& child_frame; //the child frame of the robot
        const double& wheel_span;
        double leftTreadSpeed, rightTreadSpeed;
        double x; //the x coordinate of the robot (meters)
        double y; //the y coordinate of the robot (meters)
        geometry_msgs::Quaternion angle; 
        const double MAX_XY_DELTA = 0.25;
        const double MAX_THETA_DELTA = 0.065;
        ros::Time t_0;

       
    /******************************************************************************************************
    * setOdometry: Set odometry from fiducial markers, provides smoothing
    * Preconditions: can advertise to set_drivebase_odometry topic, can provide service to 
    *               /set_drivebase_odometry : (tfr_msgs/SetOdometry)
    * Postconditions: angle is updated with its new value, true is returned after the angle has been updated
    *********************************************************************************************************/
    bool setOdometry(tfr_msgs::SetOdometry::Request& request,
            tfr_msgs::SetOdometry::Response& response)
    {

        auto dx = request.pose.position.x - x;
        if (std::abs(dx) >= MAX_XY_DELTA)
            dx = (dx >= 0) ? MAX_XY_DELTA : -MAX_XY_DELTA;
        x += dx;

        auto dy = request.pose.position.y - y;
        if (std::abs(dy) > MAX_XY_DELTA)
            dy = (dy >= 0) ? MAX_XY_DELTA : -MAX_XY_DELTA;
        y += dy;

        auto new_q = getTfQuaternion(request.pose.orientation);
        auto old_q = getTfQuaternion(angle);
        auto delta = new_q * old_q.inverse();
        if (std::abs(delta.getZ()) > MAX_THETA_DELTA)
        {
            auto sign = ( delta.getZ() * delta.getW() >= 0)? 1 : -1;
            tf2::Quaternion rotation{0.0, 0.0, 0.065 * sign, 0.998};
            auto new_value = old_q * rotation;
            angle = getStdQuaternion(new_value);
        }
        else
            angle = request.pose.orientation;
        return true;
    }

    /******************************************************************************************************
    * setOdometry: Set odometry from fiducial markers, provides no smoothing
    * Preconditions: can advertise to set_drivebase_odometry topic, can provide service to 
    *               /set_drivebase_odometry : (tfr_msgs/SetOdometry)
    * Postconditions: outputs message stating that odometry has been reset, angel has been reset based on position,
    *               true is returned after angle has been updated
    *********************************************************************************************************/
    bool resetOdometry(tfr_msgs::SetOdometry::Request& request,
            tfr_msgs::SetOdometry::Response& response)
    {
        ROS_INFO("Drivebase Odometry Publisher: resetting drivebase odometry");

        x = request.pose.position.x;
        y = request.pose.position.y;
        angle = request.pose.orientation;
        return true;
    }
    
    /***************************************************************************
    * tf2::Quaternion getTfQuaternion: create a quaternion based on orientation
    * Preconditions: can determin orientiation
    * Postconditions: a quaternion value is returned
    ****************************************************************************/
    tf2::Quaternion getTfQuaternion(geometry_msgs::Quaternion& q)
    {
        tf2::Quaternion q_0{q.x, q.y, q.z, q.w};
        return q_0;
    }

    /*************************************************************************************************
    * geometry_msgs::Quaternion getStdQuaternion: create a quaternion based on a different quaternion
    * Preconditions: quaternion parameter is initalized
    * Postconditions: a quaternion value is returned
    ***************************************************************************************************/
    geometry_msgs::Quaternion getStdQuaternion(tf2::Quaternion& q_0)
    {
        geometry_msgs::Quaternion q;
        q.x = q_0.getX();
        q.y = q_0.getY();
        q.z = q_0.getZ();
        q.w = q_0.getW();
        return q;
    }

    /*************************************************************************
     * quaternionToYaw: converts a quaterion value to a yaw (z-axis rotation)
     * Preconditions: quaternion parameter is initalized
     * Postconditions: yaw value is returned
     *************************************************************************/
    double quaternionToYaw(geometry_msgs::Quaternion& q)
    {
        // yaw (z-axis rotation)
        double siny = +2.0 * (q.w * q.z + q.x * q.y);
        double cosy = +1.0 - 2.0 * (q.y*q.y + q.z*q.z);  
        double result = atan2(siny, cosy);
        return result;
    }
        
    /*************************************************************************
     * rotateQuaternionByYaw: rotates a quaternion value by a yaw (z-axis rotation)
     * Preconditions: quaternion and yaw parameters are initalized
     * Postconditions: quaternion paramater is updated with the new values
     *************************************************************************/
    void rotateQuaternionByYaw(geometry_msgs::Quaternion& q, double yaw)
    {
        tf2::Quaternion q_0{q.x, q.y, q.z, q.w};
        tf2::Quaternion q_1{};
        q_1.setRPY(0, 0, yaw);
        q_0 *= q_1;
        q.x = q_0.getX();
        q.y = q_0.getY();
        q.z = q_0.getZ();
        q.w = q_0.getW();
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "drivebase_odometry_publisher");

    //NodeHandle is the main access point to communications with the ROS system.
    ros::NodeHandle n;
    std::string parent_frame, child_frame;
    double wheel_span; //wheel_span: the separation of the treads of the robot.
    double rate; //rate: how quickly to publish hz.
    ros::param::param<std::string>("~parent_frame", parent_frame, "odom");
    ros::param::param<std::string>("~child_frame", child_frame, "base_footprint");
    ros::param::param<double>("~wheel_span", wheel_span, 0.5715); // TODO: Get a more accurate wheelspan and define it in one spot.  This value is very important to our turning accuracy from encoders. 
    ros::param::param<double>("~rate", rate, 32.0); //Higher than 32 hz causes frames to get missed. Need to figure out why because the ideal value would be 64 for 1 cm resolution. ALthough 2cm is not bad. 
    DrivebaseOdometryPublisher publisher{n, parent_frame, child_frame, wheel_span};
    ros::Rate loop_rate(rate);
    while(ros::ok())
    {
        publisher.processOdometry(); //readings are published across the network
        ros::spinOnce();
        loop_rate.sleep();

    }
    return 0;
}
