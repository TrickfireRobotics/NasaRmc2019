/**
 * Calculates the distance of the robot to the map origin (odom) based on aruco
 * fiducial marker detection.
 *
 * parameters:
 *   ~footprint_frame: The reference frame of the robot_footprint(string,
 *   default="footprint")
 *   ~bin_frame: The reference frame of the bin (string, default="bin_footprint")
 *   ~odom_frame: The reference frame of odom  (string, default="odom")
 *   ~debug: print debugging info (bool, default: false)
 *   ~rate: how fast to process images
 * subscribed topics:
 *   image (sensor_msgs/Image) - the camera topic
 * published topics:
 *   fiducial_odom (geometry_msgs/Odometry)- the odometry topic 
 * */
#include <ros/ros.h>
#include <ros/console.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <tfr_msgs/ArucoAction.h>
#include <tfr_msgs/WrappedImage.h>
#include <tfr_msgs/SetOdometry.h>
#include <tfr_utilities/tf_manipulator.h>
#include <actionlib/client/simple_action_client.h>
#include <robot_localization/SetPose.h>
#include <tf2/convert.h>
#include <std_srvs/Empty.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Scalar.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

class FiducialOdom
{
    public:
        FiducialOdom(ros::NodeHandle& n, 
                const std::string& f_frame, 
                const std::string& b_frame,
                const std::string& o_frame) :
            aruco{"aruco_action_server", true},
            tf_manipulator{},
            footprint_frame{f_frame},
            bin_frame{b_frame},
            odometry_frame{o_frame},
            reset_service{n.advertiseService("/reset_fusion", &FiducialOdom::resetFusion, this)}
        {
            rear_cam_client = n.serviceClient<tfr_msgs::WrappedImage>("/on_demand/rear_cam/image_raw");
            front_cam_client = n.serviceClient<tfr_msgs::WrappedImage>("/on_demand/front_cam/image_raw");
            publisher = n.advertise<nav_msgs::Odometry>("fiducial_odom", 10 );
            ROS_INFO("Fiducial Odom Publisher Connecting to Server");
            aruco.waitForServer();
            ROS_INFO("Fiducial Odom Publisher Connected to Server");
            //fill transform buffer
            ros::Duration(2).sleep();
            //connect to the image clients
            tfr_msgs::WrappedImage request{};
            ros::Duration busy_wait{0.1};
            while(!rear_cam_client.call(request))
                busy_wait.sleep();
            while(!front_cam_client.call(request))
                busy_wait.sleep();
            ROS_INFO("Fiducial Odom Publisher: Connected Image Clients");
        }

        ~FiducialOdom() = default;
        FiducialOdom(const FiducialOdom&) = delete;
        FiducialOdom& operator=(const FiducialOdom&) = delete;
        FiducialOdom(FiducialOdom&&) = delete;
        FiducialOdom& operator=(FiducialOdom&&) = delete;

        bool resetFusion(std_srvs::Empty::Request& request,
                std_srvs::Empty::Response& response)
        {
            ROS_INFO("RESETTING SENSORS");
            processOdometry(true);
            return true;
        }

        void processOdometry(bool reset)
        {
            tfr_msgs::ArucoResultConstPtr result = nullptr;
            tfr_msgs::WrappedImage image_wrapper{};

            //grab an image
            if (rear_cam_client.call(image_wrapper))
                result = sendAruco(image_wrapper);

            if ((result == nullptr || result->number_found == 0) && front_cam_client.call(image_wrapper))
                result = sendAruco(image_wrapper);

            if (result != nullptr && result->number_found !=0)
            {
                geometry_msgs::PoseStamped unprocessed_pose = result->relative_pose;

                //transform from camera to footprint perspective
                geometry_msgs::PoseStamped processed_pose;
                if (!tf_manipulator.transform_pose(unprocessed_pose,
                            processed_pose, footprint_frame))
                    return;

                processed_pose.pose.position.z = 0;

                //we need to express that in terms of odom
                geometry_msgs::Transform relative_bin_transform{};

                //get bin_odom transform
                if (!tf_manipulator.get_transform(relative_bin_transform,
                            bin_frame, odometry_frame))
                    return;

                //footprint_odom transform
                tf2::Transform p_0{};
                tf2::convert(processed_pose.pose, p_0);
                tf2::Transform p_1{};
                tf2::convert(relative_bin_transform, p_1);

                geometry_msgs::Transform relative_transform{};

                //take the  difference between bin->odom and bin->robot
                auto difference = p_1.inverseTimes(p_0.inverse());
                relative_transform = tf2::toMsg(difference);

                //process the odometry
                geometry_msgs::Pose relative_pose{};
                relative_pose.position.x = relative_transform.translation.x;
                relative_pose.position.y = relative_transform.translation.y;
                relative_pose.position.z = 0;
                relative_pose.orientation = relative_transform.rotation;

                // handle odometry data
                nav_msgs::Odometry odom;
                odom.header.frame_id = odometry_frame;
                odom.header.stamp = ros::Time::now();
                odom.child_frame_id = footprint_frame;

                //get our pose and fudge some covariances
                odom.pose.pose = relative_pose;
                odom.pose.covariance = {  1e-1,   0,   0,   0,   0,   0,
                    0,1e-1,   0,   0,   0,   0,
                    0,   0,1e-1,   0,   0,   0,
                    0,   0,   0,1e-1,   0,   0,
                    0,   0,   0,   0,1e-1,   0,
                    0,   0,   0,   0,   0,1e-1}; 
                //fire it off! and cleanup
                publisher.publish(odom);

                //control error propagation in the drivebase odometry publisher removed for debugging
              /* tfr_msgs::SetOdometry odom_req{};
               odom_req.request.pose = odom.pose.pose;
                if (!reset)
                {
                    ros::service::call("/set_drivebase_odometry", odom_req);
                }
                else
               {
                    for (int i = 1; i < 100; i += 1)
                    {
                        ros::service::call("/set_drivebase_odometry", odom_req);
                    } 
                } */  

            }  
        }

    private:
        ros::Publisher publisher;
        ros::ServiceClient rear_cam_client;
        ros::ServiceClient front_cam_client;
        ros::ServiceServer reset_service;
        actionlib::SimpleActionClient<tfr_msgs::ArucoAction> aruco;
        tf2_ros::TransformBroadcaster broadcaster;
        TfManipulator tf_manipulator;

        const std::string& footprint_frame;
        const std::string& bin_frame;
        const std::string& odometry_frame;

        tfr_msgs::ArucoResultConstPtr sendAruco(const tfr_msgs::WrappedImage& msg)
        {
            tfr_msgs::ArucoGoal goal;
            goal.image = msg.response.image;
            goal.camera_info = msg.response.camera_info;
            //send it to the server
            aruco.sendGoal(goal);
            aruco.waitForResult();
            return aruco.getResult();
        }

};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "fiducial_odom_publisher");
    ros::NodeHandle n{};

    std::string footprint_frame, bin_frame, odometry_frame;
    double rate;
    ros::param::param<std::string>("~footprint_frame", footprint_frame, "footprint");
    ros::param::param<std::string>("~bin_frame", bin_frame, "bin_footprint");
    ros::param::param<std::string>("~odometry_frame", odometry_frame, "odom");
    ros::param::param<double>("~rate",rate, 10);

    FiducialOdom fiducial_odom{n, footprint_frame, bin_frame,
        odometry_frame};

    ros::Rate r(rate);
    while(ros::ok())
    {
        fiducial_odom.processOdometry(false);
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
