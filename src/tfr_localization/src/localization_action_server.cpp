/*
 * The action server in charge of localizing the robot.
 *
 * Takes in the empty action request, and provides no feedback.
 * Turns until it sees the aruco markers, exits succesfully once it does.
 *
 * Needs access to the image wrapper topic wrapper to fetch images, 
 * name is specified as a parameter.
 *
 * parameters:
 *  - ~turn_speed: how fast to turn [rad/s] (double, default: 0.0)
 *  - ~turn_duration: how long to turn [s] (double, default: 0.0)
 *
 * published topics:
 *  - /cmd_vel publishes to the drivebase (geometry_msgs/Twist)
 * */

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <tfr_msgs/ArucoAction.h>
#include <tfr_msgs/LocalizationAction.h>
#include <tfr_msgs/WrappedImage.h>
#include <tfr_msgs/PoseSrv.h>
#include <tfr_utilities/tf_manipulator.h>
#include <geometry_msgs/Twist.h>

class Localizer
{
    public:
        Localizer(ros::NodeHandle &n, double& velocity, double&
                duration, double& thresh) : 
            aruco{n, "aruco_action_server"},
            server{n, "localize", boost::bind(&Localizer::localize, this, _1) ,false},
            cmd_publisher{n.advertise<geometry_msgs::Twist>("cmd_vel", 5)},
            turn_velocity{velocity},
            turn_duration{duration},
            threshold{thresh}

        {
            ROS_INFO("Localization Action Server: Connecting Aruco");
            if( not aruco.waitForServer(ros::Duration(0))){
                ROS_INFO("Failed to connect to Aruco client");
            } else {
                ROS_INFO("Autonomous Action Server: Connected to digging server");
            }
            

            ROS_INFO("Localization Action Server: Connecting Image Client");
            rear_cam_client = n.serviceClient<tfr_msgs::WrappedImage>("/on_demand/rear_cam/image_raw");
            front_cam_client = n.serviceClient<tfr_msgs::WrappedImage>("/on_demand/front_cam/image_raw");
            tfr_msgs::WrappedImage request{};
            ros::Duration busy_wait{0.1};
            while(!rear_cam_client.call(request)) 
                busy_wait.sleep();
            while(!front_cam_client.call(request))
                busy_wait.sleep();
            ROS_INFO("Localization Action Server: Connected Image Clients");
            ROS_INFO("Localization Action Server: Starting");
            server.start();
            ROS_INFO("Localization Action Server: Started");
        };
        ~Localizer() = default;
        Localizer(const Localizer&) = delete;
        Localizer& operator=(const Localizer&) = delete;
        Localizer(Localizer&&) = delete;
        Localizer& operator=(Localizer&&) = delete;
    private:
        actionlib::SimpleActionServer<tfr_msgs::LocalizationAction> server;
        actionlib::SimpleActionClient<tfr_msgs::ArucoAction> aruco;
        ros::Publisher cmd_publisher;
        ros::ServiceClient rear_cam_client;
        ros::ServiceClient front_cam_client;
        TfManipulator tf_manipulator;
        double turn_velocity;
        double turn_duration;
        double threshold;

        void localize( const tfr_msgs::LocalizationGoalConstPtr &goal)
        {
            ROS_INFO("Localization Action Server: Localize Starting");
            
            //setup
            bool odometry = goal->set_odometry, success = true, set = false;
            geometry_msgs::Twist cmd;
            cmd.angular.z = 0;
            cmd_publisher.publish(cmd);

            ROS_INFO("Localization Action Server: odometry %s, target yaw %f",
                    odometry ? "set": "unset", goal->target_yaw);

            tfr_msgs::LocalizationResult output;
            //loop
            while (true) {

                ROS_INFO("Localization Action Server: iterating");
                if (checkPreempt(output, success)){break;}
                
                if ( not ros::param::getCached("~turn_velocity", turn_velocity)) {turn_velocity = .5;}
                
                tfr_msgs::ArucoResultConstPtr result = getArucoResult();
                
                
                if (result != nullptr && result->number_found > 0) {
                    //we found something
                    geometry_msgs::PoseStamped unprocessed_pose = result->relative_pose;

                    //transform from camera to footprint perspective
                    geometry_msgs::PoseStamped processed_pose;
                    if (!tf_manipulator.transform_pose(unprocessed_pose, processed_pose, "base_footprint")) {
                        ROS_WARN("Localization Action Server: Transform Failed");
                        server.setAborted(output);
                        success = false;
                        break;
                    }

                    ROS_INFO("transformed");
                    
                    processed_pose.pose.position.z = 0;
                    processed_pose.header.stamp = ros::Time::now();

                    //send the message
                    tfr_msgs::PoseSrv::Request request{};
                    request.pose = processed_pose;
                    tfr_msgs::PoseSrv::Response response;
                    output.pose = processed_pose.pose;
                    

                    while(true) {
                        if (checkPreempt(output, success)) {break;} 
                        if(ros::service::call("/localize_bin", request, response)) {
                            ROS_INFO("localized");
                            tfr_msgs::LocalizationResult result;
                            odometry = false;
                            set = true;
                            break;
                        } else {
                            ROS_INFO("Localization Action Server: retrying to localize movable point");
                        }
                    }

                    auto siny = +2.0 * 
                        (processed_pose.pose.orientation.w * processed_pose.pose.orientation.z + 
                         processed_pose.pose.orientation.x * processed_pose.pose.orientation.y);
                    auto cosy = +1.0 - 2.0 * 
                        (processed_pose.pose.orientation.y * processed_pose.pose.orientation.y + 
                         processed_pose.pose.orientation.z * processed_pose.pose.orientation.z );  
                    auto angle = atan2(siny, cosy);

                    auto difference = std::abs(goal->target_yaw) - std::abs(angle);
                    ROS_INFO("Angle %f Difference %f", angle, difference);
                    if (std::abs(difference) < threshold)
                    {
                        ROS_INFO("Difference is %f, which is less than threshold",std::abs(difference));
                        if (!set)
                        {
                            server.setSucceeded(output);
                        }
                        break;
                    } else {
                        /*if (difference > 0 && turn_velocity < 0){
                            ROS_INFO("Setting Velocity +");
                            turn_velocity /= 2;
                            turn_velocity = std::abs(turn_velocity);
                        } else if (difference < 0 && turn_velocity > 0){
                            ROS_INFO("Setting Velocity -");
                            turn_velocity /= 2;
                            turn_velocity = -std::abs(turn_velocity);
                        }*/
                    }
                }
                ROS_INFO("Localization Action Server: turning");

                geometry_msgs::Twist cmd;
                cmd.angular.z = turn_velocity; 
                cmd_publisher.publish(cmd); 
                //Removed the below to try and continuously spin until we localize
              //  ros::Duration(turn_duration).sleep(); 
              //  ROS_INFO("Localization Action Server: stopping");

               // cmd.angular.z = 0;
                //cmd_publisher.publish(cmd);
                //ros::Duration(0.5).sleep(); //This pause seems way too long so I shortened it
            }

            if (success)
                server.setSucceeded(output);
 
            cmd.angular.z = 0;
            cmd_publisher.publish(cmd);
            //teardown
            ROS_INFO("Localization Action Server: Localize Finished");
        }
        
        tfr_msgs::ArucoResultConstPtr getArucoResult(){
            tfr_msgs::ArucoResultConstPtr result = nullptr;
            tfr_msgs::WrappedImage image_wrapper{};
            //rear camera
            if (rear_cam_client.call(image_wrapper))
                result = sendAruco(image_wrapper);
            ROS_INFO("Localization Action Server: rearcam %d", result->number_found);

            //front camera 
            if (result != nullptr && result->number_found == 0 && front_cam_client.call(image_wrapper))
                result = sendAruco(image_wrapper);
            ROS_INFO("Localization Action Server: frontcam %d", result->number_found);
            return result;
        }

        tfr_msgs::ArucoResultConstPtr sendAruco(const tfr_msgs::WrappedImage& msg) {
            tfr_msgs::ArucoGoal goal;
            goal.image = msg.response.image;
            goal.camera_info = msg.response.camera_info;
            //send it to the server
            aruco.sendGoal(goal);
            aruco.waitForResult();
            return aruco.getResult();
        }
        
        bool checkPreempt(tfr_msgs::LocalizationResult& output, bool& success){
            if (server.isPreemptRequested() || !server.isActive() || ! ros::ok()) {
                ROS_INFO("Localization Action Server: preempt requested");
                server.setPreempted(output);
                success = false;
                return true;
            }
            return false;
        }

};



int main(int argc, char** argv)
{
    ros::init(argc, argv, "localization_action_server");
    ros::NodeHandle n{};
    double turn_velocity, turn_duration, threshold;
    ros::param::param<double>("~turn_velocity", turn_velocity, 0.0);
    ros::param::param<double>("~turn_duration", turn_duration, 0.0);
    ros::param::param<double>("~yaw_threshold", threshold, 0.0);
    if (turn_velocity == 0.0 || turn_duration == 0.0)
        ROS_WARN("Localization Action Server: Uninitialized Parameters");
    Localizer localizer(n, turn_velocity, turn_duration, threshold);
    ros::Rate rate(10);
    while(ros::ok()){
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
