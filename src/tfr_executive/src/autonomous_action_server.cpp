/*
 * DESCRIPTION
 * The executive action server, is the main business logic for autonomous 
 * operation. It is initialized at system startup, and is commanded by the 
 * distributed mission control node. 
 * 
 * The autonomous operation starts when commence autonomous operation 
 * command gets called. This will continue, until:
 * - The action gets into an inoperable state and ceases operation.
 * - The action completes the mission successfully.
 * - The action server is given a command, and is preempted.
 * 
 * Once the mission ceases operation resuming autonomy is not a system 
 * priority. Attempting to return the system to a safe state is.
 * 
 * If preemption or mission failure occurs, it is the duty of the executive 
 * node to preempt all other active action servers it has called, allow them 
 * to return to a safe state on a timeout and promptly exit. 
 * 
 * On startup the action server will be expecting a command to start autonomy, 
 * and initialize autonomous operation. 
 *
 * PRECONDITION
 * The clock service must be up and started, anything else is undefined
 * behavior.
 * 
 * PARAMETERS
 * - ~rate: the rate in hz to check to preemption during long running calls, (double, default: 10)
 * - ~localization: whether to run localization or not (bool, default: true);
 * - ~navigation_to: whether to run navigation_to or not (bool, default: true);
 * - ~digging: whether to run digging or not (bool, default: true);
 * - ~hole: whether to place the hole or not (bool, default: true);
 * - ~navigation_from: whether to run from or not (bool, default: true);
 * - ~dumping: whether to run dumping or not (bool, default: true);
 * 
 * PUBLISHED TOPICS
 * - /com 
 *   - the communication topic
 * */
#include <ros/ros.h>
#include <ros/console.h>
#include <tfr_msgs/EmptyAction.h>
#include <tfr_msgs/DurationSrv.h>
#include <tfr_msgs/LocalizationAction.h>
#include <tfr_msgs/NavigationAction.h>
#include <tfr_msgs/DiggingAction.h>
#include <tfr_msgs/SetOdometry.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <tfr_utilities/location_codes.h>
#include <tfr_utilities/status_code.h>
#include <tfr_utilities/status_publisher.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>

class AutonomousExecutive
{
    public:
        AutonomousExecutive(ros::NodeHandle &n,double f):        
            server{n, "autonomous_action_server", 
                boost::bind(&AutonomousExecutive::autonomousMission, this, _1),
                false},
           
            localizationClient{n, "localize", true},
            navigationClient{n, "navigate", true},
            diggingClient{n, "dig", true},
            dumpingClient{n, "dump", true},
            frequency{f},
            status_publisher{n},
            drivebase_publisher{n.advertise<geometry_msgs::Twist>("cmd_vel", 5)},
            moveClient{n, "move_base", true}
            
        {
            ros::param::param<bool>("~localization_to", LOCALIZATION_TO, true);
            ros::param::param<bool>("~localization_from", LOCALIZATION_FROM, true);
            ros::param::param<bool>("~localization_finish", LOCALIZATION_FINISH, true);
            if (LOCALIZATION_TO || LOCALIZATION_FROM || LOCALIZATION_FINISH)
            {
                ROS_INFO("Autonomous Action Server: Connecting to localization server");
                if( not localizationClient.waitForServer(ros::Duration(10))){
                    ROS_INFO("Failed to connect to localization client");
                } else {
                    ROS_INFO("Autonomous Action Server: Connected to localization server");
                    status_publisher.info(StatusCode::EXC_CONNECT_LOCALIZATION, 1.0);
                }
            }
            ros::param::param<bool>("~navigation_to", NAVIGATION_TO, true);
            ros::param::param<bool>("~navigation_from", NAVIGATION_FROM, true);
            if (NAVIGATION_TO || NAVIGATION_FROM)
            {
                if( not navigationClient.waitForServer(ros::Duration(10))){
                    ROS_INFO("Failed to connect to navigation client");
                }
                else{
                    status_publisher.info(StatusCode::EXC_CONNECT_NAVIGATION, 1.0);
                }
            }
            ros::param::param<bool>("~digging", DIGGING, true);
            if (DIGGING)
            {
                ROS_INFO("Autonomous Action Server: Connecting to digging server");
                if( not diggingClient.waitForServer(ros::Duration(10))){
                    ROS_INFO("Failed to connect to digging client");
                } else {
                    ROS_INFO("Autonomous Action Server: Connected to digging server");
                }
            }
            ros::param::param<bool>("~dumping", DUMPING, true);
            if (DUMPING)
            {
                ROS_INFO("Autonomous Action Server: Connecting to digging server");
                dumpingClient.waitForServer(ros::Duration(10));
                ROS_INFO("Autonomous Action Server: Connected to digging server");
            }
            server.start();
            ROS_INFO("Autonomous Action Server: online, %f",
                    ros::Time::now().toSec());
        }
        ~AutonomousExecutive() = default;
        AutonomousExecutive(const AutonomousExecutive&) = delete;
        AutonomousExecutive& operator=(const AutonomousExecutive&) = delete;
        AutonomousExecutive(AutonomousExecutive&&) = delete;
        AutonomousExecutive& operator=(AutonomousExecutive&&) = delete;
    private:
        /* ACTION DESCRIPTION
         * The main autonomous procedure is long running and needs to be responsive to 
         * preemption. This means that after making time consuming calls, for example 
         * starting navigation, it needs to do so in a non blocking manner, and query
         * for preemption from the user at a set interval. At this interval it is also 
         * appropriate to ask for feedback from the time consuming processes, or check 
         * for success. The definition of long running means any call that starts a 
         * subsystem. 
         * 
         * Also this is an action server so it is responsible for reporting important 
         * state changes and updates to the standard communication channel.
         * 
         * Judiciously following these guidelines, the skeleton of the main procedure is 
         * as follows:
         * 1. Run the localization subsystem.
         * 2. Store the odometry information gathered.
         * 3. Run the navigation subsystem with the drive to mining zone setting.
         * 4. Run the digging action subsystem with the digging time, from the `get_digging_time` service.
         * 5. Run the navigation subsystem with the return from mining zone option.
         * 6. Run the dumping subsystem.
         * 7. Put the system in teleop mode and await instructions.
         * 
         * ACTION COMPONENTS
         * - Goal: none
         * - Feedback: none
         * - Result: none
         * PRECONDITIONS:
         * autonomousMission accepts a pointer to a goal. This goal can be in the
         * form of an action that the robot does. I.E. digging, dumping, navigation
         *
         * POSTCONDITIONS:
         * Upon successfully completing the goal sever will be set to succeed.
         * Upon failure server will be set to aborted
         */ 
    
    
        void autonomousMission(const tfr_msgs::EmptyGoalConstPtr &goal)
        {
            
            ROS_INFO("Autonomous Action Server: mission started");
            if (server.isPreemptRequested() || ! ros::ok())
            {
                server.setPreempted();
                return;
            }

            if (LOCALIZATION_TO)
            {
                ROS_INFO("Autonomous Action Server: commencing Localization To");
                localize(false, 0);
                ROS_INFO("Autonomous Action Server: finished Localization To");
            }

            if (NAVIGATION_TO)
            {
                ROS_INFO("Autonomous Action Server: commencing navigation to");
 
                tfr_msgs::NavigationGoal goal;
                //messages can't support user defined types
                goal.location_code= static_cast<uint8_t>(tfr_utilities::LocationCode::MINING);
                navigationClient.sendGoal(goal);
                //handle preemption
                while ( !navigationClient.getState().isDone() && ros::ok())
                {
                    if (server.isPreemptRequested() || !server.isActive() || ! ros::ok())
                    {
                        navigationClient.cancelAllGoals();
                        moveClient.cancelAllGoals();
                        server.setPreempted();
                        ROS_INFO("Autonomous Action Server: navigation preempted");
                        return;
                    }
                    frequency.sleep();
                }

                if (navigationClient.getState()!=actionlib::SimpleClientGoalState::SUCCEEDED)
                {
                    ROS_INFO("Autonomous Action Server: navigation to failed");
                    server.setAborted();
                    return;
                }
                ROS_INFO("Autonomous Action Server: navigation finished");
            }

            if (DIGGING)
            {
                ROS_INFO("Autonomous Action Server: commencing digging");
                tfr_msgs::DiggingGoal goal{};
                ROS_INFO("Autonomous Action Server: retrieving digging time");
                tfr_msgs::DurationSrv digging_time;
                ros::service::call("digging_time", digging_time);
                ROS_INFO("Autonomous Action Server: digging time retreived %f",
                        digging_time.response.duration.toSec());
                goal.diggingTime = digging_time.response.duration;
                diggingClient.sendGoal(goal);

                //handle preemption
                while (!diggingClient.getState().isDone())
                {
                    if (server.isPreemptRequested()  || !server.isActive() || ! ros::ok())
                    {
                        diggingClient.cancelAllGoals();
                        server.setPreempted();
                        ROS_INFO("Autonomous Action Server: digging preempted");
                        return;
                    }
                    frequency.sleep();
                }
                if (diggingClient.getState()!=actionlib::SimpleClientGoalState::SUCCEEDED)
                {
                    ROS_INFO("Autonomous Action Server: digging failed");
                    server.setAborted();
                    return;
                }
                ROS_INFO("Autonomous Action Server: backing up");
                geometry_msgs::Twist vel;
                vel.linear.x = -.25;
                drivebase_publisher.publish(vel);
                ros::Duration(5.0).sleep();
                vel.linear.x = 0;
                drivebase_publisher.publish(vel);
                ROS_INFO("Autonomous Action Server: digging finished");
            }
            if (LOCALIZATION_FROM)
            {
                ROS_INFO("Autonomous Action Server: commencing Localization From");
                localize(false, 3.14);
                ROS_INFO("Autonomous Action Server: finished Localization From");
            }
            if (NAVIGATION_FROM)
            {
                ROS_INFO("Autonomous Action Server: Connecting to navigation server");
                navigationClient.waitForServer();
                ROS_INFO("Autonomous Action Server: Connected to navigation server");

                ROS_INFO("Autonomous Action Server: commencing Navigation From");
 
                tfr_msgs::NavigationGoal goal;
                //messages can't support user defined types
                goal.location_code=
                    static_cast<uint8_t>(tfr_utilities::LocationCode::DUMPING);
                navigationClient.sendGoal(goal);
                //handle preemption
                while ( !navigationClient.getState().isDone() && ros::ok())
                {
                    if (server.isPreemptRequested() || !server.isActive() || ! ros::ok())
                    {
                        navigationClient.cancelAllGoals();
                        moveClient.cancelAllGoals();
                        server.setPreempted();
                        ROS_INFO("Autonomous Action Server: navigation preempted");
                        return;
                    }
                    frequency.sleep();
                }

                if (navigationClient.getState()!=actionlib::SimpleClientGoalState::SUCCEEDED)
                {
                    ROS_INFO("Autonomous Action Server: navigation to failed");
                    server.setAborted();
                    return;
                }
                ROS_INFO("Autonomous Action Server: finished Navigation From");
            }
            if (LOCALIZATION_FINISH)
            {
                ROS_INFO("Autonomous Action Server: commencing Localize Finish");
                localize(false, 0.0);
                ROS_INFO("Autonomous Action Server: finished Localize Finish");
            }
             if (DUMPING)
            {
                ROS_INFO("Autonomous Action Server: Connecting to dumping server");
                navigationClient.waitForServer();
                ROS_INFO("Autonomous Action Server: Connected to dumping server");

                ROS_INFO("Autonomous Action Server: commencing dumping");
 
                tfr_msgs::EmptyGoal goal;
                dumpingClient.sendGoal(goal);
                //handle preemption
                while ( !dumpingClient.getState().isDone() && ros::ok())
                {
                    if (server.isPreemptRequested() || !server.isActive() || ! ros::ok())
                    {
                        dumpingClient.cancelAllGoals();
                        server.setPreempted();
                        ROS_INFO("Autonomous Action Server: dumping preempted");
                        return;
                    }
                    frequency.sleep();
                }

                if (dumpingClient.getState()!=actionlib::SimpleClientGoalState::SUCCEEDED)
                {
                    ROS_INFO("Autonomous Action Server: dumping failed");
                    server.setAborted();
                    return;
                }
                ROS_INFO("Autonomous Action Server: dumping finished");

            }
            ROS_INFO("Autonomous Action Server: AUTONOMOUS MISSION SUCCESS");
            server.setSucceeded();
        }
        /*
        * PRECONDITIONS:
        * Localize accepts a boolean value for odomtry and a 
        * double value for yaw (motion around axis)
        * POSTCONDITIONS: 
        * If localized goal fails, server is aborted. If 
        * success the loclization is completed.
        */
        void localize(bool set_odometry, double yaw)
        {
            while (not localizationClient.isServerConnected()){
                ROS_INFO("Autonomous Action Server: Localization not connected. Attempting to wait for connection.");
                if (server.isPreemptRequested() || ! ros::ok())
                {
                    server.setPreempted();
                    return;
                }
                if( not localizationClient.waitForServer(ros::Duration(5))){
                    ROS_INFO("Failed to connect to localization client");
                } else {
                    ROS_INFO("Autonomous Action Server: Connected to localization server");
                    status_publisher.info(StatusCode::EXC_CONNECT_LOCALIZATION, 1.0);
                }
            }
            
            
            ROS_INFO("Autonomous Action Server: commencing localization");
            ROS_INFO("Autonomous Action Server: yaw %f", yaw);
            ROS_INFO("Autonomous Action Server: odometryi %d", set_odometry);
            
            
            
            tfr_msgs::LocalizationGoal goal{};
            goal.set_odometry = set_odometry;
            goal.target_yaw = yaw;
            localizationClient.sendGoal(goal);
            //handle preemption
            while (!localizationClient.getState().isDone())
            {
                if (server.isPreemptRequested() || !server.isActive() || ! ros::ok())
                {
                    
                    localizationClient.cancelAllGoals();
                    ROS_INFO("Autonomous Action Server: localization preempted");
                    localizationClient.waitForResult();
                    ROS_INFO("Autonomous Action Server: localization finished");
                    
                    server.setPreempted();
                    return;
                }
                frequency.sleep();
            }
            if (localizationClient.getState()!=actionlib::SimpleClientGoalState::SUCCEEDED)
            {
                ROS_INFO("Autonomous Action Server: localization failed");
                server.setAborted();
                return;
            }
            else
            {

                ROS_INFO("Autonomous Action Server: stabilized odometry");
                std_srvs::Empty empty;
                ros::service::call("/reset_fusion", empty);
           }
            ROS_INFO("Autonomous Action Server: forward localization");
            geometry_msgs::Twist vel;
            vel.linear.x = 0.25;
            drivebase_publisher.publish(vel);
            ros::Duration(1.15).sleep();
            vel.linear.x = 0;
            drivebase_publisher.publish(vel);
            std_srvs::Empty req;
            ros::service::call("/move_base/clear_costmaps", req);
            ROS_INFO("Autonomous Action Server: localization finished");
        }

        actionlib::SimpleActionServer<tfr_msgs::EmptyAction> server;
        actionlib::SimpleActionClient<tfr_msgs::LocalizationAction> localizationClient;
        actionlib::SimpleActionClient<tfr_msgs::NavigationAction> navigationClient;
        actionlib::SimpleActionClient<tfr_msgs::DiggingAction> diggingClient;
        actionlib::SimpleActionClient<tfr_msgs::EmptyAction> dumpingClient;
        actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> moveClient;

       StatusPublisher status_publisher;

        bool LOCALIZATION_TO;
        bool LOCALIZATION_FROM;
        bool LOCALIZATION_FINISH;
        bool NAVIGATION_TO;
        bool NAVIGATION_FROM;
        bool DIGGING;
        bool DUMPING;
        //how often to check for preemption
        ros::Duration frequency;
        ros::Publisher drivebase_publisher;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "autonomous_action_server");
    ros::NodeHandle n{};
    double rate;
    ros::param::param<double>("~rate", rate, 10.0);
    AutonomousExecutive autonomousExecutive{n, 1.0/rate};
    ros::spin();
    return 0;
}
