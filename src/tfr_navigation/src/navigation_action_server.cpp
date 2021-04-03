#include <ros/ros.h>
#include <ros/console.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <tfr_msgs/NavigationAction.h>
#include <tfr_msgs/PoseSrv.h>
#include <tfr_utilities/location_codes.h>
#include <boost/bind.hpp>
#include <cstdint>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
class Navigator
{ 
public:
    /* 
        * Immutable struct of geometry constraints for the goal selection
        * algorithm
        * */
    struct GeometryConstraints 
    {
        public:
            GeometryConstraints(double d, double f) :
                safe_mining_distance{d}, finish_line{f}{};

            double get_safe_mining_distance() const
            {
                return safe_mining_distance;
            }

            double get_finish_line() const
            {
                return finish_line;
            }
        private:
            //The distance to travel from the bin
            double safe_mining_distance;
            //the distance from the bin the finish line is
            double finish_line;
    };

    //constructor
    Navigator(ros::NodeHandle& n,
            const GeometryConstraints &c,
            const double& height_adj,
            const std::string &bin_f):
        node{n}, 
        rate{10},
        height_adjustment{height_adj},
        constraints{c},
        server{n, "navigate", boost::bind(&Navigator::navigate, this, _1) ,false},
        nav_stack{n, "move_base", true},
        bin_frame{bin_f}
    {
        ROS_DEBUG("Navigation server constructed %f", ros::Time::now().toSec());
        

        //NOTE ros is not really big on runtime exceptions,  I'll post an annoying
        //warning at startup
        if (    constraints.get_safe_mining_distance() < 0 || 
                constraints.get_finish_line() < 0)
        {
            ROS_WARN("Mining constraints should be positive %f", 
                    ros::Time::now().toSec());
            ROS_WARN("    safe_mining_distance: %f",
                    constraints.get_safe_mining_distance());
            ROS_WARN("    finish_line: %f",
                    constraints.get_finish_line());
        }

        ROS_INFO("Navigation server connecting to nav_stack");
        nav_stack.waitForServer();
        ROS_INFO("Navigation server connected to nav_stack");
        server.start();
        ROS_INFO("Navigation server awaiting connection");
    }

    ~Navigator(){}
    Navigator(const Navigator&) = delete;
    Navigator& operator=(const Navigator&) = delete;
    Navigator(Navigator&&) = delete;
    Navigator& operator=(Navigator&&) = delete;



private:
    /*
        *  Goal: 
        *      -uint8_t code corresponding to where we want to navigate. Goal list is
        *      described in Navigation.action in the tfr_msgs package and in
        *      tfr_utilities/include/tfr_utilities/location_codes.h
        *  Feedback:
        *      -none
        * */

    //navigate
    void navigate(const tfr_msgs::NavigationGoalConstPtr &goal)
    {
        auto code = static_cast<tfr_utilities::LocationCode>(goal->location_code);
        ROS_INFO("Navigation server started");
        //start with initial goal
        move_base_msgs::MoveBaseGoal nav_goal{};
        initializeGoal(nav_goal, code);
        nav_stack.sendGoal(nav_goal);

        //test for completion
        while (true)
        {
            ROS_INFO("Navigation Server Preempt Request:%d Active:%d", server.isPreemptRequested(), server.isActive());

            //Deal with preemption or error
            if (server.isPreemptRequested() || !ros::ok()) 
            {
                ROS_INFO("%s: preempted", ros::this_node::getName().c_str());
                nav_stack.cancelAllGoals();
                server.setPreempted();
                return;
            }
            else
            {
                rate.sleep();
            }
            ROS_INFO("Navigation Stack state %s", nav_stack.getState().toString().c_str());
            if (nav_stack.getState().isDone())
                break;
        }

        if (nav_stack.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            server.setSucceeded();
        else
            server.setAborted();
        
        ROS_INFO("Navigation server finished");
    }        

    ros::NodeHandle& node;
    //NOTE delegate initialization of server to ctor
    actionlib::SimpleActionServer<tfr_msgs::NavigationAction> server;
    //NOTE delegate initialization of server to ctor
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> nav_stack;


    //parameters
    std::string frame_id{};
    std::string bin_frame{};
    std::string action_name{};
    ros::Rate rate;
    const double& height_adjustment;
    
    //the constraints to the problem
    const GeometryConstraints &constraints;

    /*  initializeGoal()
        * Set the nav goal to position/orientation associated with
        * the given location code with the reference frame of the dumping bin
        * 
        * pre: - bin_frame contains the frame id of the bin
        *      - constraints .safe_mining_distance and .finish_line are both set
        *      - height_adjustment is set
        * 
        * post: - nav_goal stamped with current time, reference set to bin, and
        *         pose set to location indicated by location code
        *
        **/
    void initializeGoal( move_base_msgs::MoveBaseGoal& nav_goal, 
            const tfr_utilities::LocationCode& goal)
    {
        //set reference frame
        nav_goal.target_pose.header.frame_id = bin_frame;
        nav_goal.target_pose.header.stamp = ros::Time::now();
        
        //set translation goal
        tf2::Quaternion quat;
        switch(goal)
        {
            case(tfr_utilities::LocationCode::MINING): //navigating to mining area
                nav_goal.target_pose.pose.position.x = constraints.get_safe_mining_distance();
                nav_goal.target_pose.pose.position.z = height_adjustment;
                
                nav_goal.target_pose.pose.orientation.w = 1; //No rotation
                quat.setRPY(0, 0, 0); //(roll, pitch, yaw)
                
                //convert quaternion from tf datatype to msg datatype 
                tf2::convert(quat, nav_goal.target_pose.pose.orientation); 
                break;
            case(tfr_utilities::LocationCode::DUMPING):
                nav_goal.target_pose.pose.position.x = constraints.get_finish_line();
                nav_goal.target_pose.pose.position.z = height_adjustment;
                nav_goal.target_pose.pose.orientation.z = 1; //Face backwards to the bin
                break;
            case(tfr_utilities::LocationCode::UNSET):
                //leave it alone
                break;
            default:
                ROS_WARN("location_code %u not recognized",
                        static_cast<uint8_t>(goal));
        }
    }
};
//end of class Navigator


/**
 * Main entry point for the navigation action server, spins up the server and
 * awaits callbacks.
 * */
int main(int argc, char** argv)
{
    ros::init(argc, argv, "navigation_action_server");
    ros::NodeHandle n;
    double safe_mining_distance, finish_line, height_adjustment;
    std::string bin_frame;

    ros::param::param<double>("~safe_mining_distance", safe_mining_distance, 5.1);
    ros::param::param<double>("~finish_line", finish_line, 0.84);
    ros::param::param<double>("~height_adjustment", height_adjustment, -.16);
    ros::param::param<std::string>("~bin_frame", bin_frame, "bin_footprint");

    Navigator::GeometryConstraints 
        constraints(safe_mining_distance, finish_line);
    Navigator navigator(n, constraints, height_adjustment, bin_frame);
    ros::spin();
    return 0;
}
