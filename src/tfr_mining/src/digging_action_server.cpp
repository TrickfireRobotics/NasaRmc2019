/****************************************************************************************
 * File:    digging_action_server.cpp
 * Node:    digging_server
 *
 * Purpose: This is an action server that handles all of the digging subsystem.
 *
 *          This file includes <tfr_msgs/DiggingAction.h>, which is one of seven headers
 *          built by catkin from `tfr_msgs/action/Digging.action`:
 *              devel/include/tfr_msgs/DiggingAction.h
 *              devel/include/tfr_msgs/DiggingActionFeedback.h
 *              devel/include/tfr_msgs/DiggingActionGoal.h
 *              devel/include/tfr_msgs/DiggingActionResult.h
 *              devel/include/tfr_msgs/DiggingFeedback.h
 *              devel/include/tfr_msgs/DiggingGoal.h
 *              devel/include/tfr_msgs/DiggingResult.h
 *
 ***************************************************************************************/

#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <tfr_msgs/DiggingAction.h>  // Note: "Action" is appended
#include <tfr_msgs/ArmMoveAction.h>  // Note: "Action" is appended
#include <tfr_utilities/arm_manipulator.h>
#include <geometry_msgs/Twist.h>
#include <tfr_utilities/teleop_code.h>
#include <actionlib/client/simple_action_client.h>
#include "digging_queue.h"
#include <std_msgs/Float32.h>

typedef actionlib::SimpleActionServer<tfr_msgs::DiggingAction> Server;
typedef actionlib::SimpleActionClient<tfr_msgs::ArmMoveAction> Client;

class DiggingActionServer {
public:
    DiggingActionServer(ros::NodeHandle &nh, ros::NodeHandle &p_nh) :
        priv_nh{p_nh}, queue{priv_nh},
        drivebase_publisher{nh.advertise<geometry_msgs::Twist>("cmd_vel", 5)},
	      // subscribers of the 3 arm actuators and turn table
        turnTableSubscriber{nh.subscribe("/device1/velocity_actual_value", 5, &DiggingActionServer::turnTableVelocityCallback, this)},
        lowerArmSubscriber{nh.subscribe("/device23/velocity_actual_value", 5, &DiggingActionServer::lowerArmVelocityCallback, this)},
        upperArmSubscriber{nh.subscribe("/device45/velocity_actual_value", 5, &DiggingActionServer::upperArmVelocityCallback, this)},
        scoopSubscriber{nh.subscribe("/device56/velocity_actual_value", 5, &DiggingActionServer::scoopVelocityCallback, this)},
        server{nh, "dig", boost::bind(&DiggingActionServer::execute, this, _1), false},
        arm_manipulator{nh}

    {
        server.start();
    }

private:

	/*
	 * Description:
   *
   * When the digging action server receives a goal, it executes one
   * iteration of the digging queue.
   *
	 * Currently the digging action server receives the number of seconds
   * it is allowed to spend on digging in the goal message. This is not
   * currently used, but in the future (TODO) it can be used to control the
   * time spent digging autonomously.
	 *
	 * Pre: There must be accurate measurements of the position of the
   * arm. Digging can start while the arm is turned off-center, but
   * ROS must be aware of this. In other words, as long as the
   * position of the arm is not miscalibrated, it's ok.
	 *
   * During: First set of positions (one for each of the 3 actuators and
   * the turn table) is called. Velocity of the 3 actuators and turn table
   * is monitored and when all are within -0.1 to 0.2 the next set of
   * positions is called. 0.1 seconds is waited after a set of positions is
   * first called to avoid instantly moving to the next set since the
   * velocities will start within -0.1 to 0.1 when a new set is called.
   *
	 * Post: The arm will be in the position specified at the end of the
   * digging queue, OR in an intermediate state if pre-empted (i.e. the
   * goal is cancelled, another goal is sent, or ROS is shutting down).
	 *
	 * Notes: ROS actionlib seemed to have a bug, and it would take
   * as much as 10 seconds to cancel a digging goal.
	 *
	 */
    void execute(const tfr_msgs::DiggingGoalConstPtr& goal)
    {
        ROS_INFO("Start digging queue.");

        std::queue<tfr_mining::DiggingSet> current_queue{queue.sets};

        while (!current_queue.empty())
        {
            tfr_mining::DiggingSet set = current_queue.front();
            current_queue.pop();
            ros::Time now = ros::Time::now();

            ROS_INFO("Starting digging set");

            std::queue<std::vector<double> > current_set{set.states};

            while (!current_set.empty())
            {
                std::vector<double> state = current_set.front();
                current_set.pop();

                //Use arm_manipulator, and NOT MoveIt, to send commands to the arm. The actuators will just move to each of the points in the digging queue, there is no trajectory or other points being generated. There is also no collision checking, so be careful.
                ROS_INFO("Moving arm to position: %.2f %.2f %.2f %.2f", state[0], state[1], state[2], state[3]);
                arm_manipulator.moveArmWithoutPlanningOrLimits(state[0], state[1], state[2], state[3]);

                //Short buffer (0.2 seconds) that robot waits when new positions are first called. Ensures robot doesn't mistakenly think it's done moving instantly because the starting velocities are 0
                ros::Duration(0.20).sleep();

                //This loop checks for the actuators and turn table to be done moving. Will keep looping until they are done moving.
                while (true) {
                  if (turnTableVelocityStatus == 0 && lowerArmVelocityStatus == 0 && upperArmVelocityStatus == 0 && scoopVelocityStatus == 0) {
                    break;
                  }
                }

                //ros::Duration(1.25).sleep();

                ros::Rate rate(10.0);

                if (server.isPreemptRequested() || !ros::ok())
                {
                    ROS_INFO("Preempting digging action server");
                    tfr_msgs::DiggingResult result;
                    server.setPreempted(result);
                    return;
                }

                rate.sleep();
            }
        }
        ROS_INFO("End digging queue.");
        tfr_msgs::DiggingResult result;
        server.setSucceeded(result);
    }

    ros::NodeHandle &priv_nh;
    ros::Publisher drivebase_publisher;
    ros::Subscriber turnTableSubscriber;
    ros::Subscriber lowerArmSubscriber;
    ros::Subscriber upperArmSubscriber;
    ros::Subscriber scoopSubscriber;

    //0 indicates actuators or turn table are not moving, 1 indicates they are moving
    bool turnTableVelocityStatus = 0;
    bool lowerArmVelocityStatus = 0;
    bool upperArmVelocityStatus = 0;
    bool scoopVelocityStatus = 0;

    //callback functions for subscribers, these are called when the subsribers recieve a message from the publishers
    void turnTableVelocityCallback(const std_msgs::Float32& turnTableVelocity) {
      //Outputting velocity for tests
      //ROS_INFO("Turn Table velocity: %f\n", turnTableVelocity.data);

      //flipping status of movement to not moving if within the velocity range
      if (turnTableVelocity.data < 0.0010 && turnTableVelocity.data > -0.0010) {
        turnTableVelocityStatus = 0;
      }
      else {
        turnTableVelocityStatus = 1;
      }
    }
    void lowerArmVelocityCallback(const std_msgs::Float32& lowerArmVelocity) {
      //Outputting velocity for tests
      //ROS_INFO("Lower arm velocity: %f\n", lowerArmVelocity.data);

      //flipping status of movement to not moving if within the velocity range
      if (lowerArmVelocity.data < 0.0010 && lowerArmVelocity.data > -0.0010) {
        lowerArmVelocityStatus = 0;
      }
      else {
        lowerArmVelocityStatus = 1;
      }
    }
    void upperArmVelocityCallback(const std_msgs::Float32& upperArmVelocity) {
      //Outputting velocity for tests
      //ROS_INFO("Upper arm velocity: %f\n", upperArmVelocity.data);

      //flipping status of movement to not moving if within the velocity range
      if (upperArmVelocity.data < 0.0010 && upperArmVelocity.data > -0.0010) {
        upperArmVelocityStatus = 0;
      }
      else {
         upperArmVelocityStatus = 1;
      }
    }
    void scoopVelocityCallback(const std_msgs::Float32& scoopVelocity) {
      //Outputting velocity for tests
      //ROS_INFO("Scoop velocity: %f\n", scoopVelocity.data);

      //flipping status of movement to not moving if within the velocity range
      if (scoopVelocity.data < 0.0010 && scoopVelocity.data > -0.0010) {
        scoopVelocityStatus = 0;
      }
      else {
        scoopVelocityStatus = 1;
      }
    }

    ArmManipulator arm_manipulator;
    tfr_mining::DiggingQueue queue;
    Server server;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "digging_server");
    ros::NodeHandle n;
    ros::NodeHandle p_n("~");

    DiggingActionServer server(n, p_n);
    ros::spin();
    return 0;
}
