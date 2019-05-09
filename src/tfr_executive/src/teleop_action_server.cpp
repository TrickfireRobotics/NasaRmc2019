/*
 * The teleop action server is  in charge of processing user commands quickly 
 * and performing smooth remote operation for our operations team. All commands
 * for teleoperation will be processed by this server, except for emergency 
 * stop, which is handled by the control system directly for fast response time.
 * 
 * The commands it supports:
 * - None
 * - Move
 *   - Forward
 *   - Backward
 *   - Turn left
 *   - Turn right
 * - Dig
 *   - Executes digging for some duration calculated by the `get_digging_time` 
 *     service, must support preemption.
 * - Dump
 *   - Raising the dumping bin, currently does not support preemption.
 * - Reset Motor state
 *   - Resets the arm, motor and control system to a safe position, ready to 
 *     recieve commands.
 *
 * PRECONDITION
 * The clock service must be up and started, anything else is undefined
 * behavior.
 * 
 * PARAMETERS 
 * -~linear_velocity: the max linear velocity. (double, default: 0.25)
 * -~angular_velocity: the max angular velocity. (double, default: 0.1)
 * - ~rate: the rate in hz to check to preemption during long running calls, (double, default: 10)
 */ 
#include <ros/ros.h>
#include <ros/console.h>
#include <tfr_utilities/teleop_code.h>
#include <tfr_utilities/control_code.h>
#include <tfr_msgs/TeleopAction.h>
#include <tfr_msgs/DiggingAction.h>
#include <tfr_msgs/EmptySrv.h>
#include <tfr_msgs/BinStateSrv.h>
#include <tfr_msgs/ArmStateSrv.h>
#include <tfr_msgs/DurationSrv.h>
#include <tfr_utilities/arm_manipulator.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <tfr_msgs/ArmMoveAction.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <iostream>
#include <chrono>
#include <thread>
#include <ctime>
#include <ratio>


class TeleopExecutive
{
    public:

        struct DriveVelocity
        {
            private:
                double linear;
                double angular;
            public:
                DriveVelocity(double lin, double ang):
                    linear{lin}, angular{ang}{}
                double getLinear(){return linear;}
                double getAngular(){return angular;}
        };

        TeleopExecutive(ros::NodeHandle &n , DriveVelocity &drive, double f, bool u_d=true) :
            server{n, "teleop_action_server",
                boost::bind(&TeleopExecutive::processCommand, this, _1),
                false},
            drivebase_publisher{n.advertise<geometry_msgs::Twist>("cmd_vel", 5)},
            arm_manipulator{n},
            bin_publisher{n.advertise<std_msgs::Float64>("/bin_position_controller/command", 5)},
            digging_client{n, "dig"},
            arm_client{n, "move_arm"},
            right_bin_pub{n.advertise<std_msgs::Int32>("/device12/set_cmd_cango/cmd_cango_3", 1)},
            left_bin_pub{n.advertise<std_msgs::Int32>("/device12/set_cmd_cango/cmd_cango_2", 1)},
            turntable_pub{n.advertise<std_msgs::Int32>("/device4/set_cmd_cango/cmd_cango_1", 1)},
            lower_arm_pub{n.advertise<std_msgs::Int32>("/device12/set_cmd_cango/cmd_cango_1", 1)},
            upper_arm_pub{n.advertise<std_msgs::Int32>("/device4/set_cmd_cango/cmd_cango_3", 1)},
            scoop_pub{n.advertise<std_msgs::Int32>("/device4/set_cmd_cango/cmd_cango_2", 1)},
            turntable_encoder_publisher{n.advertise<std_msgs::Int32>("/device4/set_cmd_sencntr/counter_1", 1)},
            lower_arm_encoder_publisher{n.advertise<std_msgs::Int32>("/device12/set_cmd_sencntr/counter_2", 1)},
            upper_arm_encoder_publisher{n.advertise<std_msgs::Int32>("/device4/set_cmd_sencntr/counter_3", 1)},
            scoop_encoder_publisher{n.advertise<std_msgs::Int32>("/device4/set_cmd_sencntr/counter_2", 1)},
            drive_stats{drive},
            frequency{f},
            use_digging{u_d}
        {
            if (use_digging){
                ROS_INFO("Teleop Action Server conecting to digging");
                digging_client.waitForServer();
                ROS_INFO("Teleop Action Server connected to digging");
                ROS_INFO("Teleop Action Server conecting to arm");
                arm_client.waitForServer();
                ROS_INFO("Teleop Action Server connected to arm");
            }
            else {
                ROS_INFO("Teleop Action Server not using digging");
            }
            server.start();
            ROS_INFO("Teleop Action Server: Online %f", ros::Time::now().toSec());
        }
        ~TeleopExecutive() = default;
        TeleopExecutive(const TeleopExecutive&) = delete;
        TeleopExecutive& operator=(const TeleopExecutive&) = delete;
        TeleopExecutive(TeleopExecutive&&) = delete;
        TeleopExecutive& operator=(TeleopExecutive&&) = delete;

    private:
    
		
	
		int calculateCommandDuration(int effort, int max_time)
		{
			int command_duration_millis = 
				static_cast<int>(
					(static_cast<double>(effort) / 1000.0) * max_time
				);
			return command_duration_millis;
		}
		
		int calculateCommandDuration(int effort)
		{
			int max_arm_command_duration_millis = 100;
			if (not ros::param::getCached("~max_arm_command_duration_millis", max_arm_command_duration_millis))
			{
				max_arm_command_duration_millis = 100;
			}
			
			return calculateCommandDuration(effort, max_arm_command_duration_millis);
		}
		
		
	
		void stop_arm_movement_after_millis(int millis)
		{
			//std::this_thread::sleep_for(std::chrono::milliseconds(millis));
			
			high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
			
			std::chrono::high_resolution_clock::time_point t2 = t1;
			
			while (std::chrono::duration_cast<duration<std::chrono::milliseconds>>(t2 - t1) < std::chrono::milliseconds(millis))
			{
				t2 = high_resolution_clock::now();
			}
			stop_arm_movement();	
		}
	
        //dev 4
        void stop_arm_movement(){
            std_msgs::Int32 msg;
            msg.data = 0;
            turntable_pub.publish(msg);
            upper_arm_pub.publish(msg);
            scoop_pub.publish(msg);   
        }
        
        //dev 12
        void stop_bin_movement(){
            std_msgs::Int32 msg;
            msg.data = 0;
            right_bin_pub.publish(msg);
            left_bin_pub.publish(msg);
            lower_arm_pub.publish(msg);   
        }

        /*
         *The main callback for processing user commands
         * 
         * ACTION SPECIFICATION
         * The logic of the processing commands is:
         * 1. unpack the message
         * 2. if dumping dump
         * 2. else if digging process asynchronously while checking for preemption
         * 2. else if reset reset
         * 3. send the most relevant driving message (forward, backward, left, right, or stop)
         * 3. exit
         * 
         * This approach avoids additional threading beyond the action server, and should meet our response requirements.
         * 
         * ACTION MESSAGE
         * - Goal: uint8 command
         * - Feedback: none
         * - Result: none
         * */
        void processCommand(const tfr_msgs::TeleopGoalConstPtr& goal)
        {
            geometry_msgs::Twist move_cmd{};
            auto code = static_cast<tfr_utilities::TeleopCode>(goal->code);
            switch(code)
            {
				case (tfr_utilities::TeleopCode::RESET_ENCODER_COUNTS_TO_START):
				{	
				    ROS_INFO("Teleop Action Server: Command Recieved, RESET ENCODERS");
					std_msgs::Int32 turntable_encoder_msg;
					std_msgs::Int32 lower_arm_encoder_msg;
					std_msgs::Int32 upper_arm_encoder_msg;
					std_msgs::Int32 scoop_encoder_msg;
					
					const int32_t turntable_encoder_starting_count = 12880;
					const int32_t lower_arm_encoder_starting_count = 0;
					const int32_t upper_arm_encoder_starting_count = 836;
					const int32_t scoop_encoder_starting_count = 1721;
					
					turntable_encoder_msg.data = turntable_encoder_starting_count;
					lower_arm_encoder_msg.data = lower_arm_encoder_starting_count;
					upper_arm_encoder_msg.data = upper_arm_encoder_starting_count;
					scoop_encoder_msg.data = scoop_encoder_starting_count;
					
					turntable_encoder_publisher.publish(turntable_encoder_msg);
		            lower_arm_encoder_publisher.publish(lower_arm_encoder_msg);
		            upper_arm_encoder_publisher.publish(upper_arm_encoder_msg);
		            scoop_encoder_publisher.publish(scoop_encoder_msg);
					
					break;
					
				}
                case (tfr_utilities::TeleopCode::STOP_DRIVEBASE):
                    {
                        ROS_INFO("Teleop Action Server: Command Recieved, STOP_DRIVEBASE");
                        //all zeros by default
                        drivebase_publisher.publish(move_cmd);
                        break;
                    }

                case (tfr_utilities::TeleopCode::FORWARD):
                    {
                        ROS_INFO("Teleop Action Server: Command Recieved, FORWARD %f",drive_stats.getLinear());
                        move_cmd.linear.x = drive_stats.getLinear();
                        drivebase_publisher.publish(move_cmd);
                        break;
                    }

                case (tfr_utilities::TeleopCode::BACKWARD):
                    {
                        ROS_INFO("Teleop Action Server: Command Recieved, BACKWARD");
                        move_cmd.linear.x = -drive_stats.getLinear();
                        drivebase_publisher.publish(move_cmd);
                        break;
                    }

                case (tfr_utilities::TeleopCode::LEFT):
                    {
                        ROS_INFO("Teleop Action Server: Command Recieved, LEFT");
                        move_cmd.angular.z = drive_stats.getAngular();
                        drivebase_publisher.publish(move_cmd);
                        break;
                    }

                case (tfr_utilities::TeleopCode::RIGHT):
                    {
                        ROS_INFO("Teleop Action Server: Command Recieved, RIGHT");
                        move_cmd.angular.z = -drive_stats.getAngular();
                        drivebase_publisher.publish(move_cmd);
                        break;
                    }

                case (tfr_utilities::TeleopCode::CLOCKWISE):
                    {
                        ROS_INFO("Teleop Action Server: Command Recieved, CLOCKWISE");
                        stop_arm_movement();
                        int effort = 1;
                        if (not ros::param::getCached("~turntable_effort", effort)) {effort = 1;}
						ROS_INFO("Writing effort: %d", effort);
                        std_msgs::Int32 msg;
                        msg.data = -1000;
                        turntable_pub.publish(msg);
						
						int command_duration_millis = calculateCommandDuration(effort);
						stop_arm_movement_after_millis(command_duration_millis);
                        break;
                    }

                case (tfr_utilities::TeleopCode::COUNTERCLOCKWISE):
                    {
                        ROS_INFO("Teleop Action Server: Command Recieved, COUNTERCLOCKWISE");
                        stop_arm_movement();
                        int effort = 1;
                        if (not ros::param::getCached("~turntable_effort", effort)) {effort = 1;}
						ROS_INFO("Writing effort: %d", effort);
                        std_msgs::Int32 msg;
                        msg.data = 1000;
                        turntable_pub.publish(msg);
						
						int command_duration_millis = calculateCommandDuration(effort);
						stop_arm_movement_after_millis(command_duration_millis);
                        break;
                    }
                    
                case (tfr_utilities::TeleopCode::LOWER_ARM_EXTEND):
                    {
                        ROS_INFO("Teleop Action Server: Command Recieved, LOWER_ARM_EXTEND");
                        stop_arm_movement();
						int effort = 1;
                        if (not ros::param::getCached("~arm_lower_effort", effort)) {effort = 1;}
						ROS_INFO("Writing effort: %d", effort);
                        std_msgs::Int32 msg;
                        msg.data = 1000;
                        lower_arm_pub.publish(msg);
						
						int command_duration_millis = calculateCommandDuration(effort);
						stop_arm_movement_after_millis(command_duration_millis);
                        break;
                    }
                    
                    
                case (tfr_utilities::TeleopCode::LOWER_ARM_RETRACT):
                    {
                        ROS_INFO("Teleop Action Server: Command Recieved, LOWER_ARM_RETRACT");
                        stop_arm_movement();
						int effort = 1;
						if (not ros::param::getCached("~arm_lower_effort", effort)) {effort = 1;}
						std_msgs::Int32 msg;
                        msg.data = -1000;
                        lower_arm_pub.publish(msg);
						
						int command_duration_millis = calculateCommandDuration(effort);
						stop_arm_movement_after_millis(command_duration_millis);
                        break;
                    }
                    
                case (tfr_utilities::TeleopCode::UPPER_ARM_EXTEND):
                    {
                        ROS_INFO("Teleop Action Server: Command Recieved, UPPER_ARM_EXTEND");
                        stop_arm_movement();
						int effort = 1;
                        if (not ros::param::getCached("~arm_upper_effort", effort)) {effort = 1;}
						std_msgs::Int32 msg;
                        msg.data = -1000;
                        upper_arm_pub.publish(msg);
						
						int command_duration_millis = calculateCommandDuration(effort);
						stop_arm_movement_after_millis(command_duration_millis);
                        break;
                    }
                    
                case (tfr_utilities::TeleopCode::UPPER_ARM_RETRACT):
                    {
                        ROS_INFO("Teleop Action Server: Command Recieved, UPPER_ARM_RETRACT");
                        stop_arm_movement();
						int effort = 1;
                        if (not ros::param::getCached("~arm_upper_effort", effort)) {effort = 1;}
						std_msgs::Int32 msg;
                        msg.data = 1000;
                        upper_arm_pub.publish(msg);
						
						int command_duration_millis = calculateCommandDuration(effort);
						stop_arm_movement_after_millis(command_duration_millis);
                        break;
                    }
                    
                case (tfr_utilities::TeleopCode::SCOOP_EXTEND):
                    {
                        ROS_INFO("Teleop Action Server: Command Recieved, SCOOP_EXTEND");
                        stop_arm_movement();
                        int effort = 1;
                        if (not ros::param::getCached("~arm_scoop_effort", effort)) {effort = 1;}
						std_msgs::Int32 msg;
                        msg.data = -1000;
                        scoop_pub.publish(msg);
						
						int command_duration_millis = calculateCommandDuration(effort);
						stop_arm_movement_after_millis(command_duration_millis);
                        break;
                    }
                    
                case (tfr_utilities::TeleopCode::SCOOP_RETRACT):
                    {
                        ROS_INFO("Teleop Action Server: Command Recieved, SCOOP_RETRACT");
                        stop_arm_movement();
                        int effort = 1;
                        if (not ros::param::getCached("~arm_scoop_effort", effort)) {effort = 1;}
						std_msgs::Int32 msg;
                        msg.data = 1000;
                        scoop_pub.publish(msg);
						
						int command_duration_millis = calculateCommandDuration(effort);
						stop_arm_movement_after_millis(command_duration_millis);
                        break;
                    }

                case (tfr_utilities::TeleopCode::DIG):
                    {
                        ROS_INFO("Teleop Action Server: commencing digging");
                        tfr_msgs::DiggingGoal goal{};
                        ROS_INFO("Teleop Action Server: retrieving digging time");
                        tfr_msgs::DurationSrv digging_time;
                        ros::service::call("digging_time", digging_time);
                        ROS_INFO("Teleop Action Server: digging time retreived %f",
                                digging_time.response.duration.toSec());
                        goal.diggingTime = digging_time.response.duration;
                        digging_client.sendGoal(goal);

                        //handle preemption
                        while (!digging_client.getState().isDone())
                        {
                            if (server.isPreemptRequested() || ! ros::ok())
                            {
                                digging_client.cancelAllGoals();
                                server.setPreempted();
                                ROS_INFO("Teleop Action Server: digging preempted");
                                return;
                            }
                            frequency.sleep();
                        }
                        ROS_INFO("Teleop Action Server: digging finished");
                        break;
                    }

                case (tfr_utilities::TeleopCode::DUMP):
                    {
                        
                        
                        //drivebase_publisher.publish(move_cmd);
                        ROS_INFO("Teleop Action Server: Command Recieved, DUMP");
                        int effort = 1;
                        if (not ros::param::getCached("~dump_effort", effort)) {effort = 1;}
						std_msgs::Int32 msg;
                        msg.data = effort;
                        right_bin_pub.publish(msg);
                        left_bin_pub.publish(msg);
                        /*//all zeros by default
                        //arm_manipulator.moveArm(0.0, 0.1, 1.07, 1.5);
                        ros::Duration(3.0).sleep();
                        //arm_manipulator.moveArm(0.87, 0.1, 1.07, 1.5);
                        ros::Duration(3.0).sleep();
                        std_msgs::Float64 bin_cmd;
                        bin_cmd.data = tfr_utilities::JointAngle::BIN_MAX;
                        tfr_msgs::BinStateSrv query;
                        while (!server.isPreemptRequested() && ros::ok())
                        {
                            ros::service::call("bin_state", query);
                            using namespace tfr_utilities;
                            if (JointAngle::BIN_MAX -  query.response.state <
                                    0.01)
                                break;
                            bin_publisher.publish(bin_cmd);
                            frequency.sleep();
                        }
                        if (server.isPreemptRequested())
                        {
                            ROS_INFO("Teleop Action Server: DUMP preempted");
                            server.setPreempted();
                            return;
                        }*/
                        ROS_INFO("Teleop Action Server: DUMP finished");
                        break;
                    }

                case (tfr_utilities::TeleopCode::RESET_DUMPING):
                    {
                        //drivebase_publisher.publish(move_cmd);
                        ROS_INFO("Teleop Action Server: Command Recieved, RESET_DUMPING");
                        int effort = 1;
                        if (not ros::param::getCached("~dump_effort", effort)) {effort = 1;}
						std_msgs::Int32 msg;
                        msg.data = -effort;
                        right_bin_pub.publish(msg);
                        left_bin_pub.publish(msg);
                       /* //all zeros by default
                        std_msgs::Float64 bin_cmd;
                        bin_cmd.data = tfr_utilities::JointAngle::BIN_MIN;
                        tfr_msgs::BinStateSrv query;
                        while (!server.isPreemptRequested() && ros::ok())
                        {
                            ros::service::call("bin_state", query);
                            using namespace tfr_utilities;
                            if (query.response.state - JointAngle::BIN_MIN <
                                    0.01)
                                break;
                            bin_publisher.publish(bin_cmd);
                            frequency.sleep();
                        }
                        if (server.isPreemptRequested())
                        {
                            ROS_INFO("Teleop Action Server: DUMPING_RESET preempted");
                            server.setPreempted();
                            return;
                        }*/
                        ROS_INFO("Teleop Action Server: DUMPING_RESET finished");
                        break;
                    }

                case (tfr_utilities::TeleopCode::RESET_STARTING):
                    {
                        ROS_INFO("Teleop Action Server: Command Recieved, RESET_STARTING");
                        /*//all zeros by default
                        drivebase_publisher.publish(move_cmd);
                        //first grab the current state of the arm
                        tfr_msgs::ArmStateSrv query;
                        ros::service::call("arm_state", query);
                        arm_manipulator.moveArm(query.response.states[0], 0.20, 1.0, 1.6);
                        ros::Duration(5.0).sleep();
                        arm_manipulator.moveArm(0, 0.20, 1.0, 1.6);
                        ros::Duration(8.0).sleep();
                        arm_manipulator.moveArm(0, 0.50, 1.0, 1.6);
                        ros::Duration(2.0).sleep();
                        arm_manipulator.moveArm(0, 0.60, 1.0, 1.6);
                        ros::Duration(2.0).sleep();
                        arm_manipulator.moveArm(0, 0.70, 1.0, 1.6);
                        ros::Duration(2.0).sleep();
                        arm_manipulator.moveArm(0, 0.80, 1.0, 1.6);
                        ros::Duration(2.0).sleep();
                        arm_manipulator.moveArm(0, 0.85, 1.0, 1.6);
                        ros::Duration(2.0).sleep();
                        arm_manipulator.moveArm(0, 0.90, 1.0, 1.6);
                        ros::Duration(2.0).sleep();*/

                        ROS_INFO("Teleop Action Server: arm reset finished");
                        break;
                    }
                case (tfr_utilities::TeleopCode::DRIVING_POSITION):
                    {
                        ROS_INFO("Teleop Action Server: Command Recieved, DRIVING_POSITION");
                        //all zeros by default
                        drivebase_publisher.publish(move_cmd);
                        //first grab the current state of the arm
                        //arm_manipulator.moveArm(0, 0.50, 1.07, 1.6);
                        //arm_manipulator.moveArm(3.05, 0.1, 0.98, -1.17);
                        ROS_INFO("Teleop Action Server: arm raise finished");
                        break;
                    }
                case (tfr_utilities::TeleopCode::RAISE_ARM):
                    {
                        ROS_INFO("Teleop Action Server: Command Recieved, RAISE_ARM");
                        tfr_msgs::ArmStateSrv query;
                        ros::service::call("arm_state", query);
                        //all zeros by default
                        drivebase_publisher.publish(move_cmd);
                        //first grab the current state of the arm
                        //arm_manipulator.moveArm(query.response.states[0], 0.10, 1.07, 1.6);
                        ros::Duration(5.0).sleep();
                        ROS_INFO("Teleop Action Server: arm raise finished");
                        break;
                    }


                default:
                    {
                        ROS_WARN("Teleop Action Server: UNRECOGNIZED COMMAND");
                        tfr_msgs::TeleopResult result{};
                        server.setAborted(result);
                        return;
                    }
            }

            tfr_msgs::TeleopResult result{};
            server.setSucceeded(result);
        }

        actionlib::SimpleActionServer<tfr_msgs::TeleopAction> server;
        actionlib::SimpleActionClient<tfr_msgs::DiggingAction> digging_client;
        actionlib::SimpleActionClient<tfr_msgs::ArmMoveAction> arm_client;
        ros::Publisher right_bin_pub;
        ros::Publisher left_bin_pub;
        ros::Publisher turntable_pub;
        ros::Publisher lower_arm_pub;
        ros::Publisher upper_arm_pub;
        ros::Publisher scoop_pub;
        ros::Publisher turntable_encoder_publisher;
		ros::Publisher lower_arm_encoder_publisher;
		ros::Publisher upper_arm_encoder_publisher;
		ros::Publisher scoop_encoder_publisher;
        ros::Publisher drivebase_publisher;
        ArmManipulator arm_manipulator;
        ros::Publisher bin_publisher;
        DriveVelocity &drive_stats;
        //how often to check for preemption
        ros::Duration frequency;
        bool use_digging;

};



int main(int argc, char** argv)
{
    ros::init(argc, argv, "teleop_action_server");
    ros::NodeHandle n{};
    double linear_velocity, angular_velocity, rate;
    bool use_digging; 
    ros::param::param<double>("~linear_velocity", linear_velocity, 0.25);
    ros::param::param<double>("~angular_velocity", angular_velocity, 0.3);
    ros::param::param<double>("~rate", rate, 10.0);
    ros::param::param<bool>("~use_digging", use_digging, true);
    TeleopExecutive::DriveVelocity velocities{linear_velocity, angular_velocity};
    TeleopExecutive teleop{n, velocities, 1.0/rate, use_digging};
    ros::spin();
    return 0;
}
