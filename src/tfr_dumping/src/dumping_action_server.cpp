#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <tfr_msgs/EmptyAction.h>
#include <tfr_msgs/ArucoAction.h>
#include <tfr_msgs/WrappedImage.h>
#include <tfr_msgs/BinStateSrv.h>
#include <tfr_utilities/control_code.h>
#include <tfr_utilities/arm_manipulator.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
/*
 *
 * Its first step is to make sure it can see the aruco board, it will abort the
 * mission if it can't. <todo>
 *
 * published topics:
 *   -/cmd_vel geometry_msgs/Twist the drivebase velocity
 *
 *
 * First checks fiducual odometry for distance from dumping location
 *
 * Next, checks odometry filtered distance
 *
 * Then, moves backwards until the displacement of the average of left tread
 * and right tread distance is greater than or equal to the original distance
 * from the dumping location accirding to fiducial odometry.
 *
 * Next, Raises bin.
 *
 * FInally, lowers bin
 * */





 class Dumper
 {
 public:
   struct DumpingConstraints
   {
   private:
     double min_lin_vel, max_lin_vel, min_ang_vel, max_ang_vel, ang_tolerance;
   public:
     DumpingConstraints(double min_lin, double max_lin,
       double min_ang, double max_ang, double ang_tol):
       min_lin_vel(min_lin), max_lin_vel(max_lin),
       min_ang_vel(min_ang), max_ang_vel(max_ang),
       ang_tolerance(ang_tol){}
       double getMinLinVel() const {return min_lin_vel;}
       double getMaxLinVel() const {return max_lin_vel;}
       double getMinAngVel() const {return min_ang_vel;}
       double getMaxAngVel() const {return max_ang_vel;}
       double getAngTolerance() const {return ang_tolerance;}
     };


     Dumper(ros::NodeHandle &node, const std::string &service_name,
       const DumpingConstraints &c) :
       server{node, "dump", boost::bind(&Dumper::dumpBinContents, this, _1), false},
       drivebase_publisher{node.advertise<geometry_msgs::Twist>("cmd_vel", 5)},
       fiducialOdomSubscriber{node.subscribe("/fiducial_odom", 5, &Dumper::fiducialOdomCallback, this)},
       drivebaseOdomSubscriber{node.subscribe("/drivebase_odom", 5, &Dumper::drivebaseOdomCallback, this)},
       detector{"light_detection"},
       arm_manipulator{node}
       {
         ROS_INFO("dumping action server initializing");
         detector.waitForServer();
         server.start();
         ROS_INFO("dumping action server initialized");
       }

       ~Dumper() = default;
       Dumper(const Dumper&) = delete;
       Dumper& operator=(const Dumper&) = delete;
       Dumper(Dumper&&) = delete;
       Dumper& operator=(Dumper&&) = delete;

     private:
       actionlib::SimpleActionServer<tfr_msgs::EmptyAction> server;
       actionlib::SimpleActionClient<tfr_msgs::EmptyAction> detector;
       // publishers
       ros::Publisher drivebase_publisher;
       // subscribers
       ros::Subscriber fiducialOdomSubscriber;
       ros::Subscriber drivebaseOdomSubscriber;
       // distances
       float currentFiducialDistance;
       float originalFiducialDistance;
       float currentTreadDistance;
       float originalTreadDistance;
       
    
       // keeps track if the first time through the callback function of the fiducialOdom
       bool isFiducialFirst = 0;

       void fiducialOdomCallback(const nav_msgs::Odometry& dumpDistance) {
           currentFiducialDistance = dumpDistance.pose.pose.position.x;
       }

       void drivebaseOdomCallback(const nav_msgs::Odometry& treadDistance) {
         currentTreadDistance = treadDistance.pose.pose.position.x;
       }

       geometry_msgs::Twist move_cmd{};

       ArmManipulator arm_manipulator;

       /*
       Moves backwards 0.5 meters, then stops
       Then extends and retracts the actuator bin
       */
  
     /*
       * back up slowwwwly we can see now
       */
       void moveNotBlind()
       {
         ROS_INFO("Dumping Action Server: Command Recieved, BACKWARD");
         move_cmd.linear.x = -.1;
         drivebase_publisher.publish(move_cmd);
         // wait until the the robot has moved the same distance the fiducial odometry originally read
         ROS_INFO("Original Tread Distance starting at %f", originalTreadDistance);
         ROS_INFO("Original Fiducial Distance starting at %f", originalFiducialDistance);
         ROS_INFO("Original Current Tread Distance Starting at %f", currentTreadDistance);
         while(((originalTreadDistance-currentTreadDistance)) < originalFiducialDistance) {}
         ROS_INFO("Original Tread Distance Finished at %f", originalTreadDistance);
         ROS_INFO("Original Fiducial Distance Finished at %f", originalFiducialDistance);
         ROS_INFO("Original Current Tread Distance Finished at %f", currentTreadDistance);
         ROS_INFO("Dumping Action Server: BACKWARD finsihed");
       }

       /*
       *  Stop Moving
       */
       void stopMoving()
       {
         ROS_INFO("Dumping Action Server: Command Recieved, STOP");
         move_cmd.linear.x = 0;
         drivebase_publisher.publish(move_cmd);
         ROS_INFO("Dumping Action Server: STOP finshed");
       }
  
  
       void dumpBinContents(const tfr_msgs::EmptyGoalConstPtr &goal)
       {
         // make sure robot is not moving so fiducial odom is accurate as possible when original fiducial distance is defined
         ros::Duration(4.0).sleep();
         
         originalFiducialDistance = currentFiducialDistance;
         originalTreadDistance = currentTreadDistance;
        
         moveNotBlind();

         stopMoving();
         ros::Duration(2.0).sleep();

         extendBin();
         ros::Duration(10.0).sleep();

         retractBin();
         ros::Duration(10.0).sleep();
       }

       void extendBin()
       {
         ROS_INFO("Dumping Action Server: Command Recieved, DUMP");
         arm_manipulator.moveLeftBinPosition(5.0); // Extend left bin actuator
         arm_manipulator.moveRightBinPosition(5.0); // Extend right bin actuator
         ROS_INFO("Dumping Action Server: DUMP finished");
       }

       void retractBin()
       {
         ROS_INFO("Dumping Action Server: Command Recieved, RESET_DUMPING");
         arm_manipulator.moveLeftBinPosition(1.0); // Retract left bin actuator
         arm_manipulator.moveRightBinPosition(1.0); // Retract right bin actuator
         ROS_INFO("Dumping Action Server: DUMPING_RESET finished");
       }

     };

     /*
     initialize ros
     create Dumper object
     */
     int main(int argc, char **argv)
     {
       ros::init(argc, argv, "dumping_action_server");
       ros::NodeHandle n;
       double min_lin_vel, max_lin_vel, min_ang_vel, max_ang_vel, ang_tolerance;
       ros::param::param<double>("~min_lin_vel",min_lin_vel, 0);
       ros::param::param<double>("~max_lin_vel",max_lin_vel, 0);
       ros::param::param<double>("~min_ang_vel",min_ang_vel, 0);
       ros::param::param<double>("~max_ang_vel",max_ang_vel, 0);
       ros::param::param<double>("~ang_tolerance",ang_tolerance, 0);
       std::string service_name;
       ros::param::param<std::string>("~image_service_name", service_name, "");
       Dumper::DumpingConstraints constraints(min_lin_vel, max_lin_vel,
         min_ang_vel, max_ang_vel, ang_tolerance);
       Dumper dumper(n, service_name, constraints);
       ros::spin();
       return 0;
     }
