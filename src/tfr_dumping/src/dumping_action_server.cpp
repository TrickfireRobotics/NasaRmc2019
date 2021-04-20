/*
 * First checks fiducual odometry for distance from dumping location
 *
 * Next, checks drivebase odometry distance
 *
 * Then, moves backwards until the displacement of the drivebase odometry
 * is greater than or equal to the original distance from the dumping
 * location according to fiducial odometry.
 *
 * Next, Raises bin.
 *
 * FInally, lowers bin
 *
 * published topic: -/cmd_vel geometry_msgs/Twist the drivebase velocity
 * */

#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/Twist.h>
#include <image_transport/image_transport.h>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tfr_msgs/ArucoAction.h>
#include <tfr_msgs/BinStateSrv.h>
#include <tfr_msgs/EmptyAction.h>
#include <tfr_msgs/WrappedImage.h>
#include <tfr_utilities/arm_manipulator.h>
#include <tfr_utilities/control_code.h>



class Dumper {
public:
  
  Dumper(ros::NodeHandle &node, const std::string &service_name, float half_robot_length, float adjust_distance):
         server{node, "dump", boost::bind(&Dumper::dumpBinContents, this, _1), false},
         half_robot_length{half_robot_length}, adjust_distance{adjust_distance},
         drivebase_publisher{node.advertise<geometry_msgs::Twist>("cmd_vel", 5)},
         fiducialOdomSubscriber{node.subscribe("/fiducial_odom", 5, &Dumper::fiducialOdomCallback, this)},
         filteredOdomSubscriber{node.subscribe("/odometry/filtered", 5, &Dumper::filteredOdomCallback, this)},
         arm_manipulator{node} {
            ROS_INFO("dumping action server initializing");
            server.start();
            ROS_INFO("dumping action server initialized");
  }

  Dumper(const Dumper &) = delete;
  Dumper &operator=(const Dumper &) = delete;
  Dumper(Dumper &&) = delete;
  Dumper &operator=(Dumper &&) = delete;
  ~Dumper() = default;


private:
  actionlib::SimpleActionServer<tfr_msgs::EmptyAction> server;

  ros::Publisher drivebase_publisher;

  ros::Subscriber fiducialOdomSubscriber;
  ros::Subscriber filteredOdomSubscriber;
  
  float half_robot_length, adjust_distance;
  float currentFiducialDistance, originalFiducialDistance;
  float currentTreadDistanceX, currentTreadDistanceY, actualCurrentTreadDistance;
  float originalTreadDistanceX, originalTreadDistanceY, actualOriginalTreadDistance;
  float movedDistance = 0;

  geometry_msgs::Twist move_cmd{};

  ArmManipulator arm_manipulator;



  // continually upddating but only needed to set original fiducial distance
  void fiducialOdomCallback(const nav_msgs::Odometry &dumpDistance) {
    currentFiducialDistance = dumpDistance.pose.pose.position.x;
  }

  // continually updating actualCurrentTreadDistance so we can know how far the robot has moved
  void filteredOdomCallback(const nav_msgs::Odometry &treadDistance) {
    currentTreadDistanceX = treadDistance.pose.pose.position.x;
    currentTreadDistanceY = treadDistance.pose.pose.position.y;
    // distance formula
    actualCurrentTreadDistance = sqrt((currentTreadDistanceX * currentTreadDistanceX) +
                                      (currentTreadDistanceY * currentTreadDistanceY));
    // this was done because I think the distance formula was resulting in the wrong sign,
    // so I just flipped it
    actualCurrentTreadDistance = -actualCurrentTreadDistance;
  }
  
  
  
  // moves backwards until the moved distance is equal to or greater than the original fiducial odometry distance
  void moveNotBlind() {
    ROS_INFO("Dumping Action Server: Command Recieved, BACKWARD");
    move_cmd.linear.x = -.1;
    drivebase_publisher.publish(move_cmd);
    
    ROS_INFO("Original Tread Distance starting at %f", actualOriginalTreadDistance);
    ROS_INFO("Original Fiducial Distance starting at %f", originalFiducialDistance);
    ROS_INFO("Original Current Tread Distance Starting at %f", actualCurrentTreadDistance);
    
    while (movedDistance < originalFiducialDistance) {
      movedDistance = (actualOriginalTreadDistance - actualCurrentTreadDistance) +
                       half_robot_length + adjust_distance;
    }
    
    ROS_INFO("Original Tread Distance Finished at %f", actualOriginalTreadDistance);
    ROS_INFO("Original Fiducial Distance Finished at %f", originalFiducialDistance);
    ROS_INFO("Original Current Tread Distance Finished at %f", actualCurrentTreadDistance);
    ROS_INFO("Dumping Action Server: BACKWARD finsihed");
  }

  void stopMoving() {
    ROS_INFO("Dumping Action Server: Command Recieved, STOP");
    move_cmd.linear.x = 0;
    drivebase_publisher.publish(move_cmd);
    ROS_INFO("Dumping Action Server: STOP finshed");
  }

  void extendBin() {
    ROS_INFO("Dumping Action Server: Command Recieved, DUMP");
    arm_manipulator.moveLeftBinPosition(5.0);  // Extend left bin actuator
    arm_manipulator.moveRightBinPosition(5.0); // Extend right bin actuator
    ROS_INFO("Dumping Action Server: DUMP finished");
  }

  void retractBin() {
    ROS_INFO("Dumping Action Server: Command Recieved, RESET_DUMPING");
    arm_manipulator.moveLeftBinPosition(1.0);  // Retract left bin actuator
    arm_manipulator.moveRightBinPosition(1.0); // Retract right bin actuator
    ROS_INFO("Dumping Action Server: DUMPING_RESET finished");
  }

  // where the actions of dumping are called
  void dumpBinContents(const tfr_msgs::EmptyGoalConstPtr &goal) {
    // sleep makes sure robot is not moving so fiducial odom is accurate as possible
    // when original fiducial distance is set
    ros::Duration(4.0).sleep();

    originalFiducialDistance = currentFiducialDistance;
    actualOriginalTreadDistance = actualCurrentTreadDistance;

    moveNotBlind();

    stopMoving();
    ros::Duration(2.0).sleep();

    extendBin();
    ros::Duration(10.0).sleep();

    retractBin();
    ros::Duration(10.0).sleep();
  }
};



int main(int argc, char **argv) {
  ros::init(argc, argv, "dumping_action_server");
  ros::NodeHandle n;

  float half_robot_length, adjust_distance;
  ros::param::param<float>("~half_robot_length", half_robot_length, 0);
  ros::param::param<float>("~adjust_distance", adjust_distance, 0);

  std::string service_name;
  ros::param::param<std::string>("~image_service_name", service_name, "");

  Dumper dumper(n, service_name, half_robot_length, adjust_distance);

  ros::Rate rate(10);
  ros::spin();
  return 0;
}
