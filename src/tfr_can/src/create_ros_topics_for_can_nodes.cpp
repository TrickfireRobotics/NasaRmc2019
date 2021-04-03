#include "master.h"
#include "bridge.h"
#include "logger.h"
#include "joint_state_publisher.h"
#include "joint_state_subscriber.h"
#include "entry_publisher.h"
#include "entry_subscriber.h"

#include <thread>
#include <chrono>
#include <memory>
#include <iomanip>

//#include <ros/package.h> // for looking up the location of the current package, in order to find our EDS files.
//#include <ros>

const std::string busname = "can1";

// Set the baudrate of your CAN bus. Most drivers support the values
// "1M", "500K", "125K", "100K", "50K", "20K", "10K" and "5K".
const std::string baudrate = "250K";

const size_t num_devices_required = 4;

const double loop_rate = 32; // 32 Hz
const int slow_loop_rate = 1; // 1 Hz

// CANopen node IDs:
const int SERVO_CYLINDER_LOWER_ARM = 23;
const int SERVO_CYLINDER_UPPER_ARM = 45;
const int SERVO_CYLINDER_SCOOP = 56;
const int TURNTABLE = 1;
const int SERVO_CYLINDER_BIN_LEFT = 77; 
const int SERVO_CYLINDER_BIN_RIGHT = 88; 

// initialize the topics for any Servo Cylinder actuator 
void setupServoCylinderDevice(kaco::Device& device, kaco::Bridge& bridge, std::string& eds_files_path)
{
    
    device.load_dictionary_from_library();
    
    device.load_dictionary_from_eds(eds_files_path + "SC_MC630R11_v_0_7_OD.eds");
    
    PRINT("Set position mode");
    device.set_entry("modes_of_operation", device.get_constant("profile_position_mode"));

    PRINT("Enable operation");
    device.execute("enable_operation");


	// min: 0 -> 0, 
	// max: 47104 -> 6.28==2pi
    auto jspub = std::make_shared<kaco::JointStatePublisher>(device, 0, 47104); 
    bridge.add_publisher(jspub, loop_rate);
    
    auto jssub = std::make_shared<kaco::JointStateSubscriber>(device, 0, 47104);
    bridge.add_subscriber(jssub);
    
    // read the current torque value
    auto iopub_1 = std::make_shared<kaco::EntryPublisher>(device, "torque_actual_value");
    bridge.add_publisher(iopub_1, loop_rate);
    
    auto iosub_1 = std::make_shared<kaco::EntrySubscriber>(device, "torque_actual_value");
    bridge.add_subscriber(iosub_1);
    
    // read/write the max allowed torque value
    auto iopub_2 = std::make_shared<kaco::EntryPublisher>(device, "max_torque");
    bridge.add_publisher(iopub_2, slow_loop_rate);
    
    auto iosub_2 = std::make_shared<kaco::EntrySubscriber>(device, "max_torque");
    bridge.add_subscriber(iosub_2);
    
    
    // read the current velocity value
    auto iopub_3 = std::make_shared<kaco::EntryPublisher>(device, "velocity_actual_value");
    bridge.add_publisher(iopub_3, loop_rate);
    
    // read/write max speed
    auto iopub_4 = std::make_shared<kaco::EntryPublisher>(device, "profile_velocity");
    bridge.add_publisher(iopub_4, slow_loop_rate);
    
    auto iosub_4 = std::make_shared<kaco::EntrySubscriber>(device, "profile_velocity");
    bridge.add_subscriber(iosub_4);

    // read/write heartbeat time interval in milliseconds
    auto iopub_5 = std::make_shared<kaco::EntryPublisher>(device, "producer_heartbeat_time");
    bridge.add_publisher(iopub_5, slow_loop_rate);
    
    auto iosub_5 = std::make_shared<kaco::EntrySubscriber>(device, "producer_heartbeat_time");
    bridge.add_subscriber(iosub_5);
    
    // profile_acceleration
    auto iopub_6 = std::make_shared<kaco::EntryPublisher>(device, "profile_acceleration");
    bridge.add_publisher(iopub_6, slow_loop_rate);
    
    auto iosub_6 = std::make_shared<kaco::EntrySubscriber>(device, "profile_acceleration");
    bridge.add_subscriber(iosub_6);
    
    
    // profile_deceleration
    auto iopub_7 = std::make_shared<kaco::EntryPublisher>(device, "profile_deceleration");
    bridge.add_publisher(iopub_7, slow_loop_rate);
    
    auto iosub_7 = std::make_shared<kaco::EntrySubscriber>(device, "profile_deceleration");
    bridge.add_subscriber(iosub_7);

    auto iopub_8 = std::make_shared<kaco::EntryPublisher>(device, "position_demand");
    bridge.add_publisher(iopub_8, slow_loop_rate);

    auto iopub_9 = std::make_shared<kaco::EntryPublisher>(device, "positioning_option_code");
    bridge.add_publisher(iopub_9, slow_loop_rate);
    
    auto iosub_9 = std::make_shared<kaco::EntrySubscriber>(device, "positioning_option_code");
    bridge.add_subscriber(iosub_9);
}

void setupMaxonDevice(kaco::Device& device, kaco::Bridge& bridge, std::string& eds_files_path)
{
    
    device.load_dictionary_from_library();
    
    device.load_dictionary_from_eds(eds_files_path + "tfr_epos4_config.dcf");
    
    PRINT("Set position mode");
    device.set_entry("modes_of_operation", device.get_constant("profile_position_mode"));

    PRINT("Enable operation");
    device.execute("enable_operation");


    // min: 0 -> 0, 
    // max: 1024 encoder clicks * 4.3 Maxon gear * 70 worm gear = 308224   
    auto jspub = std::make_shared<kaco::JointStatePublisher>(device, -308224, 308224); 
    bridge.add_publisher(jspub, loop_rate);
    
    auto jssub = std::make_shared<kaco::JointStateSubscriber>(device, -308224, 308224); 
    bridge.add_subscriber(jssub);		

    // Read the "statusword" from the CANopen device.
    // The statusword is available on all CANopen DS402 (motion control) devices.
    // It's call statusword because it tells you about the status of the device, and "word" because the data takes up 1 word (2 bytes).
    // 
    // Here is a link (from some company that uses CANopen) explaining what is included in the statusword. This is standard across CANopen DS402 devices. search "Bit 10 (Target Reached)" near top of page.
    // https://us.nanotec.com/products/manual/PD4C_CAN_EN/modes%252Fprofile_position.html/
    // 
    // The reason for reading the statusword is to check when the motor reaches the target position.
    // This way, the digging queue can wait until the arm is in the expected position before moving to the next one.
    auto iopub_1 = std::make_shared<kaco::EntryPublisher>(device, "statusword");
    bridge.add_publisher(iopub_1, loop_rate);

    auto iopub_2 = std::make_shared<kaco::EntryPublisher>(device, "torque_actual_values/torque_actual_value_averaged");
    bridge.add_publisher(iopub_2, loop_rate);

    auto iopub_3 = std::make_shared<kaco::EntryPublisher>(device, "velocity_actual_values/velocity_actual_value_averaged");
    bridge.add_publisher(iopub_3, loop_rate);

}

// Usage: e.g. intToHexString(10) == "A"
std::string intToHexString(int n)
{
    std::stringstream stream;
    stream << std::hex << n;
    std::string result( stream.str() );

    return result;
}

void resetCanopenNode(std::string busname, int node_id)
{ 
    // For reference on the "reset node" message, see: 
    //  https://en.wikipedia.org/wiki/CANopen#Network_management_(NMT)_protocols
    const std::string nmt_command_reset_node = "81";
    const std::string nmt_command_reset_communication = "82";

    std::string node_id_hex = intToHexString(node_id);

    std::system(("cansend " + busname + " 000#" + nmt_command_reset_node + node_id_hex).c_str());
    
    return;
}


int main(int argc, char* argv[]) {

	
	kaco::Master master;
	if (!master.start(busname, baudrate)) {
		ERROR("Starting master failed.");
		return EXIT_FAILURE;
	}

    std::this_thread::sleep_for(std::chrono::seconds(1));
	
    // send the CANopen "reset node" message to each of the servo cylinder actuators. This is done because the actuators send out one heartbeart message when they are powered on or reset. We send the messages here so that Kacanopen will see the heartbeat from  each actuators and realize that they are there.
    resetCanopenNode(busname, SERVO_CYLINDER_SCOOP);
    std::this_thread::sleep_for(std::chrono::milliseconds(250));

    resetCanopenNode(busname, SERVO_CYLINDER_UPPER_ARM);
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
	
    resetCanopenNode(busname, SERVO_CYLINDER_LOWER_ARM);
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
	
   resetCanopenNode(busname, SERVO_CYLINDER_BIN_LEFT);
   std::this_thread::sleep_for(std::chrono::milliseconds(250));
	
    resetCanopenNode(busname, SERVO_CYLINDER_BIN_RIGHT);
    std::this_thread::sleep_for(std::chrono::milliseconds(250));

    // Reset the turntable motor controller.
    // There is a bug which occurs during normal operation, where once the robot is connected to power, the turntable motor controller gets into an error state. It's said that this has something to do with the Xavier booting up.
    // There was also previously a bug with calling resetCanopenNode() when the node is a value less than 16 (because it doesn't end up getting padded to 2 digits. This was fixed in another branch and could be merged.
    // In the meantime we will just write out the message for resetting the turntable here.
    
    std::system(("cansend " + busname + " 000#0215").c_str()); // Go to 'stopped' state
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
    
    std::system(("cansend " + busname + " 000#8015").c_str()); // Go to 'pre-operational' state
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
    
    std::system(("cansend " + busname + " 000#8115").c_str()); // Go to 'reset node' state
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
    
  // std::system(("cansend " + busname + " 000#8115").c_str()); Doesn't work in error state.  Need to figure out a better way. 
  //  PRINT("The turntable reset message just got sent in spot 1");
//	std::this_thread::sleep_for(std::chrono::seconds(10));
//	PRINT("The 10 second wait is over");

	while (master.num_devices()<num_devices_required) {
		ERROR("Number of devices found: " << master.num_devices() << ". Waiting for " << num_devices_required << ".");
		std::this_thread::sleep_for(std::chrono::seconds(2));
	}

	// Create bridge
	ros::init(argc, argv, "canopen_bridge");
	kaco::Bridge bridge;

	for (size_t i=0; i<master.num_devices(); ++i) {

		kaco::Device& device = master.get_device(i);
		device.start();

		PRINT("Found device with node ID "<<device.get_node_id()<<": "<<device.get_entry("manufacturer_device_name"));
		std::string eds_files_path;
		if (ros::param::getCached("~eds_files_path", eds_files_path)) {
				PRINT("Great it worked.");
				PRINT(eds_files_path);
		} else {
		    ERROR("tfr_can could not find the private parameter 'eds_files_path'. Make sure this parameter is getting set in the launch file for tfr_can.");
		}
		
		int deviceId = device.get_node_id();

        if (deviceId == SERVO_CYLINDER_LOWER_ARM)
        {
            setupServoCylinderDevice(device, bridge, eds_files_path);
        }
		
		if (deviceId == SERVO_CYLINDER_UPPER_ARM)
        {
            setupServoCylinderDevice(device, bridge, eds_files_path);
        }
		
		if (deviceId == SERVO_CYLINDER_SCOOP)
        {
            setupServoCylinderDevice(device, bridge, eds_files_path);
        }
		
		if (deviceId == SERVO_CYLINDER_BIN_LEFT)
       {
            setupServoCylinderDevice(device, bridge, eds_files_path);
       }

		if (deviceId == SERVO_CYLINDER_BIN_RIGHT)
        {
            setupServoCylinderDevice(device, bridge, eds_files_path);
        }
		
		if (deviceId == TURNTABLE) //THIS IS WHERE WE LOAD THE EDS LIBRARY
	{
	    setupMaxonDevice(device, bridge, eds_files_path);
	}
		
		
		
		else if (deviceId == 8) //Drivetrain Motor controller
		{
			device.load_dictionary_from_eds(eds_files_path + "roboteq_motor_controllers_v60.eds");
			
		//	ROS_DEBUG_STREAM("tfr_can: case: Device 8" << std::endl);
			
			// Roboteq SBL2360.

			auto iosub_8_1_1 = std::make_shared<kaco::EntrySubscriber>(device, "cmd_cango/cmd_cango_1");
    		bridge.add_subscriber(iosub_8_1_1);

			auto iopub_8_1_2 = std::make_shared<kaco::EntryPublisher>(device, "qry_motcmd/channel_1");
    		bridge.add_publisher(iopub_8_1_2, loop_rate);

			auto iopub_8_1_3 = std::make_shared<kaco::EntryPublisher>(device, "qry_motamps/channel_1");
    		bridge.add_publisher(iopub_8_1_3, loop_rate);
			
			//auto iopub_8_1_4 = std::make_shared<kaco::EntryPublisher>(device, "qry_blrspeed/channel_1");
    		//bridge.add_publisher(iopub_8_1_4, loop_rate);
    		
    		auto iopub_8_1_5 = std::make_shared<kaco::EntryPublisher>(device, "qry_blcntr/qry_blcntr_1");
    		bridge.add_publisher(iopub_8_1_5, loop_rate);
			
            auto iopub_8_1_6 = std::make_shared<kaco::EntryPublisher>(device, "qry_abcntr/channel_1");
    		bridge.add_publisher(iopub_8_1_6, loop_rate);
			
			auto iosub_8_2_1 = std::make_shared<kaco::EntrySubscriber>(device, "cmd_cango/cmd_cango_2");
    		bridge.add_subscriber(iosub_8_2_1);

			auto iopub_8_2_2 = std::make_shared<kaco::EntryPublisher>(device, "qry_motcmd/channel_2");
    		bridge.add_publisher(iopub_8_2_2, loop_rate);

			auto iopub_8_2_3 = std::make_shared<kaco::EntryPublisher>(device, "qry_motamps/channel_2");
    		bridge.add_publisher(iopub_8_2_3, loop_rate);
			
			//auto iopub_8_2_4 = std::make_shared<kaco::EntryPublisher>(device, "qry_blrspeed/channel_2");
    		//bridge.add_publisher(iopub_8_2_4, loop_rate);
    		
    		auto iopub_8_2_5 = std::make_shared<kaco::EntryPublisher>(device, "qry_blcntr/qry_blcntr_2");
    		bridge.add_publisher(iopub_8_2_5, loop_rate);

            auto iopub_8_2_6 = std::make_shared<kaco::EntryPublisher>(device, "qry_abcntr/channel_2");
    		bridge.add_publisher(iopub_8_2_6, loop_rate);
		
		//Reads battery voltage	
		auto iopub_8 = std::make_shared<kaco::EntryPublisher>(device, "qry_volts/v_bat");
    		bridge.add_publisher(iopub_8, loop_rate);

		}
		
		

	}
	PRINT("About to call bridge.run()");
	bridge.run();
	
    master.stop();
    
	return EXIT_SUCCESS;
}
