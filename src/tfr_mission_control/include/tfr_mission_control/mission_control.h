#pragma once

#include <rqt_gui_cpp/plugin.h>
#include <tfr_mission_control/ui_mission_control.h>
#include "tfr_mission_control/joy_constants.h"

#include <ros/ros.h>
#include <ros/console.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/Joy.h>

#include <tfr_msgs/SystemStatus.h>
#include <tfr_msgs/DurationSrv.h>
#include <tfr_msgs/EmptyAction.h>
#include <tfr_msgs/TeleopAction.h>
#include <tfr_msgs/ArmMoveAction.h>
#include <tfr_msgs/ArmStateSrv.h>

#include <tfr_utilities/teleop_code.h>
#include <tfr_utilities/status_code.h>

#include <cstddef>
#include <mutex>

#include <QWidget>
#include <QObject>
#include <QTimer>
#include <QScrollBar>
#include <QPushButton>
#include <QPlainTextEdit>
#include <QKeyEvent>

namespace tfr_mission_control {

    /* Main entry point for the qt application, contains all of state, business
     * logic, and only has one screen. Uses the associated .ui file for layout
     * and design.
     * */
    class MissionControl : public rqt_gui_cpp::Plugin{

        Q_OBJECT

        public:

            /* ======================================================================== */
            /* Constructor/Destructor                                                   */
            /* ======================================================================== */

            MissionControl();
            ~MissionControl();

            /* ======================================================================== */
            /* Initialize/Shutdown                                                      */
            /* ======================================================================== */

            void initPlugin(qt_gui_cpp::PluginContext& context) override;

            void shutdownPlugin() override;

        private:

            /* ======================================================================== */
            /* Constants                                                                */
            /* ======================================================================== */

            const std::string TAG = "MissionControl";
            //how long to wait without keyborad input before killing the motors(hz)
            //NOTE can cause bouncy keys if user has too long of a delay for
            //repeated keys
            const double MOTOR_INTERVAL = 1000/5;

            /* ======================================================================== */
            /* Variables                                                                */
            /* ======================================================================== */

            //the user interface
            Ui::MissionControlWidget ui;
            //Out application context
            QWidget* widget;
            //the mission timer
            QTimer* countdownClock;

            // Timer for reading the keyboard/joystick state variables
            // to run teleop commands.
            ros::Timer inputReadTimer;
            const double INPUT_READ_RATE = 0.1;

            //The action servers
            actionlib::SimpleActionClient<tfr_msgs::EmptyAction> autonomy;
            actionlib::SimpleActionClient<tfr_msgs::TeleopAction> teleop;
            actionlib::SimpleActionClient<tfr_msgs::ArmMoveAction> arm_client;

            // Subscribes to our custom status messages.
            ros::Subscriber com;
      	    // For subscribing to joy messages for joystick input.
            // Constants for joy array indices are defined in joy_indices.h.
      	    ros::Subscriber joySub;
            // Flag for accepting teleop commands.
            std::atomic<bool> teleopEnabled;

            // Flag for ongoing teleop activity, like any keys being held.
            // Helps teleop send a "stop" command if this flag is false.
            std::atomic<bool> teleopActive;

            enum TeleopCommand
            {
                // Driving
                driveForward = 0, driveBackward,
                driveLeft, driveRight,
                driveStop,
                // Lower arm
                lowerArmExtend, lowerArmRetract,
                // Upper arm
                upperArmExtend, upperArmRetract,
                // Scoop
                scoopExtend, scoopRetract,
                // Turntable
                ctrClockwise, clockwise,
                // Dumping
                dump, resetDumping
            };

            // Idiom to "zero-initialize": to initialize all bools as false.
            bool teleopState[15] = { 0 };
            // This mutex should be locked to claim control over teleopState
            // between different threads.
            std::mutex teleopStateMutex;

            /* ======================================================================== */
            /* Methods                                                                  */
            /* ======================================================================== */
            //utility for shutting down motors and turntable
            void softwareStop();
            //sets system state to teleop ready
            void setTeleop(bool value);
            //sets system state to autonomy ready
            void setAutonomy(bool value);

            //sets control system to output commands
            void setControl(bool value);

            void setJoystick(bool value);


            //sets control system to output commands

            void setArmPID(bool value);

            void resetTurntable();

            /* ======================================================================== */
            /* Events                                                                   */
            /* ======================================================================== */

            // Updates keyboard state variables every time a key event occurs.
            bool eventFilter(QObject *obj, QEvent *event);
            bool processKey(const Qt::Key key, const bool keyPress);

            /* ======================================================================== */
            /* Callbacks                                                                */
            /* ======================================================================== */

            //triggered by incoming status message, and cascades other signals into thread
            //safe gui update
            void updateStatus(const tfr_msgs::SystemStatusConstPtr &status);

            // Responds to joystick messages.
            void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

            // Periodically processes input control variables for teleop.
            void inputReadTimerCallback(const ros::TimerEvent& event);

            protected slots:

                /* ======================================================================== */
                /* Slots                                                                    */
                /* ======================================================================== */

                //MISSION START
                virtual void startMission();    //starts clock and autonomy
                virtual void startManual();     //starts clock and teleop
                virtual void startTimeService();

                //MODE STATE CHANGES
                virtual void goAutonomousMode();
                virtual void goTeleopMode();

                //GUI
                virtual void renderClock();
                virtual void renderStatus();

                //MISC
                virtual void performTeleop(const tfr_utilities::TeleopCode& code);
                virtual void toggleControl(bool state);    //e-stop and start


                // virtual void toggleJoystick(bool state);


                virtual void toggleMotors(bool state);    //e-stop and start

            signals:
                /* ======================================================================== */
                /* Signals                                                                  */
                /* ======================================================================== */

                //used to make cascade work for status update
                void emitStatus(QString status);
    };
} // namespace
