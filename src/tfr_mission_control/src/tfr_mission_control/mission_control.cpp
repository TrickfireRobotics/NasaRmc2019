#include "tfr_mission_control/mission_control.h"

#include <pluginlib/class_list_macros.h>

namespace tfr_mission_control {

    /* ========================================================================== */
    /* Constructor/Destructor                                                     */
    /* ========================================================================== */

    /*
     * First thing to get called, NOTE ros::init called by superclass
     * Not needed here
     * */
     MissionControl::MissionControl()
        : rqt_gui_cpp::Plugin(),
        widget(nullptr),
        autonomy{"autonomous_action_server",true},
        teleop{"teleop_action_server",true},
        arm_client{"move_arm", true},
        teleopEnabled{false},
        teleopActive{false}
    {
        setObjectName("MissionControl");
    }

    /* NOTE: I think these QObjects have reference counting and are held by
     * smart pointers under the hood. (I never ever see any qt programmers
     * implement destructors or free memory on QObjects in any tutorial, example
     * code or forum). However I do not know for sure, so I do it here to make
     * myself feel better, and it doesn't seem to complain.
     * */
    MissionControl::~MissionControl()
    {
        delete countdownClock;
        delete widget;
    }

    /* ========================================================================== */
    /* Initialize/Shutdown                                                        */
    /* ========================================================================== */

    /*
     * Actually sets up all of the components on the screen and registers
     * application context. Some of this code is boilerplate generated by that
     * "catkin_creat_rqt" script, and it looked complicated, so I just took it
     * at face value. I'll mark those sections with a //boilerplate annotation.
     * */
    void MissionControl::initPlugin(qt_gui_cpp::PluginContext& context)
    {
        //boilerplate
        widget = new QWidget();
        ui.setupUi(widget);
        if (context.serialNumber() > 1)
        {
            widget->setWindowTitle(widget->windowTitle() +
                    " (" + QString::number(context.serialNumber()) + ")");
        }

        //sets our window active, and makes sure we handle keyboard events
        widget->setFocusPolicy(Qt::StrongFocus);
        widget->installEventFilter(this);
        //boilerplate
        context.addWidget(widget);

        countdownClock = new QTimer(this); //mission clock, runs repeatedly

        // Since rqt creates a Nodelet for the ROS side of things, we can
        // use getNodeHandle() and similar functions instead of creating
        // a NodeHandle member.
        // getMTNodeHandle() gives us a multithreaded NodeHandle.
        com = getMTNodeHandle().subscribe("/com", 5,
            &MissionControl::updateStatus, this);

        joySub = getMTNodeHandle().subscribe<sensor_msgs::Joy>("/joy", 5,
            &MissionControl::joyCallback, this);

        inputReadTimer = getMTNodeHandle().createTimer(ros::Duration(0.1),
            &MissionControl::inputReadTimerCallback, this);

        /* Sets up all the signal/slot connections.
         *
         * For those unfamilair with qt this is the backbone of event driven
         * programming in qt. Signals get generated when an event of interest
         * happens: button pressed/released, timer expires... You can attach
         * signals to 0 or many slots. Slots are functions that get put into an
         * application level queue and processed when time allows. This allows
         * for thread safety.
         *
         * Signals can pass data, however slots they attach to must be able to
         * recieve that data, so signatures need to match. Sometimes you don't
         * want to do this, so you can get around it with lambdas (which I end
         * up doing below)
         *
         * Qt might or might not run different ui objects on many different
         * threads, and rqt will be running on a diffent thread. So in general,
         * if you need to have synchronous state passing, do it through the
         * signal/slot system, and you should be fine.
         * */
        connect(ui.start_button, &QPushButton::clicked,this, &MissionControl::startMission);
        connect(ui.clock_button, &QPushButton::clicked,this, &MissionControl::startManual);
        connect(ui.autonomy_button, &QPushButton::clicked, this,  &MissionControl::goAutonomousMode);
        connect(ui.teleop_button, &QPushButton::clicked, this,  &MissionControl::goTeleopMode);
	connect(ui.set_encoders, &QPushButton::clicked, this,  &MissionControl::resetTurntable);

        //Control
        connect(ui.control_enable_button,&QPushButton::clicked, [this] () {toggleControl(true);});
        connect(ui.control_disable_button,&QPushButton::clicked, [this] () {toggleControl(false);});

        /* Joystick
         * Note, Joystick is not linked to E-stop. setJoystick needs to be replaced with toggleJoystick
         *
         * */

        connect(ui.joy_enable_button,&QPushButton::clicked, [this]() {setJoystick(true);});
        connect(ui.joy_disable_button,&QPushButton::clicked, [this]() {setJoystick(false);});

        //PID
        connect(ui.arm_pid_enable_button,&QPushButton::clicked, [this] () {setArmPID(true);});
        connect(ui.arm_pid_disable_button,&QPushButton::clicked, [this] () {setArmPID(false);});

        connect(countdownClock, &QTimer::timeout, this,  &MissionControl::renderClock);
        connect(this, &MissionControl::emitStatus, ui.status_log,
            &QPlainTextEdit::appendPlainText, Qt::QueuedConnection);
        connect(ui.status_log, &QPlainTextEdit::textChanged, this, &MissionControl::renderStatus);

        /* NOTE Remember how I said parameters of signals/slots need to match
         * up. I want to be able to process teleop commands by passing the code
         * into the perform teleop command. I could write a bunch of functors,
         * to make the parameters match up but that could get tedious, luckily
         * qt5 did a little phenagling which allows us to use lambdas instead.
         * */

        connect(ui.reset_starting_button,&QPushButton::clicked,
                [this] () {performTeleop(tfr_utilities::TeleopCode::RESET_STARTING);});
        connect(ui.reset_dumping_button,&QPushButton::clicked,
                [this] () {performTeleop(tfr_utilities::TeleopCode::RESET_DUMPING);});
        connect(ui.dump_button,&QPushButton::clicked,
                [this] () {performTeleop(tfr_utilities::TeleopCode::DUMP);});
        connect(ui.dig_button,&QPushButton::clicked,
                [this] () {performTeleop(tfr_utilities::TeleopCode::DIG);});

        /* for commands which do the turntable/drivebase we want to kill the
         * motors after release*/
        // arm buttons
        //lower arm
        connect(ui.lower_arm_extend_button,&QPushButton::clicked,
                [this] () {performTeleop(tfr_utilities::TeleopCode::LOWER_ARM_EXTEND);});
        connect(ui.lower_arm_retract_button,&QPushButton::clicked,
                [this] () {performTeleop(tfr_utilities::TeleopCode::LOWER_ARM_RETRACT);});
        //upper arm
        connect(ui.upper_arm_extend_button,&QPushButton::clicked,
                [this] () {performTeleop(tfr_utilities::TeleopCode::UPPER_ARM_EXTEND);});
        connect(ui.upper_arm_retract_button,&QPushButton::clicked,
                [this] () {performTeleop(tfr_utilities::TeleopCode::UPPER_ARM_RETRACT);});
        //scoop
        connect(ui.scoop_extend_button,&QPushButton::clicked,
                [this] () {performTeleop(tfr_utilities::TeleopCode::SCOOP_EXTEND);});
        connect(ui.scoop_retract_button,&QPushButton::clicked,
                [this] () {performTeleop(tfr_utilities::TeleopCode::SCOOP_RETRACT);});
        // raise arm to straight, neutral position above robot
        connect(ui.raise_arm_button,&QPushButton::clicked,
                [this] () {performTeleop(tfr_utilities::TeleopCode::RAISE_ARM);});
        //turntable
        connect(ui.cw_button,&QPushButton::clicked,
                [this] () {performTeleop(tfr_utilities::TeleopCode::CLOCKWISE);});
        connect(ui.ccw_button,&QPushButton::clicked,
                [this] () {performTeleop(tfr_utilities::TeleopCode::COUNTERCLOCKWISE);});
        //reset encoders
        //connect(ui.set_encoders,&QPushButton::clicked,
        //       [this] () {performTeleop(tfr_utilities::TeleopCode::RESET_ENCODER_COUNTS_TO_START);});
        //drivebase
        //forward
        connect(ui.forward_button,&QPushButton::pressed,
                [this] () {performTeleop(tfr_utilities::TeleopCode::FORWARD);});
        connect(ui.forward_button,&QPushButton::released,
                [this] () {performTeleop(tfr_utilities::TeleopCode::STOP_DRIVEBASE);});
        //backward
        connect(ui.backward_button,&QPushButton::pressed,
                [this] () {performTeleop(tfr_utilities::TeleopCode::BACKWARD);});
        connect(ui.backward_button,&QPushButton::released,
                [this] () {performTeleop(tfr_utilities::TeleopCode::STOP_DRIVEBASE);});
        //left
        connect(ui.left_button,&QPushButton::pressed,
                [this] () {performTeleop(tfr_utilities::TeleopCode::LEFT);});
        connect(ui.left_button,&QPushButton::released,
                [this] () {performTeleop(tfr_utilities::TeleopCode::STOP_DRIVEBASE);});
        //right
        connect(ui.right_button,&QPushButton::pressed,
                [this] () {performTeleop(tfr_utilities::TeleopCode::RIGHT);});
        connect(ui.right_button,&QPushButton::released,
                [this] () {performTeleop(tfr_utilities::TeleopCode::STOP_DRIVEBASE);});

        // Set up the action servers.
        ROS_INFO("Mission Control: connecting autonomy");
        autonomy.waitForServer();
        ROS_INFO("Mission Control: connected autonomy");
        ROS_INFO("Mission Control: connecting teleop");
        teleop.waitForServer();
        ROS_INFO("Mission Control: connected teleop");
    }

    /*
     * This get's called before the destructor, and apparently we need to
     * deallocate our ros objects manually here according to the documentation.
     * Publishers and subscribers especially
     * */
    void MissionControl::shutdownPlugin()
    {
        // Using a Qt plugin means we must manually kill ROS entities.
        inputReadTimer.stop();
        com.shutdown();
        joySub.shutdown();
        autonomy.cancelAllGoals();
        autonomy.stopTrackingGoal();
        teleop.cancelAllGoals();
        teleop.stopTrackingGoal();
    }

    /* ========================================================================== */
    /* Methods                                                                    */
    /* ========================================================================== */

    /*
     * Startup utility to reset the turntable
     * */
    void MissionControl::resetTurntable()
    {
        ROS_INFO("Mission Control: Resetting turntable");
        toggleMotors(false);
        std_srvs::Empty::Request req;
        std_srvs::Empty::Response res;
        while(!ros::service::call("/zero_turntable", req, res))
        {
            ros::Duration{INPUT_READ_RATE}.sleep();
        }

	    performTeleop(tfr_utilities::TeleopCode::DRIVING_POSITION);
        toggleMotors(true);
    }

    /* greys/ungreys all teleop buttons, and tell's system whether to process teleop or
     * not
     * */
    void MissionControl::setTeleop(bool value)
    {
        ui.left_button->setEnabled(value);
        ui.right_button->setEnabled(value);
        ui.forward_button->setEnabled(value);
        ui.backward_button->setEnabled(value);
        ui.autonomy_button->setEnabled(value);
        ui.dump_button->setEnabled(value);
        ui.dig_button->setEnabled(value);
        ui.lower_arm_extend_button->setEnabled(value);
        ui.lower_arm_retract_button->setEnabled(value);
        ui.upper_arm_extend_button->setEnabled(value);
        ui.upper_arm_retract_button->setEnabled(value);
        ui.scoop_extend_button->setEnabled(value);
        ui.scoop_retract_button->setEnabled(value);
        teleopEnabled = value;
    }

    /*
     * Toggles the autonomy stop button
     * */
    void MissionControl::setAutonomy(bool value)
    {
        ui.teleop_button->setEnabled(value);
    }

    /*
     * Toggles the control buttons
     * */
    void MissionControl::setControl(bool value)
    {
        ui.control_enable_button->setEnabled(!value);
        ui.control_disable_button->setEnabled(value);
    }

    void MissionControl::setJoystick(bool value)
    {
        ui.joy_enable_button->setEnabled(!value);
        ui.joy_disable_button->setEnabled(value);
    }

    void MissionControl::setArmPID(bool value){
        ros::param::set("/write_arm_values", value);
        ui.arm_pid_enable_button->setEnabled(!value);
        ui.arm_pid_disable_button->setEnabled(value);
    }

    /*
     * Utility for stopping all motors
     * */
    void MissionControl::softwareStop()
    {
        //performTeleop(tfr_utilities::TeleopCode::STOP_DRIVEBASE);
        teleop.waitForResult();
    }

    /* ========================================================================== */
    /* Events                                                                     */
    /* ========================================================================== */
    /*
    * Key press/release events should not directly trigger teleop commands.
    * Instead, we keep track of the key states and use key events to change
    * those key state variables. Separate callbacks can periodically check
    * all the key states and perform teleop (on a different thread).
    *
    * No matter how many times the keys get pressed down (from unintentional
    * key bouncing), teleop will still be performed at a fixed rate.
    *
    * When writing callbacks for quick, repeated events like key presses,
    * it is important that the callback returns as fast as possible.
    * */

    bool MissionControl::eventFilter(QObject* obj, QEvent* event)
    {
        bool keyPress = event->type() == QEvent::KeyPress;

        if (teleopEnabled && (keyPress || event->type() == QEvent::KeyRelease))
        {
            const Qt::Key key = static_cast<Qt::Key>(
                static_cast<QKeyEvent*>(event)->key());

            return processKey(key, keyPress);
        }
        else
        {
            // The event was not a key press/release, so pass it on to
            // something else.
            return QObject::eventFilter(obj, event);
        }
    }

    // This function returns a bool so its use inside of eventFilter() will
    // allow the filter to consume or pass on a key event.
    bool MissionControl::processKey(const Qt::Key key, const bool keyPress)
    {
        TeleopCommand command;
        // It might look tedious, but a switch statement for this few keys
        // is more efficient than using an STL data structure to track keys.
        switch (key)
        {
            // driving controls
            case (Qt::Key_W):
                command = driveForward;
                break;
            case (Qt::Key_S):
                command = driveBackward;
                break;
            case (Qt::Key_A):
                command = driveLeft;
                break;
            case (Qt::Key_D):
                command = driveRight;
                break;
            case (Qt::Key_Shift):
                command = driveStop;
                break;
            // lower arm controls
            case (Qt::Key_U):
                command = lowerArmExtend;
                break;
            case (Qt::Key_J):
                command = lowerArmRetract;
                break;
            // upper arm controls
            case (Qt::Key_I):
                command = upperArmExtend;
                break;
            case (Qt::Key_K):
                command = upperArmRetract;
                break;
            // scoop controls
            case (Qt::Key_O):
                command = scoopExtend;
                break;
            case (Qt::Key_L):
                command = scoopRetract;
                break;
            // turntable controls
            case (Qt::Key_P):
                command = clockwise;
                break;
            case (Qt::Key_Semicolon):
                command = ctrClockwise;
                break;
            // dumping controls
            case (Qt::Key_Y):
                command = dump;
                break;
            case (Qt::Key_H):
                command = resetDumping;
                break;

            default:
                // If we aren't using a key, don't consume its event.
                return false;
        }

        // Lock the mutex to "claim" the control bools.
        std::lock_guard<std::mutex> controlLock(teleopStateMutex);
        teleopState[command] = keyPress;

        // Consume the key event, so nothing else can use it.
        return true;
    } // MissionControl::processKey()

    /* ========================================================================== */
    /* Callbacks                                                                  */
    /* ========================================================================== */
    /*
     * Callback for our status subscriber this happens in the "ros" thread so I
     * need to communicate to the GUI in a safe signal/slot combination.
     *
     * This triggers our custom emitStatus signal, which triggers the built in
     * text update in the text box, a seperate timer event is constantly
     * scrolling the window on a set fast interval.
     * */
    void MissionControl::updateStatus(const tfr_msgs::SystemStatusConstPtr &status)
    {
        const std::string statusStr = getStatusMessage(
            static_cast<StatusCode>(status->status_code), status->data);
        // The status message must be wrapped in a signal-safe "Q" object.
        QString statusMsg = QString::fromStdString(statusStr);
        emit emitStatus(statusMsg);
    }

    /*
     * Callback for incoming joystick messages.
     * Use only with the Microsoft Xbox 360 Wired Controller for Linux.
     *
     * Arm control scheme:
     * https://en.wikipedia.org/wiki/Excavator_controls#ISO_controls
     * */
    void MissionControl::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
    {
        using namespace JoyIndices;
        using namespace JoyBounds;

        if(!teleopEnabled)
        {
            return;
        }
        // Lock the mutex to "claim" the control bools.
        std::lock_guard<std::mutex> controlLock(teleopStateMutex);

        // Driving
        teleopState[driveStop] = joy->buttons[BUTTON_LB];
        teleopState[driveForward] = joy->axes[AXIS_DPAD_Y] > MIN_DPAD;
        teleopState[driveBackward] = joy->axes[AXIS_DPAD_Y] < -1 * MIN_DPAD;
        teleopState[driveLeft] = joy->axes[AXIS_DPAD_X] > MIN_DPAD;
        teleopState[driveRight] = joy->axes[AXIS_DPAD_X] < -1 * MIN_DPAD;

        // Lower arm
        teleopState[lowerArmExtend] = joy->axes[AXIS_RIGHT_Y] < -1 * MIN_RIGHT;
        teleopState[lowerArmRetract] = joy->axes[AXIS_RIGHT_Y] > MIN_RIGHT;

        // Upper arm
        teleopState[upperArmExtend] = joy->axes[AXIS_LEFT_Y] > MIN_LEFT;
        teleopState[upperArmRetract] = joy->axes[AXIS_LEFT_Y] < -1 * MIN_LEFT;

        // Scoop
        teleopState[scoopExtend] = joy->axes[AXIS_RIGHT_X] < -1 * MIN_RIGHT;
        teleopState[scoopRetract] = joy->axes[AXIS_RIGHT_X] > MIN_RIGHT;

        // Turntable
        teleopState[ctrClockwise] = joy->axes[AXIS_LEFT_X] > MIN_LEFT;
        teleopState[clockwise] = joy->axes[AXIS_LEFT_X] < -1 * MIN_LEFT;

        // Dumping
        teleopState[dump] = joy->buttons[BUTTON_X];
        teleopState[resetDumping] = joy->buttons[BUTTON_Y];
    }

    /*
     * This callback periodically reads the input control variables
     * and sends the respective control commands over teleop.
     * */
    void MissionControl::inputReadTimerCallback(const ros::TimerEvent& event)
    {
        if(!teleopEnabled)
        {
            return;
        }

        tfr_utilities::TeleopCode code;
        // Using unique_lock instead of lock_guard so we can control when
        // to unlock, because we want to unlock the booleans before
        // performing teleop.
        std::unique_lock<std::mutex> controlLock(teleopStateMutex);


        // Using the inequality operator != is like doing A XOR B, so that
        // only one can be active at a time. If you try to go forward
        // and backward at the same time, this results in no action.

        // These if blocks are organized by highest to lowest priority.
        if(teleopState[driveStop])
        {
            code = tfr_utilities::TeleopCode::STOP_DRIVEBASE;
        }
        else if(teleopState[lowerArmExtend] != teleopState[lowerArmRetract])
        {
            // Ternary operators are sometimes cleaner than if/else blocks.
            code = teleopState[lowerArmExtend]
                ? tfr_utilities::TeleopCode::LOWER_ARM_EXTEND
                : tfr_utilities::TeleopCode::LOWER_ARM_RETRACT;
        }
        else if(teleopState[upperArmExtend] != teleopState[upperArmRetract])
        {
            code = teleopState[upperArmExtend]
                ? tfr_utilities::TeleopCode::UPPER_ARM_EXTEND
                : tfr_utilities::TeleopCode::UPPER_ARM_RETRACT;
        }
        else if(teleopState[scoopExtend] != teleopState[scoopRetract])
        {
            code = teleopState[scoopExtend]
                ? tfr_utilities::TeleopCode::SCOOP_EXTEND
                : tfr_utilities::TeleopCode::SCOOP_RETRACT;
        }
        else if(teleopState[clockwise] != teleopState[ctrClockwise])
        {
            code = teleopState[clockwise]
                ? tfr_utilities::TeleopCode::CLOCKWISE
                : tfr_utilities::TeleopCode::COUNTERCLOCKWISE;
        }
        else if(teleopState[dump] != teleopState[resetDumping])
        {
            code = teleopState[dump]
                ? tfr_utilities::TeleopCode::DUMP
                : tfr_utilities::TeleopCode::RESET_DUMPING;
        }
        else if(teleopState[driveLeft] != teleopState[driveRight])
        {
            code = teleopState[driveLeft]
                ? tfr_utilities::TeleopCode::LEFT
                : tfr_utilities::TeleopCode::RIGHT;
        }
        else if(teleopState[driveForward] != teleopState[driveBackward])
        {
            code = teleopState[driveForward]
                ? tfr_utilities::TeleopCode::FORWARD
                : tfr_utilities::TeleopCode::BACKWARD;
        }
        else if(teleopActive)
        {
            // Teleop was active, but there is no command to send.
            // Render it inactive and stop the drivebase, because
            // a key/joystick was released.
            controlLock.unlock();
            teleopActive = false;
            performTeleop(tfr_utilities::TeleopCode::STOP_DRIVEBASE);
            return;
        }
        else
        {
            // If there is no action and teleop is inactive, leave early.
            return;
        }

        controlLock.unlock();

        if(code != tfr_utilities::TeleopCode::STOP_DRIVEBASE)
        {
            teleopActive = true;
        }

        performTeleop(code);
    } // inputReadTimerCallback()

    /* ========================================================================== */
    /* Slots                                                                      */
    /* ========================================================================== */

    //self explanitory, starts the time service in executive
    void MissionControl::startTimeService()
    {
        std_srvs::Empty start;
        ros::service::call("start_mission", start);
        //start updating the gui mission clock
        countdownClock->start(500);
    }

    //starts mission in autonomous mode
    void MissionControl::startMission()
    {
        startTimeService();
        goAutonomousMode();
        toggleControl(true);
        toggleMotors(true);
    }

    //starts mission is teleop mode
    void MissionControl::startManual()
    {
        startTimeService();
        goTeleopMode();
        //toggleControl(true);
        //toggleMotors(true);
    }

    //triggers state change into autonomous mode from teleop
    void MissionControl::goAutonomousMode()
    {
        resetTurntable();
        softwareStop();
        setAutonomy(true);
        tfr_msgs::EmptyGoal goal{};
        while (!teleop.getState().isDone()) teleop.cancelAllGoals();
        autonomy.sendGoal(goal);
        setTeleop(false);
        widget->setFocus();

    }

    //triggers state change into teleop from autonomy
    void MissionControl::goTeleopMode()
    {
        setAutonomy(false);
        while (!autonomy.getState().isDone()) autonomy.cancelAllGoals();
        //resetTurntable();
        setTeleop(true);
        softwareStop();
        widget->setFocus();
    }

    // Performs a teleop command asynchronously.
    void MissionControl::performTeleop(const tfr_utilities::TeleopCode& code)
    {
        tfr_msgs::TeleopGoal goal;
        goal.code = static_cast<uint8_t>(code);
        teleop.sendGoal(goal);
    }

    //toggles control for estop (on/off)
    void MissionControl::toggleControl(bool state)
    {
        std_srvs::SetBool request;
        request.request.data = state;
        while(!ros::service::call("toggle_control", request));
        setControl(state);
    }

    //toggles control for estop (on/off)
    void MissionControl::toggleMotors(bool state)
    {
        std_srvs::SetBool request;
        request.request.data = state;
        while(!ros::service::call("toggle_motors", request));
    }

    //self explanitory
    void MissionControl::renderClock()
    {
        tfr_msgs::DurationSrv remaining_time;
        ros::service::call("time_remaining", remaining_time);
        ui.time_display->display(remaining_time.response.duration.toSec());
    }

    //scrolls the status window
    void MissionControl::renderStatus()
    {
        ui.status_log->verticalScrollBar()->setValue(ui.status_log->verticalScrollBar()->maximum());
    }
} // namespace

PLUGINLIB_EXPORT_CLASS(tfr_mission_control::MissionControl,
        rqt_gui_cpp::Plugin)
