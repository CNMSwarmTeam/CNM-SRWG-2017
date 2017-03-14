//#INCLUDES
//--------------------------------------------
#include <ros/ros.h>

// ROS libraries
#include <angles/angles.h>
#include <random_numbers/random_numbers.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

// ROS messages
#include <std_msgs/Float32.h>
#include <std_msgs/Int16.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/Range.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <apriltags_ros/AprilTagDetectionArray.h>

// Include Controllers
#include "PickUpController.h"
#include "DropOffController.h"
#include "SearchController.h"

// To handle shutdown signals so the node quits
// properly in response to "rosnode kill"
#include <ros/ros.h>
#include <signal.h>

using namespace std;

// Variables
//--------------------------------------------
random_numbers::RandomNumberGenerator* rng;     // Random number generator... should be DELETED later

// STATE MACHINE STATE CONSTANTS (for mobility SWITCH)
//--------------------------------------------
#define STATE_MACHINE_TRANSFORM 0
#define STATE_MACHINE_ROTATE 1
#define STATE_MACHINE_SKID_STEER 2
#define STATE_MACHINE_PICKUP 3
#define STATE_MACHINE_DROPOFF 4

int stateMachineState = STATE_MACHINE_TRANSFORM; //stateMachineState keeps track of current state in mobility state machine

const unsigned int mapHistorySize = 500;        // How many points to use in calculating the map average position

//GEOMETRY_MSG::POSE2D CLASS OBJECTS            //x, y, theta public variables (vectors)
//--------------------------------------------
geometry_msgs::Pose2D currentLocation;          //current location of robot
geometry_msgs::Pose2D currentLocationMap;       //current location on MAP
geometry_msgs::Pose2D currentLocationAverage;   //???
geometry_msgs::Pose2D goalLocation;             //location to drive to

geometry_msgs::Pose2D centerLocation;           //location of center location
geometry_msgs::Pose2D centerLocationMap;        //location of center on map
geometry_msgs::Pose2D centerLocationOdom;       //location of center ODOM

geometry_msgs::Pose2D mapLocation[mapHistorySize]; //An array in which to store map positions

std_msgs::String msg;                           //std_msgs shares current STATE_MACHINE STATUS in mobility state machine
geometry_msgs::Twist velocity;                  //Linear and Angular Velocity Expressed as a Vector

//Controller Class Objects
//--------------------------------------------
PickUpController pickUpController;
DropOffController dropOffController;
SearchController searchController;

int currentMode = 0;
float mobilityLoopTimeStep = 0.1;               // time between the mobility loop calls
float status_publish_interval = 1;
float killSwitchTimeout = 10;
bool targetDetected = false;                    //for target detection    (seen a target)
bool targetCollected = false;                   //for target collection   (picked up a target)
bool avoidingObstacle = false;

// Set true when the target block is less than targetDist so we continue
// attempting to pick it up rather than switching to another block in view.
bool lockTarget = false;

// Failsafe state. No legitimate behavior state. If in this state for too long
// return to searching as default behavior.
bool timeOut = false;

// Set to true when the center ultrasound reads less than 0.14m. Usually means
// a picked up cube is in the way.
bool blockBlock = false;

// Set true when we are insie the center circle and we need to drop the block,
// back out, and reset the boolean cascade.
bool reachedCollectionPoint = false;

// used for calling code once but not in main
bool init = false;

// used to remember place in mapAverage array
int mapCount = 0;

//Function Calls
//--------------------------------------------

//sets speed
void sendDriveCommand(double linearVel, double angularVel);

void openFingers();                             // Open fingers to 90 degrees
void closeFingers();                            // Close fingers to 0 degrees
void raiseWrist();                              // Return wrist back to 0 degrees
void lowerWrist();                              // Lower wrist to 50 degrees
void mapAverage();                              // constantly averages last 100 positions from map

//PUBLISHER/SUBSCRIBER/TIMER
//--------------------------------------------

// Publishers
ros::Publisher stateMachinePublish;
ros::Publisher status_publisher;
ros::Publisher fingerAnglePublish;
ros::Publisher wristAnglePublish;
ros::Publisher infoLogPublisher;
ros::Publisher driveControlPublish;

// Subscribers
ros::Subscriber joySubscriber;
ros::Subscriber modeSubscriber;
ros::Subscriber targetSubscriber;
ros::Subscriber obstacleSubscriber;
ros::Subscriber odometrySubscriber;
ros::Subscriber mapSubscriber;

// Timers
ros::Timer stateMachineTimer;
ros::Timer publish_status_timer;
ros::Timer targetDetectedTimer;

time_t timerStartTime;                          // records time for delays in sequanced actions, 1 second resolution.

unsigned int startDelayInSeconds = 1;           // An initial delay to allow the rover to gather enough position data to
                                                // average its location.
float timerTimeElapsed = 0;

char host[128];
string publishedName;
char prev_state_machine[128];

//Transforms
tf::TransformListener *tfListener;

// OS Signal Handler
void sigintEventHandler(int signal);

//Callback handlers
void joyCmdHandler(const sensor_msgs::Joy::ConstPtr& message);
void modeHandler(const std_msgs::UInt8::ConstPtr& message);
void targetHandler(const apriltags_ros::AprilTagDetectionArray::ConstPtr& tagInfo);
void obstacleHandler(const std_msgs::UInt8::ConstPtr& message);
void odometryHandler(const nav_msgs::Odometry::ConstPtr& message);
void mapHandler(const nav_msgs::Odometry::ConstPtr& message);
void mobilityStateMachine(const ros::TimerEvent&);
void publishStatusTimerEventHandler(const ros::TimerEvent& event);
void targetDetectedReset(const ros::TimerEvent& event);

//CNM Code Follows:
//--------------------------------------------

//ARRAYS FOR CENTER

//Actual Center Array
float CenterXCoordinates[500];
float CenterYCoordinates[500];

//GPS Points from across the octagon
float mapCenterXCoordinates[500];
float mapCenterYCoordinates[500];

//ODOM Points from across the octagon
float mapOdomXCoordintes[500];
float mapOdomYCoordinates[500];

geometry_msgs::Pose2D cnmCenterLocation;                    //AVG Center Location spit out by AVGCenter
geometry_msgs::Pose2D avgCenterRotation;                    //AVG Center of the octagon rotation

double CENTEROFFSET = .95;                                  //offset for seeing center
double AVOIDOBSTDIST = .55;                                 //distance to drive for avoiding targets
double AVOIDTARGDIST = .45;                                 //distance to drive for avoiding targets

//Target Collection Variables
//--------------------------------------------

//ORIGINAL FILE VARIABLE:  MOVED HERE FOR EASY LOCATING

float searchVelocity = 0.2;                                 // meters/second  ORIGINALLY .2

//Variables for MobilityStateMachine:  MOVED HERE FOR SCOPE

float rotateOnlyAngleTolerance = 0.5;                       //jms chnaged from .4
int returnToSearchDelay = 5;

//NEST INFORMATION

bool centerSeen = false;                                    //If we CURRENTLY see the center
bool cnmHasCenterLocation = false;                          //If we have a center/nest location at all
bool cnmLocatedCenterFirst = false;                         //If this is the first time we have seen the nest

//FINDING NEST BEHAVIOR

bool cnmCenteringFirstTime = true;                          //First time entering the Centering function
bool cnmCentering;                                          //If we are trying to Center the rover
bool cnmInitDriveFor = false;                               //The initial drive forward

//INITIAL NEST SEARCH

bool cnmFirstBootProtocol = true;
bool cnmHasWaitedInitialAmount = false;
bool cnmInitialPositioningComplete = false;
bool cnmHasMovedForward = false;
bool cnmHasTurned180 = false;

//Variables for Obstacle Avoidance

bool cnmAvoidObstacle = false;
bool cnmSeenAnObstacle = false;

//Variables for PickUp

bool cnmFinishedPickUp = true;
bool cnmWaitToReset = false;

//Variables for reverse/180 behvaior
bool firstReverse = true;
bool cnmReverse = false;
bool cnmReverseDone = true;
bool cnmTurn180Done = true;
int cnmCheckTimer = 0;

//Variable for avoiding targets when carrying a target
bool cnmAvoidTargets = false;
bool cnmRotate = false;

//Obstacle Avoidance Timers
//---------------------------------------------
ros::Timer cnmAvoidObstacleTimer;
ros::Duration cnmAvoidObstacleTimerTime(10);

//First Boot Timers
//---------------------------------------------

//Waits 10 seconds and Drives Forward
ros::Timer cnmInitialPositioningTimer;
ros::Duration cnmInitialPositionTimerTime(10);      //time in seconds

//Waits 10 seconds and Turns 180
ros::Timer cnmForwardTimer;
ros::Duration cnmForwardTimerTime(10);

//Waits 10 Seconds and triggers the robot to continue search
ros::Timer cnmInitialWaitTimer;
ros::Duration cnmInitialWaitTime(10);

//Target Avoidance Timers
//---------------------------------------------
ros::Timer cnmAvoidOtherTargetTimer;
ros::Duration cnmAvoidOtherTargetTimerTime(4);

//UPDATE Search Timer (Updates Center Location)
//---------------------------------------------
ros::Timer cnmUpdateSearchTimer;
ros::Duration cnmUpdateSearchTimerTime(180);

//Reverse Timers
//---------------------------------------------
ros::Timer cnmReverseTimer;
ros::Duration cnmReverseTimerTime(2);

ros::Timer cnmWaitToResetWGTimer;
ros::Duration cnmWaitToResetWGTimerTime(2);

//180 Timers(TIED TO REVERSE)
//---------------------------------------------
ros::Timer cnmTurn180Timer;
ros::Duration cnmTurn180TimerTime(2.5);

//Centering Timer (used to center rover and find more accurate point to
    //translate centers position to
//---------------------------------------------
ros::Timer cnmFinishedCenteringTimer;
ros::Duration cnmFinishedCenteringTimerTime(4);


ros::Timer cnmAfterPickUpTimer;
ros::Duration cnmAfterPickUpTimerTime(2);


//CNM moved ORIGINAL CODE to utility functions
//---------------------------------------------
void InitComp();                                                //INIT COMPONENTS (Builds map, sets Pose 2D Objects defaults)

bool CNMTransformCode();                                        //A function for the Transform segment in Mobility State Machine
                                                                //	- returns true only if it needs to return
bool CNMPickupCode();                                           //A function for PickUpController in Mobility State Machine
                                                                //	- returns false if it needs to break
bool CNMRotateCode();                                           //A function for the Rotate Mobility State Machine Code

void CNMSkidSteerCode();                                        //A function with all the skid steer mobility code
//---------------------------------------------

void CNMFirstBoot();                                            //Code for robot to run on initial switch to autonomous mode

//NEW REVERSE ATTEMPT
void CNMStartReversing();                                       //Begins Reverse Timers
void CNMReverseReset();                                         //Resets Reverse Variables

void CNMFirstSeenCenter();                                      //Initial Center Find Code
void CNMRefindCenter();                                         //Refind Center Code

void CNMTargetPickup(PickUpResult result);                      //Wrist/Gripper Setting on pickup

void CNMTargetAvoid();                                          //Code To Avoid Targets

bool CNMCentered(double count, double countRight, double countLeft);    //Squares Rover up on nest when found

void CNMAVGCenter(geometry_msgs::Pose2D currentLocation);       //Avergages derived center locations

void CNMAVGMap();                                               //Averages GPS AND ODOM points around the octagon


//Timer Functions/Callbacks Handlers
//-----------------------------------

//INITIAL NEST SEARCH
void CNMInitPositioning(const ros::TimerEvent& event);          //Wait Before Driving Forward
void CNMForwardInitTimerDone(const ros::TimerEvent& event);     //Wait Before 180
void CNMInitialWait(const ros::TimerEvent& e);                  //Wait Before Continued Search

//TIMER FOR SQUARING UP ON NEST
void CNMCenterTimerDone(const ros::TimerEvent& event);          //Timer before telling rover it has finished squaring up on nest

//PICKUP TIMERS/DROP OFF TIMERS
void cnmFinishedPickUpTime(const ros::TimerEvent& e);           //Wait before detecting other tags to avoid
void cnmWaitToResetWG(const ros::TimerEvent& e);                //After Dropping off, wait before resetting grippers

//REVERSE TIMER
void CNMReverseTimer(const ros::TimerEvent& event);             //REVERSES
void CNMTurn180(const ros::TimerEvent& event);                  //Turns 180

//Obstacle Avoidance Timer
void CNMAvoidObstacle(const ros::TimerEvent& event);            //Timer Function(when timer fires, it runs this code)

//Target Avoidance Timer
void CNMAvoidOtherTargets(const ros::TimerEvent& event);        //NOT BEING USED

//Update Timer  --NOT USED--
void CNMUpdateSearch(const ros::TimerEvent& event);             //NOT BEING USED


//MAIN
//--------------------------------------------
int main(int argc, char **argv)
{

    gethostname(host, sizeof(host));
    string hostname(host);

    InitComp();

    // instantiate random number generator
    rng = new random_numbers::RandomNumberGenerator();

    if (argc >= 2)
    {
        publishedName = argv[1];
        cout << "Welcome to the world of tomorrow " << publishedName
            << "!  Mobility turnDirectionule started." << endl;
    }
    else
    {
        publishedName = hostname;
        cout << "No Name Selected. Default is: " << publishedName << endl;
    }

    // NoSignalHandler so we can catch SIGINT ourselves and shutdown the node
    ros::init(argc, argv, (publishedName + "_MOBILITY"), ros::init_options::NoSigintHandler);
    ros::NodeHandle mNH;

    // Register the SIGINT event handler so the node can shutdown properly
    signal(SIGINT, sigintEventHandler);

    joySubscriber = mNH.subscribe((publishedName + "/joystick"), 10, joyCmdHandler);
    modeSubscriber = mNH.subscribe((publishedName + "/mode"), 1, modeHandler);
    targetSubscriber = mNH.subscribe((publishedName + "/targets"), 10, targetHandler);
    obstacleSubscriber = mNH.subscribe((publishedName + "/obstacle"), 10, obstacleHandler);
    odometrySubscriber = mNH.subscribe((publishedName + "/odom/filtered"), 10, odometryHandler);
    mapSubscriber = mNH.subscribe((publishedName + "/odom/ekf"), 10, mapHandler);

    status_publisher = mNH.advertise<std_msgs::String>((publishedName + "/status"), 1, true);
    stateMachinePublish = mNH.advertise<std_msgs::String>((publishedName + "/state_machine"), 1, true);
    fingerAnglePublish = mNH.advertise<std_msgs::Float32>((publishedName + "/fingerAngle/cmd"), 1, true);
    wristAnglePublish = mNH.advertise<std_msgs::Float32>((publishedName + "/wristAngle/cmd"), 1, true);
    infoLogPublisher = mNH.advertise<std_msgs::String>("/infoLog", 1, true);
    driveControlPublish = mNH.advertise<geometry_msgs::Twist>((publishedName + "/driveControl"), 10);

    publish_status_timer = mNH.createTimer(ros::Duration(status_publish_interval), publishStatusTimerEventHandler);
    stateMachineTimer = mNH.createTimer(ros::Duration(mobilityLoopTimeStep), mobilityStateMachine);
    targetDetectedTimer = mNH.createTimer(ros::Duration(0), targetDetectedReset, true);


    //CNM TIMERS
    //----------------------------------------------------

    //-----INITIAL CENTER FIND TIMERS-----

    //Waits Before Driving Forward
    cnmInitialPositioningTimer = mNH.createTimer(cnmInitialPositionTimerTime, CNMInitPositioning, true);
    cnmInitialPositioningTimer.stop();

    //Waits before Turning 180 Degrees
    cnmForwardTimer = mNH.createTimer(cnmForwardTimerTime, CNMForwardInitTimerDone, true);
    cnmForwardTimer.stop();

    //Waits before Running Interrupted Search
    cnmInitialWaitTimer = mNH.createTimer(cnmInitialWaitTime, CNMInitialWait, true);
    cnmInitialWaitTimer.stop();

    //-----REVERSE BEHAVIOR TIMERS------

    //Timer for mandatory reversing
    cnmReverseTimer = mNH.createTimer(cnmReverseTimerTime, CNMReverseTimer, true);
    cnmReverseTimer.stop();    

    //Timer for turning 180 degrees
    cnmTurn180Timer = mNH.createTimer(cnmTurn180TimerTime, CNMTurn180, true);
    cnmTurn180Timer.stop();

    //-----DROPOFF TIMERS-----

    //Waits to reset Wrist/Gripper to a lowered driving state (Prevents trapping blocks under gripper)
    cnmWaitToResetWGTimer = mNH.createTimer(cnmWaitToResetWGTimerTime, cnmWaitToResetWG, true);
    cnmWaitToResetWGTimer.stop();

    //-----PICKUP TIMERS-----

    //Waits a time after pickup before viewing other targets as obstacles
    cnmAfterPickUpTimer = mNH.createTimer(cnmAfterPickUpTimerTime, cnmFinishedPickUpTime, true);
    cnmAfterPickUpTimer.stop();

    //AVOIDING TARGETS IF CARRYING ONE
    cnmAvoidOtherTargetTimer = mNH.createTimer(cnmAvoidOtherTargetTimerTime, CNMAvoidOtherTargets, true);
    cnmAvoidOtherTargetTimer.stop();

    //-----OBSTACLE AVOIDANCE-----

    //Timer for Obstacle Avoidance
    cnmAvoidObstacleTimer = mNH.createTimer(cnmAvoidObstacleTimerTime, CNMAvoidObstacle, true);
    cnmAvoidObstacleTimer.stop();

    //-----CENTERFIND TIMERS-----

    //CENTERING TIMER
    cnmFinishedCenteringTimer = mNH.createTimer(cnmFinishedCenteringTimerTime, CNMCenterTimerDone, true);
    cnmFinishedCenteringTimer.stop();

    //-----UPDATES-----

    //UPDATES MAP LOCATION  --NOT BEING USED, MAY DELETE--
    cnmUpdateSearchTimer = mNH.createTimer(cnmUpdateSearchTimerTime, CNMUpdateSearch);

    tfListener = new tf::TransformListener();
    std_msgs::String msg;
    msg.data = "Log Started";
    infoLogPublisher.publish(msg);

    stringstream ss;
    ss << "Rover start delay set to " << startDelayInSeconds << " seconds";
    msg.data = ss.str();
    infoLogPublisher.publish(msg);

    timerStartTime = time(0);

    ros::spin();

    return EXIT_SUCCESS;
}

// This is the top-most logic control block organised as a state machine.
// This function calls the dropOff, pickUp, and search controllers.
// This block passes the goal location to the proportional-integral-derivative
// controllers in the abridge package.
void mobilityStateMachine(const ros::TimerEvent&)
{

    std_msgs::String stateMachineMsg;

    // calls the averaging function, also responsible for
    // transform from Map frame to odom frame.
    mapAverage();

    // Robot is in automode
    if (currentMode == 2 || currentMode == 3)
    {

        //cnmFirstBootProtocol runs the first time the robot is set to autonomous mode (2 || 3)
        if(cnmFirstBootProtocol) { CNMFirstBoot(); }

        if(!cnmReverseDone && cnmReverse) { sendDriveCommand(-0.2, 0.0); return; }


        // time since timerStartTime was set to current time
        timerTimeElapsed = time(0) - timerStartTime;

        // init code goes here. (code that runs only once at start of
        // auto mode but wont work in main goes here)
        if (!init)
        {
            if (timerTimeElapsed > startDelayInSeconds)
            {
                // Set the location of the center circle location in the map
                // frame based upon our current average location on the map.
                centerLocationMap.x = currentLocationAverage.x;
                centerLocationMap.y = currentLocationAverage.y;
                centerLocationMap.theta = currentLocationAverage.theta;

                // initialization has run
                init = true;
            }
            else { return; }
        }

        // If no target collected or no detected blocks,
        // set fingers to open wide and to raised position.
        if (!targetCollected && !targetDetected)
        {

            //SET NORMAL DRIVING GRIPPER/WRIST ANGLE
            //---------------------------------------------
            if(!cnmWaitToReset)
            {
                // set gripper
                std_msgs::Float32 angle;

                // close fingers all the way
                angle.data = 0;

                fingerAnglePublish.publish(angle);

                //raise wrist partially (avoid obstacle calls)
                angle.data = .6;

                // raise wrist
                wristAnglePublish.publish(angle);
            }
        }

        // Select rotation or translation based on required adjustment
        switch (stateMachineState)
        {
            // If no adjustment needed, select new goal
        case STATE_MACHINE_TRANSFORM:
        {
            stateMachineMsg.data = "TRANSFORMING";

            if (!CNMTransformCode()) { break; }
            //Purposefully fall through to next case without breaking
        }

        // Calculate angle between currentLocation.theta and goalLocation.theta
        // Rotate left or right depending on sign of angle
        // Stay in this state until angle is minimized
        case STATE_MACHINE_ROTATE:
        {
            stateMachineMsg.data = "ROTATING";

            if(CNMRotateCode()) { break; }

            //Purposefully fall through to next case without breaking
        }

        // Calculate angle between currentLocation.x/y and goalLocation.x/y
        // Drive forward
        // Stay in this state until angle is at least PI/2
        case STATE_MACHINE_SKID_STEER:
        {
            stateMachineMsg.data = "SKID_STEER";

            CNMSkidSteerCode();

            break;
        }

        case STATE_MACHINE_PICKUP:
        {
            stateMachineMsg.data = "PICKUP";

            if(CNMPickupCode()) { return; }

            break;
        }

        case STATE_MACHINE_DROPOFF: {
            stateMachineMsg.data = "DROPOFF";
            break;
        }

        default:
        {
            break;
        }

        } /* end of switch() */
    }

    // mode is NOT auto
    else
    {
        // publish current state for the operator to see
        stateMachineMsg.data = "WAITING";
    }

    // publish state machine string for user, only if it has changed, though
    if (strcmp(stateMachineMsg.data.c_str(), prev_state_machine) != 0)
    {
        stateMachinePublish.publish(stateMachineMsg);
        sprintf(prev_state_machine, "%s", stateMachineMsg.data.c_str());
    }
}

void sendDriveCommand(double linearVel, double angularError)
{
    velocity.linear.x = linearVel,
        velocity.angular.z = angularError;

    // publish the drive commands
    driveControlPublish.publish(velocity);
}

/*************************
* ROS CALLBACK HANDLERS *
*************************/

void targetHandler(const apriltags_ros::AprilTagDetectionArray::ConstPtr& message)
{

    // If in manual mode do not try to automatically pick up the target
    if (currentMode == 1 || currentMode == 0) { return; }


    // found a target april tag and looking for april cubes;
    // with safety timer at greater than 5 seconds.
    //---------------------------------------------
    PickUpResult result;
    //---------------------------------------------

    double numTargets = 0;
    double numTargLeft = 0;
    double numTargRight = 0;

    static bool turnLeft = false;
    static bool turnRight = false;

    // if a target is detected and we are looking for center tags
    if (message->detections.size() > 0 && !reachedCollectionPoint)
    {
        //IMPORTANT VARIABLES
        //---------------------------------------------
        float cameraOffsetCorrection = 0.020; //meters;

        centerSeen = false;             //set to false
        double count = 0;
        double countRight = 0;
        double countLeft = 0;

        //IF WE SEE A CENTER TAG LOOP: this gets # number of center tags
        //---------------------------------------------
        for (int i = 0; i < message->detections.size(); i++)
        {
            if (message->detections[i].id == 256)
            {
                geometry_msgs::PoseStamped cenPose = message->detections[i].pose;

                // checks if tag is on the right or left side of the image
                if (cenPose.pose.position.x + cameraOffsetCorrection > 0) { countRight++; }
                else { countLeft++; }

                centerSeen = true;
                cnmHasCenterLocation = true;
                count++;
            }
            else if(message->detections[i].id == 0)
            {
                geometry_msgs::PoseStamped cenPose = message->detections[i].pose;

                numTargets++;
                if (cenPose.pose.position.x + cameraOffsetCorrection > 0) { numTargRight++; }
                else { numTargLeft++; }
            }
        }

        dropOffController.setDataTargets(count,countLeft,countRight);

        //CNM MODIFIED: If we see the center and don't have a target collected
        //---------------------------------------------
        if(centerSeen && !targetCollected)
        {

            if(cnmCenteringFirstTime)
            {
                cnmCentering = true;
                cnmCenteringFirstTime = false;

                std_msgs::String msg;
                msg.data = "Seen A Center Tag";
                infoLogPublisher.publish(msg);

                goalLocation = currentLocation;
                stateMachineState = STATE_MACHINE_TRANSFORM;

                if(cnmFirstBootProtocol)
                {
                    cnmFirstBootProtocol = false;
                    cnmInitialPositioningComplete = true;

                    cnmInitialPositioningTimer.stop();
                    cnmForwardTimer.stop();
                }
            }

            if(CNMCentered(count, countRight, countLeft)  && !targetCollected)
            {

                //If we haven't seen the center before
                //---------------------------------------------
                if(!cnmLocatedCenterFirst) { CNMFirstSeenCenter(); }

                //If we have seen the center before
                //---------------------------------------------
                else if(cnmLocatedCenterFirst && cnmInitialPositioningComplete) { CNMRefindCenter(); }

                if(!cnmReverse && cnmInitialPositioningComplete) { CNMStartReversing(); }

                cnmFinishedCenteringTimer.start();

                searchController.doAnotherOctagon();
            }

            //FINAL STEPS
            //---------------------------------------------
            targetDetected = false;
            pickUpController.reset();
            return;
        }

        //If we see the center, have a target, and are not in an avoiding targets state
        //---------------------------------------------
        if (centerSeen && targetCollected && !cnmAvoidTargets && !cnmReverse)
        {
            stateMachineState = STATE_MACHINE_TRANSFORM;
            goalLocation = cnmCenterLocation;
        }

        // end found target and looking for center tags
    }

    //if we see an april tag, are not carrying a target, and if timer is ok
    //---------------------------------------------
    if (message->detections.size() > 0 && !targetCollected && timerTimeElapsed > 5)
    {
        //Check to see if have found the nests location at all and if we currently see an obstacle
        //---------------------------------------------
        if(cnmHasCenterLocation && !cnmSeenAnObstacle)
        {

            //If we see the center, ignore the target and back up!
            //---------------------------------------------
            if(centerSeen)
            {
                targetDetected = false;

                stateMachineState = STATE_MACHINE_ROTATE;

                pickUpController.reset();

                CNMStartReversing();
            }

            //Pick Up The Target
            //---------------------------------------------
            else
            {
                //Check to see if we are currently trying to reverse
                //---------------------------------------------
                if(cnmReverse)
                {
                    //Have we at least turned 180 degrees?
                    //---------------------------------------------
                    if(cnmTurn180Done)
                    {
                        std_msgs::String msg;
                        msg.data = "180 Conditions met";
                        infoLogPublisher.publish(msg);

                        //If so, stop trying to reverse
                        //---------------------------------------------
                        CNMReverseReset();

                        sendDriveCommand(0.0, 0.0);
                    }
                }
                else
                {
                    targetDetected = true;

                    //pickup state so target handler can take over driving.
                    //---------------------------------------------
                    stateMachineState = STATE_MACHINE_PICKUP;
                    result = pickUpController.selectTarget(message);

                    CNMTargetPickup(result);
                }
            }

        }

        //If not gotten the centers point, avoid the target
        //---------------------------------------------
        else
        {

            targetDetected = false;

        }

/*
        if(cnmReverse  && !centerSeen)
        {
            cnmReverse = false;
            targetDetected = false;

            if(message->detections.size() > 0)
            {
                cnmReverse = true;
            }
        }
        else
        {

        }
*/
    }

    //CNM ADDED: if we see a target and have already picked one up
    //---------------------------------------------
    else if(message->detections.size() > 1 && targetCollected && timerTimeElapsed > 5)
    {
        //Avoid Targets
        //---------------------------------------------
        //- 2 Conditions for avoiding targets:
            //1.) We haven't found the center yet
            //2.) We are currently carrying a block
                //a.)  Do we see the center?  If so we need to continue drop off but stop
                //b.)  If not, just avoid!

        //Do we currently have a target?
        if(targetCollected  && numTargets > 1)
        {
            //Do we see the center?
            if(centerSeen)
            {
                //Are we in our final approach to drop off a target?
                if(dropOffController.getCenterApproach())
                {

                    //DROP OFF TAG?
                    //REVERSE AND TRY AGAIN?

                }
            }
            else if(cnmFinishedPickUp)
            {
                //Is target on left or right?
                if(numTargLeft > numTargRight)
                {
                    turnLeft = true;
                    goalLocation.theta = currentLocation.theta - (M_PI/2);
                }
                else
                {
                    turnRight = true;
                    goalLocation.theta = currentLocation.theta + (M_PI/2);
                }

                //select new position
                goalLocation.x = currentLocation.x + (AVOIDTARGDIST * cos(goalLocation.theta));
                goalLocation.y = currentLocation.y + (AVOIDTARGDIST * sin(goalLocation.theta));
            }
        }

        //If we DON'T have a target
        else
        {
            //
        }
        //CNMTargetAvoid();
    }

}

void modeHandler(const std_msgs::UInt8::ConstPtr& message)
{
    currentMode = message->data;
    sendDriveCommand(0.0, 0.0);
}

void obstacleHandler(const std_msgs::UInt8::ConstPtr& message)
{
    static bool firstTimeRotate = true;
    static bool firstTimeSeeObst = true;

    if (currentMode == 1 || currentMode == 0) { return; }

    //no matter what we receive from obstacle
    else if ((!targetDetected || targetCollected) && (message->data > 0))
    {
        cnmSeenAnObstacle = true;

        cnmAvoidObstacleTimer.start();

        if(!cnmAvoidObstacle)
        {
            if(firstTimeSeeObst)
            {
                std_msgs::String msg;
                msg.data = "SEE OBSTACLE; STOPPING";
                infoLogPublisher.publish(msg);
                firstTimeSeeObst = false;
            }

            sendDriveCommand(0.0, 0.0);
        }
        else
        {
            if(firstTimeRotate)
            {
                std_msgs::String msg;
                msg.data = "ROTATING";
                infoLogPublisher.publish(msg);

                searchController.obstacleWasAvoided();

                firstTimeRotate = false;
            }

            //if searching right, turn right
            if(searchController.cnmIsAlternating()) { sendDriveCommand(0.0, -0.2); }

            //if searching left, turn left
            else { sendDriveCommand(0.0, 0.2); }

        }
    }

    //if we saw an obstacle but no longer see one
    else if (cnmSeenAnObstacle && (!targetDetected || targetCollected) && (message->data == 0))
    {
        cnmSeenAnObstacle = false;
        cnmAvoidObstacleTimer.stop();

        if(cnmAvoidObstacle)
        {
            if(!firstTimeRotate)
            {
                std_msgs::String msg;
                msg.data = "DRIVING FORWARD";
                infoLogPublisher.publish(msg);

                firstTimeRotate = true;
            }

            if(searchController.cnmIsAlternating())
            {
                if(targetCollected) { goalLocation.theta = currentLocation.theta + (M_PI/6); }

                else { goalLocation.theta = currentLocation.theta - (M_PI/6); }
            }

            else
            {
                if(targetCollected) { goalLocation.theta = currentLocation.theta - (M_PI/6); }

                else { goalLocation.theta = currentLocation.theta + (M_PI/6); }
            }

            goalLocation.x = currentLocation.x + (AVOIDOBSTDIST * (cos(goalLocation.theta)));
            goalLocation.y = currentLocation.y + (AVOIDOBSTDIST * (sin(goalLocation.theta)));

            stateMachineState = STATE_MACHINE_ROTATE;

            cnmAvoidObstacle = false;
        }

        firstTimeSeeObst = true;
    }

    // the front ultrasond is blocked very closely. 0.14m currently
    if (message->data == 4)
    {
        blockBlock = true;
    }
    else
    {
        blockBlock = false;
    }
}

void odometryHandler(const nav_msgs::Odometry::ConstPtr& message)
{
    //Get (x,y) location directly from pose
    currentLocation.x = message->pose.pose.position.x;
    currentLocation.y = message->pose.pose.position.y;

    //Get theta rotation by converting quaternion orientation to pitch/roll/yaw
    tf::Quaternion q(message->pose.pose.orientation.x, message->pose.pose.orientation.y, message->pose.pose.orientation.z, message->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    currentLocation.theta = yaw;
}

void mapHandler(const nav_msgs::Odometry::ConstPtr& message)
{
    //Get (x,y) location directly from pose
    currentLocationMap.x = message->pose.pose.position.x;
    currentLocationMap.y = message->pose.pose.position.y;

    //Get theta rotation by converting quaternion orientation to pitch/roll/yaw
    tf::Quaternion q(message->pose.pose.orientation.x, message->pose.pose.orientation.y, message->pose.pose.orientation.z, message->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    currentLocationMap.theta = yaw;
}

void joyCmdHandler(const sensor_msgs::Joy::ConstPtr& message)
{
    if (currentMode == 0 || currentMode == 1)
    {
        sendDriveCommand(abs(message->axes[4]) >= 0.1 ? message->axes[4] : 0, abs(message->axes[3]) >= 0.1 ? message->axes[3] : 0);
    }
}

void publishStatusTimerEventHandler(const ros::TimerEvent&)
{
    std_msgs::String msg;
    msg.data = "CNMSRWG17 Online";
    status_publisher.publish(msg);
}

void targetDetectedReset(const ros::TimerEvent& event)
{
    targetDetected = false;

    std_msgs::Float32 angle;
    angle.data = 0;

    // close fingers
    fingerAnglePublish.publish(angle);

    // raise wrist
    wristAnglePublish.publish(angle);
}

void sigintEventHandler(int sig)
{
    // All the default sigint handler does is call shutdown()
    ros::shutdown();
}


//DEFAULT MAP AVERAGE  --NOT USING--
//-----------------------------------

void mapAverage()
{
    // store currentLocation in the averaging array
    mapLocation[mapCount] = currentLocationMap;
    mapCount++;

    if (mapCount >= mapHistorySize)
    {
        mapCount = 0;
    }

    double x = 0;
    double y = 0;
    double theta = 0;

    // add up all the positions in the array
    for (int i = 0; i < mapHistorySize; i++)
    {
        x += mapLocation[i].x;
        y += mapLocation[i].y;
        theta += mapLocation[i].theta;
    }

    // find the average
    x = x / mapHistorySize;
    y = y / mapHistorySize;

    // Get theta rotation by converting quaternion orientation to pitch/roll/yaw
    theta = theta / 100;
    currentLocationAverage.x = x;
    currentLocationAverage.y = y;
    currentLocationAverage.theta = theta;

    // only run below code if a centerLocation has been set by initilization
    if (init)
    {
        // map frame
        geometry_msgs::PoseStamped mapPose;

        // setup msg to represent the center location in map frame
        mapPose.header.stamp = ros::Time::now();

        mapPose.header.frame_id = publishedName + "/map";
        mapPose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, centerLocationMap.theta);
        mapPose.pose.position.x = centerLocationMap.x;
        mapPose.pose.position.y = centerLocationMap.y;
        geometry_msgs::PoseStamped odomPose;
        string x = "";

        try
        {
            //attempt to get the transform of the center point in map frame to odom frame.
            tfListener->waitForTransform(publishedName + "/map", publishedName + "/odom", ros::Time::now(), ros::Duration(1.0));
            tfListener->transformPose(publishedName + "/odom", mapPose, odomPose);
        }

        catch (tf::TransformException& ex)
        {
            ROS_INFO("Received an exception trying to transform a point from \"map\" to \"odom\": %s", ex.what());
            x = "Exception thrown " + (string)ex.what();
            std_msgs::String msg;
            stringstream ss;
            ss << "Exception in mapAverage() " + (string)ex.what();
            msg.data = ss.str();
            infoLogPublisher.publish(msg);
        }

        // Use the position and orientation provided by the ros transform.
        centerLocation.x = odomPose.pose.position.x; //set centerLocation in odom frame
        centerLocation.y = odomPose.pose.position.y;
    }
}

//DEFAULT INITIALIZE COMPONENTS
//-----------------------------------
void InitComp()
{
    //dropOffController.setSearch(&searchController);  //CNMAdded

    //This is original code from main, moved to a utility function.

    //create map
    for (int i = 0; i < 100; i++)
    {
        mapLocation[i].x = 0;
        mapLocation[i].y = 0;
        mapLocation[i].theta = 0;
    }

    centerLocation.x = 0;
    centerLocation.y = 0;
    centerLocationOdom.x = 0;
    centerLocationOdom.y = 0;

}

//MOBILITY TRANFORM STATES
//-----------------------------------

bool CNMTransformCode()
{

    // If returning with a target
    if (targetCollected && !avoidingObstacle && !cnmCentering)
    {
        // calculate the euclidean distance between
        // centerLocation and currentLocation
        dropOffController.setCenterDist(hypot(cnmCenterLocation.x - currentLocation.x, cnmCenterLocation.y - currentLocation.y));
        dropOffController.setDataLocations(cnmCenterLocation, currentLocation, timerTimeElapsed);

        DropOffResult result = dropOffController.getState();

        if (result.timer)
        {
            timerStartTime = time(0);
            reachedCollectionPoint = true;
        }

        std_msgs::Float32 angle;

        if (result.fingerAngle != -1)
        {
            angle.data = result.fingerAngle;
            fingerAnglePublish.publish(angle);
        }

        if (result.wristAngle != -1)
        {
            angle.data = result.wristAngle;
            wristAnglePublish.publish(angle);
        }

        if (result.reset)
        {
            timerStartTime = time(0);
            targetCollected = false;
            targetDetected = false;
            lockTarget = false;
            sendDriveCommand(0.0, 0.0);  //JS changed from 0.0

            // move back to transform step
            stateMachineState = STATE_MACHINE_TRANSFORM;
            reachedCollectionPoint = false;
            centerLocationOdom = currentLocation;

            dropOffController.reset();

            cnmWaitToReset = true;
            cnmWaitToResetWGTimer.start();
        }
        else if (result.goalDriving && timerTimeElapsed >= 5)
        {
            goalLocation = result.centerGoal;
            stateMachineState = STATE_MACHINE_ROTATE;
            timerStartTime = time(0);
        }

        // we are in precision/timed driving
        else
        {
            goalLocation = currentLocation;
            sendDriveCommand(result.cmdVel, result.angleError);
            stateMachineState = STATE_MACHINE_TRANSFORM;

            return false;
        }
    }

    //If angle between current and goal is significant
    //if error in heading is greater than 0.4 radians
    else if (fabs(angles::shortest_angular_distance(currentLocation.theta, goalLocation.theta)) >
        rotateOnlyAngleTolerance)
    {
        stateMachineState = STATE_MACHINE_ROTATE;
    }

    //If goal has not yet been reached drive and maintain heading
    else if (fabs(angles::shortest_angular_distance(currentLocation.theta,
        atan2(goalLocation.y - currentLocation.y, goalLocation.x - currentLocation.x))) < M_PI_2)
    {
        stateMachineState = STATE_MACHINE_SKID_STEER;
    }

    //Otherwise, drop off target and select new random uniform heading
    //If no targets have been detected, assign a new goal
    else if (!targetDetected && timerTimeElapsed > returnToSearchDelay)
    {
        if(cnmInitialPositioningComplete && !cnmReverse  && !cnmCentering)
        {
            int position;
            double distance;

            goalLocation = searchController.search(currentLocation);

            position = searchController.cnmGetSearchPosition();

            distance = searchController.cnmGetSearchDistance();

            stringstream ss;
            ss << "Traveling to point " << position << " in pattern:  " << distance;
            msg.data = ss.str();
            infoLogPublisher.publish(msg);

            if(cnmHasCenterLocation) { CNMAVGMap(); }

            if(searchController.cnmGetSearchPosition() == 9 && searchController.getHasDoneRotation()  && cnmHasCenterLocation)
            {
                CNMAVGCenter(avgCenterRotation);
            }
        }
    }

    return true;
}

bool CNMRotateCode()
{

    // Calculate the diffrence between current and desired
    // heading in radians.
    float errorYaw = angles::shortest_angular_distance(currentLocation.theta, goalLocation.theta);

    // If angle > 0.4 radians rotate but dont drive forward.
    if (fabs(angles::shortest_angular_distance(currentLocation.theta, goalLocation.theta)) > rotateOnlyAngleTolerance)
    {
        // rotate but dont drive  0.05 is to prevent turning in reverse
        sendDriveCommand(0.05, errorYaw);
        return true;
    }
    else
    {
        // move to differential drive step
        stateMachineState = STATE_MACHINE_SKID_STEER;
        //fall through on purpose.
    }

    return false;
}

void CNMSkidSteerCode()
{
    // calculate the distance between current and desired heading in radians
    float errorYaw = angles::shortest_angular_distance(currentLocation.theta, goalLocation.theta);

    // goal not yet reached drive while maintaining proper heading.
    if (fabs(angles::shortest_angular_distance(currentLocation.theta, atan2(goalLocation.y - currentLocation.y, goalLocation.x - currentLocation.x))) < M_PI_2)
    {
        // drive and turn simultaniously
        sendDriveCommand(searchVelocity, errorYaw / 2);
    }
    // goal is reached but desired heading is still wrong turn only
    else if (fabs(angles::shortest_angular_distance(currentLocation.theta, goalLocation.theta)) > 0.1)
    {
        // rotate but dont drive
        sendDriveCommand(0.0, errorYaw);
    }
    else
    {
        // stop
        sendDriveCommand(0.0, 0.0);
        avoidingObstacle = false;

        // move back to transform step
        stateMachineState = STATE_MACHINE_TRANSFORM;
    }
}

bool CNMPickupCode()
{

    PickUpResult result;

    // we see a block and have not picked one up yet
    //CNM ADDED:    AND if we are not doing our reverse behavior
    if (targetDetected && !targetCollected && !cnmReverse)
    {
        result = pickUpController.pickUpSelectedTarget(blockBlock);
        sendDriveCommand(result.cmdVel, result.angleError);
        std_msgs::Float32 angle;

        if (result.fingerAngle != -1)
        {
            angle.data = result.fingerAngle;
            fingerAnglePublish.publish(angle);
        }

        if (result.wristAngle != -1)
        {
            angle.data = result.wristAngle;

            // raise wrist
            wristAnglePublish.publish(angle);
        }

        if (result.giveUp)
        {
            targetDetected = false;
            stateMachineState = STATE_MACHINE_TRANSFORM;
            sendDriveCommand(0, 0);
            pickUpController.reset();
        }

        if (result.pickedUp)
        {
            pickUpController.reset();

            // assume target has been picked up by gripper
            targetCollected = true;
            result.pickedUp = false;
            stateMachineState = STATE_MACHINE_ROTATE;

            goalLocation.theta = atan2(cnmCenterLocation.y - currentLocation.y, cnmCenterLocation.x - currentLocation.x);
            //goalLocation.theta = atan2(centerLocationOdom.y - currentLocation.y, centerLocationOdom.x - currentLocation.x);

            // set center as goal position
            goalLocation.x = cnmCenterLocation.x;
            goalLocation.y = cnmCenterLocation.y;

            //goalLocation.x = centerLocationOdom.x = 0;
            //goalLocation.y = centerLocationOdom.y;

            // lower wrist to avoid ultrasound sensors
            std_msgs::Float32 angle;
            angle.data = 0.6;  //.8
            wristAnglePublish.publish(angle);
            sendDriveCommand(0.0, 0);

            cnmAfterPickUpTimer.start();
            cnmFinishedPickUp = false;

            return true;
        }
    }
    else
    {
        stateMachineState = STATE_MACHINE_TRANSFORM;
    }

    return false;
}

//Wrist/Gripper behavior on pickup moved to this utility function

void CNMTargetPickup(PickUpResult result)
{
    std_msgs::Float32 angle;

    //OPEN FINGERS
    //---------------------------------------------
    if (result.fingerAngle != -1)
    {
        angle.data = result.fingerAngle;
        fingerAnglePublish.publish(angle);
    }

    //LOWER WRIST
    //---------------------------------------------
    if (result.wristAngle != -1)
    {
        angle.data = result.wristAngle;
        wristAnglePublish.publish(angle);
    }
}

//CNM Functions Follow
//-----------------------------------

//Initial Nest Search

void CNMFirstBoot()
{
    static bool firstTimeInBoot = true;

    //FIRST TIME IN THIS FUNCTION
    if(firstTimeInBoot)
    {
        //Print a message to the info box letting us know the robot recognizes the state change
        std_msgs::String msg;
        msg.data = "Switched to AUTONOMOUS; Waiting For Find Center Protocol";
        infoLogPublisher.publish(msg);

        //goalLocation = currentLocation;

        firstTimeInBoot = false;

        //START TIMER to move forward
        cnmInitialPositioningTimer.start();
    }

    //IF WE SEE CENTER, BREAK EVERYTHING
    if(centerSeen)
    {
        cnmFirstBootProtocol = false;
        cnmInitialPositioningComplete = true;

        cnmInitialPositioningTimer.stop();
        cnmForwardTimer.stop();

        goalLocation = currentLocation;
    }

    //OTHERWISE-------

    //If we have waited the initial timer out, and are driving forward
    else if(cnmHasWaitedInitialAmount)
    {
        //If we HAVE moved Forward and are turning 180
        if(cnmHasMovedForward)
        {
            static bool firstIn180 = true;

            if(firstIn180)
            {
                std_msgs::String msg;
                msg.data = "Finishing 180, waiting before starting search";
                infoLogPublisher.publish(msg);

                firstIn180 = false;
            }

            //IF we HAVEN'T finished turning 180
            if(!cnmHasTurned180)
            {
                static bool firstInWait = true;

                if(firstInWait)
                {
                    std_msgs::String msg;
                    msg.data = "Finishing 180, waiting before starting search";
                    infoLogPublisher.publish(msg);
                    firstInWait = false;
                }

                //call last wait timer
                cnmInitialWaitTimer.start();
            }

        }

        //IF we haven't finished moving forward yet
        else
        {
            static bool firstIn = true;

            if(firstIn)
            {
                std_msgs::String msg;
                msg.data = "Starting Timer To Turn 180";
                infoLogPublisher.publish(msg);
                firstIn = false;
            }

            //Call forward timer, once timer fires, assumes we are done driving forward
            cnmForwardTimer.start();
        }
    }

    else
    {
        sendDriveCommand(0.0, 0.0);
    }
}

//Reverse

void CNMReverseReset()
{
    //RESET VARIABLES
    //---------------------------------------------
    //std_msgs::String msg;
    //msg.data = "RESET VARIABLES";
    //infoLogPublisher.publish(msg);

    cnmReverse = false;
    cnmReverseDone = false;
    firstReverse = true;
    cnmReverseDone = false;

    cnmCheckTimer = 0;

    cnmReverseTimer.stop();
    cnmTurn180Timer.stop();
}

void CNMStartReversing()
{
    //Pass Info to Info Log
    //---------------------------------------------
    std_msgs::String msg;
    msg.data = "STARTING REVERSE TIMER";
    infoLogPublisher.publish(msg);

    //Start First Timer
    //---------------------------------------------
    cnmReverseTimer.start();

    //Set Variables Appropriately
    //---------------------------------------------
    cnmReverse = true;
    firstReverse = false;
    cnmReverseDone = false;
    cnmTurn180Done = false;
    cnmCheckTimer = 0;

    //BACKUP!!!
    //---------------------------------------------
    sendDriveCommand(-0.2, 0);
}

//Target Avoidance

void CNMTargetAvoid()
{
    //if we don't see the center
    //---------------------------------------------
    if(!centerSeen)
    {
        //if we haven't triggered this behavior
            //ADDED:  getCenterApproach method to drop off class to get state of dropping off targets
            //  This was necessary so we don't try to avoid targets when driving in the center
        //---------------------------------------------
        if(!cnmAvoidTargets && !dropOffController.getCenterApproach())
        {
            //Trigger Bool
            cnmAvoidTargets = true;

            //Print Message to Info Box
            std_msgs::String msg;
            msg.data = "Avoiding Blocks; Carrying Target.  Starting Timer";
            infoLogPublisher.publish(msg);

            if(searchController.cnmIsAlternating()) { goalLocation.theta = currentLocation.theta - (M_PI/6); }
            else { goalLocation.theta = currentLocation.theta + (M_PI/6); }

            //select new position 25 cm from current location
            goalLocation.x = currentLocation.x + (AVOIDTARGDIST * cos(goalLocation.theta));
            goalLocation.y = currentLocation.y + (AVOIDTARGDIST * sin(goalLocation.theta));

            stateMachineState = STATE_MACHINE_ROTATE;

            //START NEW TIMER
            cnmAvoidOtherTargetTimer.start();
        }

    }
}

//Rotation for squaring up on center

bool CNMCentered(double count, double countRight, double countLeft)
{
    static float linearSpeed, angularSpeed;

    //VARIABLES
    //-----------------------------------
    const int amountOfTagsToSee = 5;
    bool seenEnoughTags = false;

    bool right, left;

    if(!cnmReverse)
    {
        if (countRight > 0) { right = true; }
        else { right = false; }

        if (countLeft > 0) { left = true; }
        else { left = false; }

        if(count > amountOfTagsToSee) { seenEnoughTags = true;}
        else { seenEnoughTags = false; }

        float turnDirection = 1;

        if (seenEnoughTags) //if we have seen enough tags
        {
            if ((countLeft-5) > countRight) //and there are too many on the left
        {
            right = false; //then we say none on the right to cause us to turn right
        }
            else if ((countRight-5) > countLeft)
        {
            left = false; //or left in this case
        }

            //otherwise turn till tags on both sides of image then drive straight
            if (left && right) { return true; }
        }

        if (right)
    {
        linearSpeed = -0.2 * turnDirection;
        angularSpeed = -0.35 * turnDirection;
    }
        else if (left)
    {
        linearSpeed = -0.2 * turnDirection;
        angularSpeed = 0.35 * turnDirection;
    }
        else
    {
        linearSpeed = -0.25;
        angularSpeed = 0.0;
    }

        sendDriveCommand(linearSpeed, angularSpeed);
    }

    return false;

}

//Seeing Center Behavior

void CNMFirstSeenCenter()
{
    //Create a new location object
    geometry_msgs::Pose2D location;
    location = currentLocation;

    //Print out to the screen we found the nest for the first time
    std_msgs::String msg;
    msg.data = "Found Initial Nest Location";
    infoLogPublisher.publish(msg);

    //change bool
    cnmLocatedCenterFirst = true;
    searchController.setCenterSeen(true);

    if(cnmInitialPositioningComplete)
    {
        msg.data = "Search Pattern Expanding";
        infoLogPublisher.publish(msg);
    }

    //NORMALIZE ANGLE
    double normCurrentAngle = angles::normalize_angle_positive(currentLocation.theta);

    location.x = currentLocation.x + (CENTEROFFSET * (cos(normCurrentAngle)));
    location.y = currentLocation.y + (CENTEROFFSET * (sin(normCurrentAngle)));

    CNMAVGCenter(location);

    //cnmReverse = true;
    CNMStartReversing();
}

void CNMRefindCenter()
{
    //Create a new location object
    geometry_msgs::Pose2D location;
    location = currentLocation;

    //pass search Controller the center point
        //this is in this statement so it doesn't repeatedly print
    std_msgs::String msg;
    msg.data = "Refound center, updating location";
    infoLogPublisher.publish(msg);

    //NORMALIZE ANGLE
    double normCurrentAngle = angles::normalize_angle_positive(currentLocation.theta);

    location.x = currentLocation.x + (CENTEROFFSET * (cos(normCurrentAngle)));
    location.y = currentLocation.y + (CENTEROFFSET * (sin(normCurrentAngle))); //(M_PI/4)));

    CNMAVGCenter(location);

    //cnmReverse = true;
    CNMStartReversing();
}

//CNM MAP BUILDING

void CNMAVGCenter(geometry_msgs::Pose2D newCenter)
{   

    //NOTES ON THIS FUNCTION:
    //- Takes a derived center point, puts it in an array of other
    //  center points and averages them together... allowing us to
    //  build a more dynamic center location (able to adjust with drift)

    std_msgs::String msg;
    msg.data = "Averaging Center Location";
    infoLogPublisher.publish(msg);

    const int ASIZE = 500;
    static int index = 0;

    static bool reached500 = false;

    CenterXCoordinates[index] = newCenter.x;
    CenterYCoordinates[index] = newCenter.y;

    if(index >= ASIZE)
    {
        if(!reached500) { reached500 = true; }
        index = 0;
    }
    else
    {
        index++;
    }

    float avgX = 0;
    float avgY = 0;

    if(!reached500)
    {
        for(int i = 0; i < index; i++)
        {
            avgX += CenterXCoordinates[i];
            avgY += CenterYCoordinates[i];
        }

        avgX = (avgX / index);
        avgY = (avgY / index);
    }
    else
    {
        for(int i = 0; i < ASIZE; i++)
        {
            avgX += CenterXCoordinates[i];
            avgY += CenterYCoordinates[i];
        }

        avgX = (avgX / ASIZE);
        avgY = (avgY / ASIZE);
    }

    //avgX += currentLocationMap.x;
    //avgY += currentLocationMap.y;

    //UPDATE CENTER LOCATION
    //---------------------------------------------
    cnmCenterLocation.x = (avgX);
    cnmCenterLocation.y = (avgY);

    //send to searchController
    //---------------------------------------------
    searchController.setCenterLocation(cnmCenterLocation);
}

void CNMAVGMap()
{

    //NOTES ON THIS FUNCTION:
    //-This function will only run once the rover has completed a full
    // rotation around the octagon and HAS found the center.  At each
    // point, the rover will collect a GPS and Odom location and blend
    // them together by averaging them.

    static bool reached500 = false;
    static int index = 0;
    const int ASIZE = 500;

    //GPS ARRAY
    mapCenterXCoordinates[index] = currentLocationMap.x;
    mapCenterYCoordinates[index] = currentLocationMap.y;

    //ODOM ARRAY
    mapOdomXCoordintes[index] = currentLocation.x;
    mapOdomYCoordinates[index] = currentLocation.y;

    index++;

    if(!reached500 && index == ASIZE + 1) { reached500 = true; }

    float mapAvgX = 0;
    float mapAvgY = 0;
    float odomAvgX = 0;
    float odomAvgY = 0;

    if(!reached500)
    {
        for(int i = 0; i < index; i++)
        {
            mapAvgX += mapCenterXCoordinates[i];
            mapAvgY += mapCenterYCoordinates[i];

            odomAvgX += mapOdomXCoordintes[i];
            odomAvgY += mapOdomYCoordinates[i];
        }

        mapAvgX = (mapAvgX / index);
        mapAvgY = (mapAvgY / index);

        odomAvgX = (odomAvgX / index);
        odomAvgY = (odomAvgY / index);
    }
    else
    {
        for(int i = 0; i < ASIZE; i++)
        {
            mapAvgX += mapCenterXCoordinates[i];
            mapAvgY += mapCenterYCoordinates[i];

            odomAvgX += mapOdomXCoordintes[i];
            odomAvgY += mapOdomYCoordinates[i];
        }

        mapAvgX = (mapAvgX / ASIZE);
        mapAvgY = (mapAvgY / ASIZE);

        odomAvgX = (odomAvgX / ASIZE);
        odomAvgY = (odomAvgY / ASIZE);
    }

    avgCenterRotation.x = ((mapAvgX + odomAvgX) / 2);
    avgCenterRotation.y = ((mapAvgY + odomAvgY) / 2);
}


//CNM TIMER FUNCTIONS
//-----------------------------------

//INITIAL TIMERS

void CNMInitPositioning(const ros::TimerEvent &event)
{
    std_msgs::String msg;
    msg.data = "Initial Wait Time Complete, Driving Forward";
    infoLogPublisher.publish(msg);

    goalLocation.theta = currentLocation.theta;

    //select position 25 cm from the robots location before attempting to go into search pattern
    goalLocation.x = currentLocation.x + (.45 * cos(goalLocation.theta));
    goalLocation.y = currentLocation.y + (.45 * sin(goalLocation.theta));

    cnmHasWaitedInitialAmount = true;
}

void CNMForwardInitTimerDone(const ros::TimerEvent& event)
{

    std_msgs::String msg;
    msg.data = "Finished Driving; Turning 180";
    infoLogPublisher.publish(msg);

    cnmHasMovedForward = true;

    //set NEW heading 180 degrees from current theta
    goalLocation.theta = currentLocation.theta + M_PI;

    //APPROX 45 cm away
    goalLocation.x = currentLocation.x + (.45 * cos(goalLocation.theta));
    goalLocation.y = currentLocation.y + (.45 * sin(goalLocation.theta));

    cnmForwardTimer.stop();
}

void CNMInitialWait(const ros::TimerEvent &e)
{

    std_msgs::String msg;
    msg.data = "Finished 180, Starting Search Pattern";
    infoLogPublisher.publish(msg);

    cnmHasTurned180 = true;
    cnmInitialPositioningComplete = true;
    cnmFirstBootProtocol = false;

    stringstream ss;

    //Continue an interrupted search pattern
    //---------------------------------------------
    goalLocation = searchController.continueInterruptedSearch(currentLocation, goalLocation);

    //ROTATE!!!
    //---------------------------------------------
    stateMachineState = STATE_MACHINE_ROTATE;

    int position = searchController.cnmGetSearchPosition();

    double distance = searchController.cnmGetSearchDistance();

    //SPIT OUT NEXT POINT AND HOW FAR OUT WE ARE GOING
    //---------------------------------------------
    ss << "Traveling to point " << position << " in pattern:  " << distance;
    msg.data = ss.str();
    infoLogPublisher.publish(msg);

    cnmInitialWaitTimer.stop();
}

//OBSTACLE TIMERS

void CNMAvoidObstacle(const ros::TimerEvent &event)
{

    if(centerSeen && targetCollected)
    {
        std_msgs::String msg;
        msg.data = "Continuing To Wait";
        infoLogPublisher.publish(msg);

        cnmAvoidObstacleTimer.stop();
        cnmAvoidObstacleTimer.start();
    }
    else
    {
        std_msgs::String msg;
        msg.data = "Obstacle Avoidance Initiated";
        infoLogPublisher.publish(msg);

        cnmAvoidObstacle = true;

        cnmAvoidObstacleTimer.stop();
    }
}

//TARGET AVOIDANCE

void CNMAvoidOtherTargets(const ros::TimerEvent& event)
{
    std_msgs::String msg;
    msg.data = "Finished avoid timer, trying to return to center";
    infoLogPublisher.publish(msg);

    cnmAvoidTargets = false;
    cnmRotate = false;

    cnmAvoidObstacleTimer.stop();

    if(searchController.cnmIsAlternating()) { goalLocation.theta = currentLocation.theta - (M_PI/6); }
    else { goalLocation.theta = currentLocation.theta + (M_PI/6); }

    //select new position 25 cm from current location
    goalLocation.x = currentLocation.x + (AVOIDTARGDIST * cos(goalLocation.theta));
    goalLocation.y = currentLocation.y + (AVOIDTARGDIST * sin(goalLocation.theta));

    stateMachineState = STATE_MACHINE_ROTATE;

    cnmAvoidOtherTargetTimer.stop();
}

//REVERSE TIMERS

void CNMReverseTimer(const ros::TimerEvent& event)
{
    std_msgs::String msg;
    msg.data = "REVERSE TIMER DONE, STARTING TURN180";
    infoLogPublisher.publish(msg);

    cnmTurn180Timer.start();

    //set NEW heading 180 degrees from current theta
    goalLocation.theta = currentLocation.theta + M_PI;

    double searchDist = searchController.cnmGetSearchDistance();

    //select position however far away we are currently searching from the robots location before attempting to go into search pattern
    goalLocation.x = currentLocation.x + (searchDist * cos(goalLocation.theta));
    goalLocation.y = currentLocation.y + (searchDist * sin(goalLocation.theta));

    //change robot state
    stateMachineState = STATE_MACHINE_ROTATE;

    cnmCheckTimer = 0;

    cnmReverseDone = true;

    cnmReverseTimer.stop();
}

void CNMTurn180(const ros::TimerEvent& event)
{
    cnmTurn180Done = true;

    std_msgs::String msg;
    stringstream ss;

    //Continue an interrupted search pattern
    //---------------------------------------------
    goalLocation = searchController.continueInterruptedSearch(currentLocation, goalLocation);

    //ROTATE!!!
    //---------------------------------------------
    stateMachineState = STATE_MACHINE_ROTATE;

    int position = searchController.cnmGetSearchPosition();

    double distance = searchController.cnmGetSearchDistance();

    //SPIT OUT NEXT POINT AND HOW FAR OUT WE ARE GOING
    //---------------------------------------------
    ss << "Traveling to point " << position << " in pattern:  " << distance;
    msg.data = ss.str();
    infoLogPublisher.publish(msg);

    CNMReverseReset();

    cnmTurn180Timer.stop();
}

//CENTERING TIMERS

void CNMCenterTimerDone(const ros::TimerEvent& event)
{
    cnmCentering = false;
    cnmCenteringFirstTime = true;
    cnmFinishedCenteringTimer.stop();
}

//RESET TIMERS

void cnmWaitToResetWG(const ros::TimerEvent &e)
{
    cnmWaitToReset = false;
    cnmWaitToResetWGTimer.stop();
}

void cnmFinishedPickUpTime(const ros::TimerEvent& e)
{
    cnmFinishedPickUp = true;
    cnmAfterPickUpTimer.stop();
}

//MAP UPDATE?  --NOT USED RIGHT NOW--

void CNMUpdateSearch(const ros::TimerEvent& event)
{
    if(cnmHasCenterLocation)
    {
        //std_msgs::String msg;
        //msg.data = "UPDATING MAP LOCATION";
        //infoLogPublisher.publish(msg);

        //searchController.setCenterLocation(centerLocation);
    }
}
