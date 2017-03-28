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

//CONSTANTS

double CENTEROFFSET = .90;                                  //offset for seeing center
double AVOIDOBSTDIST = .55;                                 //distance to drive for avoiding targets
double AVOIDTARGDIST = .65;                                 //distance to drive for avoiding targets

//ARRAYS FOR CENTER

const int ASIZE = 40;
int centerIndex = 0;
bool maxedCenterArray = false;

//Actual Center Array
float CenterXCoordinates[ASIZE];
float CenterYCoordinates[ASIZE];

geometry_msgs::Pose2D cnmCenterLocation;                    //AVG Center Location spit out by AVGCenter

float avgCurrentCoordsX[ASIZE];
float avgCurrentCoordsY[ASIZE];

geometry_msgs::Pose2D cnmAVGCurrentLocation; 

//Target Collection Variables
//--------------------------------------------

//ORIGINAL FILE VARIABLE:  MOVED HERE FOR EASY LOCATING

float searchVelocity = 0.25;                                 // meters/second  ORIGINALLY .2

//Variables for MobilityStateMachine:  MOVED HERE FOR SCOPE

float rotateOnlyAngleTolerance = 0.15;                       //jms chnaged from .4
int returnToSearchDelay = 8;

//NEST INFORMATION

bool centerSeen = false;                                    //If we CURRENTLY see the center
bool cnmHasCenterLocation = false;                          //If we have a center/nest location at all
bool cnmLocatedCenterFirst = false;                         //If this is the first time we have seen the nest
bool resetMap = false;

//FINDING NEST BEHAVIOR

bool cnmCenteringFirstTime = true;                          //First time entering the Centering function
bool cnmCentering = false;                                  //If we are trying to Center the rover
bool centered = false;                          		    //Confirms if we are centered
bool cnmInitDriveFor = false;                               //The initial drive forward
bool confirmedCenter = false;

//INITIAL NEST SEARCH

bool cnmFirstBootProtocol = true;
bool cnmHasWaitedInitialAmount = false;
bool cnmInitialPositioningComplete = false;
bool cnmHasMovedForward = false;
bool cnmHasTurned180 = false;

//Variables for IF we see the center

double cTagcount = 0;
double cTagcountRight = 0;
double cTagcountLeft = 0;

//Variables for IF we see targets

double numTargets = 0;
double numTargLeft = 0;
double numTargRight = 0;

//Variables for Obstacle Avoidance

bool cnmAvoidObstacle = false;
bool cnmSeenAnObstacle = false;
bool cnmStartObstDetect = false;
bool cnmCanCollectTags = true;                             //Tells the rover if it can collect tags based on if it is avoiding an obst

//Variables for PickUp

bool cnmFinishedPickUp = true;
bool cnmWaitToReset = false;
int numTagsCarrying = 0;

//Variables for DropOff

bool isDroppingOff = false;
bool readyToDrop = false;
bool dropNow = false;
bool seeMoreTargets = false;
bool waitToDrop = false;
bool backUp = false;
bool dropOffReset = false;

//Variables for reverse/180 behvaior
bool firstReverse = true;
bool cnmReverse = false;
bool cnmReverseDone = true;
bool cnmTurn180Done = true;
int cnmCheckTimer = 0;

//Variable for avoiding targets when carrying a target
bool cnmAvoidTargets = false;
bool cnmRotate = false;

//CNM Publishers
//---------------------------------------------
ros::Publisher dropProtocolPub; //dropOffProtocol
ros::Publisher finishedProtocolPub;

//CNM Subscribers
//---------------------------------------------
ros::Subscriber dropProtocolSub;		//dropOffProtocol
ros::Subscriber finishedProtocolSub;

//Times For Timers (IN SECONDS)
//---------------------------------------------
ros::Duration cnm1SecTime(1);
ros::Duration cnm2SecTime(2);
ros::Duration cnm2dot8SecTime(2.8);
ros::Duration cnm3SecTime(3);
ros::Duration cnm4SecTime(4);
ros::Duration cnm5SecTime(5);
ros::Duration cnm6SecTime(6);
ros::Duration cnm8SecTime(8);
ros::Duration cnm10SecTime(10);

//First Boot Timers
//---------------------------------------------

//Waits 10 seconds and Drives Forward
ros::Timer cnmInitialPositioningTimer;

//Waits 10 seconds and Turns 180
ros::Timer cnmForwardTimer;

//Waits 10 Seconds and triggers the robot to continue search
ros::Timer cnmInitialWaitTimer;

//Obstacle Avoidance Timers
//---------------------------------------------

//Waits 10 Seconds before beginning to turn away from targets
ros::Timer cnmAvoidObstacleTimer;

//Waits 10 Seconds before allowing the rovers to start looking for OBST.
//ONLY RAN ONCE AT START UP!
ros::Timer cnmTimeBeforeObstDetect;

//Waits 6 seconds after Obstacle or target is dropped off to pick up targets
ros::Timer cnmWaitToCollectTagsTimer;

//Target Avoidance Timers
//---------------------------------------------
ros::Timer cnmAvoidOtherTargetTimer;

//Reverse Timers
//---------------------------------------------
ros::Timer cnmReverseTimer;
ros::Timer cnmWaitToResetWGTimer;

//DropOff Timers
//---------------------------------------------
ros::Timer cnmDropOffDriveTimer;
ros::Timer cnmDropOffReverse;

//180 Timers(TIED TO REVERSE)
//---------------------------------------------
ros::Timer cnmTurn180Timer;

//Centering Timer (used to center rover and find more accurate point to
    //translate centers position to
//---------------------------------------------
ros::Timer cnmFinishedCenteringTimer;
ros::Timer cnmAfterPickUpTimer;
ros::Timer cnmConfirmCentered;

//CNM moved ORIGINAL CODE to utility functions
//---------------------------------------------
void InitComp();                                                //INIT COMPONENTS (Builds map, sets Pose 2D Objects defaults)

bool CNMTransformCode();                                        //A function for the Transform segment in Mobility State Machine
                                                                //	- returns true only if it needs to return
bool CNMPickupCode();                                           //A function for PickUpController in Mobility State Machine
                                                                //	- returns false if it needs to break
bool CNMRotateCode();                                           //A function for the Rotate Mobility State Machine Code

void CNMSkidSteerCode();                                        //A function with all the skid steer mobility code

bool CNMDropOffCode();                  		        //CNM ADDED:  More Controll over Drop Off
bool CNMDropoffCalc();

//---------------------------------------------

void CNMFirstBoot();                                            //Code for robot to run on initial switch to autonomous mode

//NEW REVERSE ATTEMPT
void CNMStartReversing();                                       //Begins Reverse Timers
void CNMReverseReset();                                         //Resets Reverse Variables

void CNMFirstSeenCenter();                                      //Initial Center Find Code
void CNMRefindCenter();                                         //Refind Center Code

void CNMTargetPickup(PickUpResult result);                      //Wrist/Gripper Setting on pickup

void CNMTargetAvoid();                                          //Code To Avoid Targets

bool CNMCentered();                                             //Squares Rover up on nest when found

void CNMAVGCenter();                                            //Avergages derived center locations
void CNMProjectCenter();                        				//Projects current location forward to the center

void CNMCenterGPS(int index);                                   //When we see center, we start storing GPS locations
void CNMAVGCenterGPS(bool hitMax, int index);

bool CNMCurrentLocationAVG();

void CNMReactToCenterProtocol();

void CNMCheckIfPickingUp();

//Pub/Sub Callback Handlers
//-----------------------------------
void dropProtocolHandler(const std_msgs::String& msg);			//dropOffProtocol
void finishedProtocolHandler(const std_msgs::String& msg);

//Timer Functions/Callbacks Handlers
//-----------------------------------

//INITIAL NEST SEARCH
void CNMInitPositioning(const ros::TimerEvent& event);          //Wait Before Driving Forward
void CNMForwardInitTimerDone(const ros::TimerEvent& event);     //Wait Before 180
void CNMInitialWait(const ros::TimerEvent& e);                  //Wait Before Continued Search

//TIMER FOR SQUARING UP ON NEST
void CNMCenterTimerDone(const ros::TimerEvent& event);          //Timer before telling rover it has finished squaring up on nest
void CNMWaitToConfirm(const ros::TimerEvent& event);

//PICKUP TIMERS/DROP OFF TIMERS
void cnmFinishedPickUpTime(const ros::TimerEvent& e);           //Wait before detecting other tags to avoid
void cnmWaitToResetWG(const ros::TimerEvent& e);                //After Dropping off, wait before resetting grippers

void CNMDropOffDrive(const ros::TimerEvent& e);     			//Timer to drive forward before drop off attempt in center
void CNMDropReversed(const ros::TimerEvent& e);

//REVERSE TIMER
void CNMReverseTimer(const ros::TimerEvent& event);             //REVERSES
void CNMTurn180(const ros::TimerEvent& event);                  //Turns 180

//Obstacle Avoidance Timer
void CNMAvoidObstacle(const ros::TimerEvent& event);            //Timer Function(when timer fires, it runs this code)
void CNMWaitBeforeDetectObst(const ros::TimerEvent& event);     //When called, triggers cnmStartObstDetect to true, allowing rover to start avoiding obstacles

void CNMWaitToCollectTags(const ros::TimerEvent& event);        //Handler triggers state to start collecting targets again

//Target Avoidance Timer
void CNMAvoidOtherTargets(const ros::TimerEvent& event);        //NOT BEING USED

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

    //CNM Publishers
    //----------------------------------------------------
    dropProtocolPub = mNH.advertise<std_msgs::String>("dropBool", 1000);                    //dropOffProtocol
    finishedProtocolPub = mNH.advertise<std_msgs::String>("dropFinished", 1000);

    //CNM Subscribers
    //----------------------------------------------------
    dropProtocolSub = mNH.subscribe("dropBool", 1000, &dropProtocolHandler);                //dropOffProtocol
    finishedProtocolSub = mNH.subscribe("dropFinished", 1000, &finishedProtocolHandler);

    //CNM TIMERS
    //----------------------------------------------------

    //-----INITIAL CENTER FIND TIMERS-----

    //Waits Before Driving Forward
    cnmInitialPositioningTimer = mNH.createTimer(cnm10SecTime, CNMInitPositioning, true);
    cnmInitialPositioningTimer.stop();

    //Waits before Turning 180 Degrees
    cnmForwardTimer = mNH.createTimer(cnm10SecTime, CNMForwardInitTimerDone, true);
    cnmForwardTimer.stop();

    //Waits before Running Interrupted Search
    cnmInitialWaitTimer = mNH.createTimer(cnm10SecTime, CNMInitialWait, true);
    cnmInitialWaitTimer.stop();

    //-----REVERSE BEHAVIOR TIMERS------

    //Timer for mandatory reversing
    cnmReverseTimer = mNH.createTimer(cnm2SecTime, CNMReverseTimer, true);
    cnmReverseTimer.stop();    

    //Timer for turning 180 degrees
    cnmTurn180Timer = mNH.createTimer(cnm8SecTime, CNMTurn180, true);
    cnmTurn180Timer.stop();

    //-----PICKUP TIMERS-----

    //Waits a time after pickup before viewing other targets as obstacles
    cnmAfterPickUpTimer = mNH.createTimer(cnm4SecTime, cnmFinishedPickUpTime, true);
    cnmAfterPickUpTimer.stop();

    //-----DROPOFF TIMERS-----

    //cnmDropOffDriveTimer = mNH.createTimer(cnm2dot8SecTime, CNMDropOffDrive, true);  //ROVER
    cnmDropOffDriveTimer = mNH.createTimer(cnm4SecTime, CNMDropOffDrive, true);    //SIM
    cnmDropOffDriveTimer.stop();

    cnmDropOffReverse = mNH.createTimer(cnm4SecTime, CNMDropReversed, true);
    cnmDropOffReverse.stop();

    //Waits to reset Wrist/Gripper to a lowered driving state (Prevents trapping blocks under gripper)
    cnmWaitToResetWGTimer = mNH.createTimer(cnm2SecTime, cnmWaitToResetWG, true);
    cnmWaitToResetWGTimer.stop();

    //AVOIDING TARGETS IF CARRYING ONE
    cnmAvoidOtherTargetTimer = mNH.createTimer(cnm4SecTime, CNMAvoidOtherTargets, true);
    cnmAvoidOtherTargetTimer.stop();

    //-----OBSTACLE AVOIDANCE-----

    //Timer for Obstacle Avoidance
    cnmAvoidObstacleTimer = mNH.createTimer(cnm10SecTime, CNMAvoidObstacle, true);
    cnmAvoidObstacleTimer.stop();

    //Timer to allow rovers to start detecting Obstacles
    cnmTimeBeforeObstDetect = mNH.createTimer(cnm8SecTime, CNMWaitBeforeDetectObst, true);
    cnmTimeBeforeObstDetect.stop();

    //Timer to allow rovers to start picking up tags again
    cnmWaitToCollectTagsTimer = mNH.createTimer(cnm3SecTime, CNMWaitToCollectTags, true);
    cnmWaitToCollectTagsTimer.stop();

    //-----CENTERFIND TIMERS-----

    //CENTERING TIMER
    cnmFinishedCenteringTimer = mNH.createTimer(cnm4SecTime, CNMCenterTimerDone, true);
    cnmFinishedCenteringTimer.stop();

    cnmConfirmCentered = mNH.createTimer(cnm1SecTime, CNMWaitToConfirm, true);
    cnmConfirmCentered.stop();

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

        if(!cnmReverseDone && cnmReverse) 
	{ 
            sendDriveCommand(-0.2, 0.0);

	    return;
	}

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
		//GRIPPER OPTIMUM SETTING:
		//FINGERS:  0 - 2      (Any further and fingers deform[AKA right finger keeps rotating and left doesn't])
		//WRIST:    0 - 1.6    (Any further and will scrape ground if rover hits bumps)

                // set gripper
                std_msgs::Float32 angle;

                // close fingers all the way
                angle.data = 0.0;

                fingerAnglePublish.publish(angle);

                //raise wrist partially (avoid obstacle calls)
                angle.data = 1.0;	//0.6 is to avoid dragging cube on the ground

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

        case STATE_MACHINE_DROPOFF: 
	{
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

    centerSeen = false;             //set to false
    cTagcount = 0;
    cTagcountRight = 0;
    cTagcountLeft = 0;

    numTargets = 0;
    numTargLeft = 0;
    numTargRight = 0;

    //If a target is detected and we are looking for center tags
    if (message->detections.size() > 0 && !reachedCollectionPoint)
    {
        //IMPORTANT VARIABLES
        //---------------------------------------------
        float cameraOffsetCorrection = 0.020; //meters;

        //IF WE SEE A CENTER TAG LOOP: this gets # number of center tags
        //---------------------------------------------
        for (int i = 0; i < message->detections.size(); i++)
        {
            if (message->detections[i].id == 256)
            {
                geometry_msgs::PoseStamped cenPose = message->detections[i].pose;

                // checks if tag is on the right or left side of the image
                if (cenPose.pose.position.x + cameraOffsetCorrection > 0) { cTagcountRight++; }
                else { cTagcountLeft++; }

                centerSeen = true;
                cnmHasCenterLocation = true;
                cTagcount++;
            }
            else if(message->detections[i].id == 0)
            {
                geometry_msgs::PoseStamped cenPose = message->detections[i].pose;

                numTargets++;
                if (cenPose.pose.position.x + cameraOffsetCorrection > 0) { numTargRight++; }
                else { numTargLeft++; }
            }
        }

        if(numTargets == 0 && isDroppingOff) { seeMoreTargets = 0; }

        //dropOffController.setDataTargets(count,countLeft,countRight);

        //CNM MODIFIED: If we see the center and don't have a target collected
        //---------------------------------------------
        if(centerSeen && !targetCollected && cTagcount > 2)
        {
	    CNMReactToCenterProtocol();

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

    //If we see an april tag, are not carrying a target, and if timer is ok
    //---------------------------------------------
    if (numTargets > 0 && !targetCollected && timerTimeElapsed > 5)
    {
        //Check to see if have found the nests location at all
        //---------------------------------------------
        if(cnmHasCenterLocation)
        {
            //If we are't allowed to pick up a tag
            //---------------------------------------------
            if (!cnmCanCollectTags)
            {
                //This code is to prevent the rover from picking up targets at inopportune moments
                    //- Called After Successful DropOff  (So it doesn't try to pick up blocks in center)
                    //- Called After Avoiding Obstacle   (So it doesn't try to pick up blocks being carried by other rovers)

                //Ignore the tag, keep avoiding the obstacle
                targetDetected = false;

                //go back to top of state machine
                stateMachineState = STATE_MACHINE_TRANSFORM;

                //reset pickup controller
                pickUpController.reset();

                //cnmCanCollectTags is set to true on a short timer triggered after avoiding an obstacle
            }

            //If we see the center, ignore the target and back up!
            //---------------------------------------------
            else if(centerSeen)
            {
                //reset target detected
                targetDetected = false;

                //send state to rotate
                stateMachineState = STATE_MACHINE_TRANSFORM;

                //Project center point
                CNMProjectCenter();

                //reset pickup
                pickUpController.reset();

                //reverse
                CNMStartReversing();
            }

            //Pick Up The Target
            //---------------------------------------------
            else
            {
                //Check to see if we are currently trying to reverse
                //---------------------------------------------
                if(!cnmReverse || (cnmReverse && cnmReverseDone))
                {
                    CNMReverseReset();

                    targetDetected = true;

                    //pickup state so target handler can take over driving.
                    //---------------------------------------------
                    stateMachineState = STATE_MACHINE_PICKUP;
                    result = pickUpController.selectTarget(message);

                    CNMTargetPickup(result);
                }
            }
        }

        //If the rover has not gotten the centers point, or lost it, avoid the target
        //---------------------------------------------
        else
        {
            //reset target detected
            targetDetected = false;

            //reset pickup
            pickUpController.reset();

            //Avoid
            CNMTargetAvoid();
        }
    }

    //CNM ADDED: if we see a target and have already picked one up
    //---------------------------------------------
    else if(numTargets > numTagsCarrying && targetCollected && cnmFinishedPickUp)
    {    
        //std_msgs::String msg;
        //msg.data = "Oops I got here";
        //infoLogPublisher.publish(msg);
    
        //Avoid Targets
        //---------------------------------------------
        //- Need to check for multiple conditions
            //1.) Are we dropping off?
            //2.) Are we on our way back?

        //check to see if dropping off
        if(isDroppingOff) { seeMoreTargets = true; }
        else { CNMTargetAvoid(); }
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
        //If we can start looking for obstacles
        if(cnmStartObstDetect)
        {            
            cnmSeenAnObstacle = true;                       //We saw an obstacle

	    if(!cnmCanCollectTags)
	    {
                cnmWaitToCollectTagsTimer.stop();
	    }
	    else
	    {
                cnmCanCollectTags = false;                      //Don't try picking anything up
	    }

	    if(targetDetected)
	    {
		targetDetected = false;

                //reset pickup
                pickUpController.reset();
	    }

            cnmAvoidObstacleTimer.start();

            if(!cnmAvoidObstacle)
            {
                if(firstTimeSeeObst)
                {
                    std_msgs::String msg;
                    msg.data = "I see an obstacle; Stopping and waiting";
                    infoLogPublisher.publish(msg);
                    firstTimeSeeObst = false;
                }

                sendDriveCommand(0.0, 0.0);
            }
            else
            {
                if(firstTimeRotate)
                {
                    searchController.obstacleWasAvoided();
                    firstTimeRotate = false;
                }

                //no matter what, turn left
                sendDriveCommand(0.0, 0.2);
            }
        }
    }

    //if we saw an obstacle but no longer see one
    else if (cnmSeenAnObstacle && (!targetDetected || targetCollected) && (message->data == 0))
    {
        cnmSeenAnObstacle = false;                      //We no longer see an obstacle

	if(!cnmCanCollectTags)
	{
	     cnmWaitToCollectTagsTimer.start();		//Start timer to start looking for targets again
	}

    	if(cnmAvoidObstacle)
    	{
       	    if(!firstTimeRotate)
            {
                //std_msgs::String msg;
                //msg.data = "DRIVING 30 degrees to the left!";
                //infoLogPublisher.publish(msg);

                firstTimeRotate = true;
            }

            goalLocation.theta = currentLocation.theta + (M_PI/6);

            goalLocation.x = currentLocation.x + (AVOIDOBSTDIST * (cos(goalLocation.theta)));
	    goalLocation.y = currentLocation.y + (AVOIDOBSTDIST * (sin(goalLocation.theta)));

	    //stateMachineState = STATE_MACHINE_ROTATE;

	    cnmAvoidObstacle = false;
        }
        else
        {
            cnmAvoidObstacleTimer.stop();                   //Double tap stopping the timer in case obstacle moves
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

//MOBILITY TRANSFORM STATES
//-----------------------------------

bool CNMTransformCode()
{

   static bool isCalculating = false;
   static bool firstCantTransform = true;
//MUST TEST:  using currentLocation vs using currentMapLocation

   // If returning with a target
    if (targetCollected && !avoidingObstacle)
    {
        if(CNMDropOffCode()) { return false; }
    }

    //If angle between current and goal is significant
    //if error in heading is greater than 0.4 radians

    //TRY MAP??
    if (fabs(angles::shortest_angular_distance(currentLocation.theta, goalLocation.theta)) >
        rotateOnlyAngleTolerance && !isCalculating)
    {
        stateMachineState = STATE_MACHINE_ROTATE;
    }

    //If goal has not yet been reached drive and maintain heading
    else if (fabs(angles::shortest_angular_distance(currentLocation.theta,
        atan2(goalLocation.y - currentLocation.y, goalLocation.x - currentLocation.x))) < M_PI_2 && !isCalculating)
    {
        stateMachineState = STATE_MACHINE_SKID_STEER;
    }

    //Otherwise, drop off target and select new random uniform heading
    //If no targets have been detected, assign a new goal

    else if (!targetDetected && timerTimeElapsed > returnToSearchDelay && cnmInitialPositioningComplete)
    //else if(!targetDetected && distToGoal < 0.5 && timerTimeElapsed > returnToSearchDelay && cnmInitialPositioningComplete)
    {
	std_msgs::String msg;

	static bool firstTimeCalculate = true;
	bool goToNextPoint = CNMCurrentLocationAVG();

    	float distToGoal = hypot(goalLocation.x - currentLocation.x, goalLocation.y - currentLocation.y);    

	if(!centerSeen && goToNextPoint && !isDroppingOff)
	{
            firstTimeCalculate = true;
	    isCalculating = false;
            int position;
            double distance;
	    int rotations;
    	    bool hasRotated = searchController.getHasDoneRotation();

            goalLocation = searchController.search(cnmAVGCurrentLocation);

            position = searchController.cnmGetSearchPosition();

            distance = searchController.cnmGetSearchDistance();

	    rotations = searchController.cnmGetNumRotations();
	
            stringstream ss;
            ss << "Traveling to point " << position << " at " << distance << " meters; DIST: " << distToGoal << " Rotations: " << rotations;

	    if(hasRotated) { ss << " " << "TRUE"; }
	    else { ss << " " << "FALSE"; }
            
	    msg.data = ss.str();
            infoLogPublisher.publish(msg);
	}
	else if(!goToNextPoint || distToGoal < 3.8)
	{
	    if(firstTimeCalculate)
            {
	        msg.data = "Calculating point to go to";
	        infoLogPublisher.publish(msg);
		firstTimeCalculate = false;
		isCalculating = true;
	    }

	    if(cnmReverse) { CNMReverseReset(); }


	    sendDriveCommand(0.0, 0.0);
	}

	else if(isDroppingOff)
	{
	    msg.data = "Haven't finished drop off, reversing";
	    infoLogPublisher.publish(msg);
	    sendDriveCommand(-0.2, 0.0);
	}	
	
	else 
	{ 
	    msg.data = "Did I make it here?";
	    infoLogPublisher.publish(msg);
	    sendDriveCommand(0.0, 0.0); 
	}
	
    }
    else
    {
	if(firstCantTransform)
	{
	    std_msgs::String msg;
	    stringstream ss;
	    ss << "Time:  " << timerTimeElapsed << " can't transform";
            msg.data = ss.str();
            infoLogPublisher.publish(msg);	
	    goalLocation = currentLocation;
	    firstCantTransform = false;        
	}

	sendDriveCommand(0.0, 0.0); 
	stateMachineState = STATE_MACHINE_TRANSFORM;
    }

    return true;
}

bool CNMRotateCode()
{
    //MUST TEST:  using currentLocation vs using currentLocationMap

    // Calculate the diffrence between current and desired
    // heading in radians.
    float errorYaw = angles::shortest_angular_distance(currentLocation.theta, goalLocation.theta);
    int dirToRotate = 1;

    if(errorYaw < 0) { dirToRotate = -1; }

    // If angle > 0.4 radians rotate but dont drive forward.
    if (fabs(angles::shortest_angular_distance(currentLocation.theta, goalLocation.theta)) > rotateOnlyAngleTolerance)
    {

	float turnSpeed = 0.25 * dirToRotate;
        // rotate but dont drive 0.05 is to prevent turning in reverse
        sendDriveCommand((-0.05 * dirToRotate), turnSpeed);
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

//MUST TEST:  Using currentLocation vs currentLocationMap

    // calculate the distance between current and desired heading in radians
    float errorYaw = angles::shortest_angular_distance(currentLocation.theta, goalLocation.theta);

    // goal not yet reached drive while maintaining proper heading.
    if (fabs(angles::shortest_angular_distance(currentLocation.theta, atan2(goalLocation.y - currentLocation.y, goalLocation.x - 	    currentLocation.x))) < M_PI_2)
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

        // move back to transform step
        stateMachineState = STATE_MACHINE_TRANSFORM;
    }
}

bool CNMPickupCode()
{

    static bool pickedUp = false;
    PickUpResult result;

    //GRIPPER OPTIMUM SETTING:
    //FINGERS:  0 - 2      (Any further and fingers deform[AKA right finger keeps rotating and left doesn't])
    //WRIST:    0 - 1.6    (Any further and will scrape ground if hits bumps)

    // we see a block and have not picked one up yet
    //CNM ADDED:    AND if we are not doing our reverse behavior
    if (targetDetected && !targetCollected && !cnmReverse  && cnmCanCollectTags)
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

            //assume target has been picked up by gripper
            targetCollected = true;

	    pickedUp = true;			//We have something picked up
            cnmFinishedPickUp = false;		//We are now waiting to finish our pickup
            result.pickedUp = false;

            // lower wrist to avoid ultrasound sensors
            std_msgs::Float32 angle;
            angle.data = 1.2;  //.8
            wristAnglePublish.publish(angle);
        }
    }
    else if(pickedUp)
    {
        static bool firstTimeIn = true;

        if(firstTimeIn)
        {
            std_msgs::String msg;
            msg.data = "Calculating way back home";
            infoLogPublisher.publish(msg);
            firstTimeIn = false;
        }

        if(CNMCurrentLocationAVG())
        {
            pickUpController.reset();

            std_msgs::String msg;
            msg.data = "I should be going back home";
            infoLogPublisher.publish(msg);

            //TEST:  MAP VS ODOM
            goalLocation.theta = atan2(cnmCenterLocation.y - currentLocationAverage.y, cnmCenterLocation.x - currentLocationAverage.x);

            // set center as goal position
            goalLocation.x = cnmCenterLocation.x;
            goalLocation.y = cnmCenterLocation.y;

            //Hand off to rotate
            stateMachineState = STATE_MACHINE_TRANSFORM;

            cnmAfterPickUpTimer.start();

            pickedUp = false;
            firstTimeIn = true;
            return true;
        }
        else
        {
	    if(searchController.cnmIsAlternating()) { sendDriveCommand(0.0, -0.15); }
	    else { sendDriveCommand(0.0, 0.15); }
        }
    }
    else
    {
        stateMachineState = STATE_MACHINE_TRANSFORM;
    }

    return false;
}

bool CNMDropOffCode()
{	
	static bool firstCenterSeen = true;
	static bool readyGoForward = false;
	static bool firstInForward = true;
	static bool tryAgain = false;
	static bool IWasLost = false;
	static bool startDropOff = false;
	static bool searchingForCenter = false; 

	bool atCenter = CNMDropoffCalc();

	if(dropOffReset)
	{
	    //RESET!
	    //---------------
       	    timerStartTime = time(0);
      	    targetCollected = false;
       	    targetDetected = false;
       	    lockTarget = false;

      	    cnmWaitToReset = true;
       	    cnmWaitToResetWGTimer.start();

            readyToDrop = false;
            dropNow = false;
            firstCenterSeen = true;
            readyGoForward = false;
            firstInForward = true;
	    tryAgain = false;
	    startDropOff = false;
	    searchingForCenter = false;
	    backUp = false;
	    dropOffReset = false;

            finishedProtocolPub.publish(msg);           //dropoffProtocol

	    cnmWaitToCollectTagsTimer.start();      	//Start Timer to trigger back to true

	    CNMReverseReset();
            CNMStartReversing();		

    	    return false;
	}

	else if(backUp)
	{
	    sendDriveCommand(-0.2, 0.0);
	    cnmDropOffReverse.start();
	}

	//if we are officially dropping target off
	else if(dropNow)
	{
	    //DROP
	    //---------------

            // set gripper
            std_msgs::Float32 angle;

            //open fingers all the way
            angle.data = 2;  //(0-2 is a good range to open and close grippers)

            fingerAnglePublish.publish(angle);

            //raise wrist partially (avoid obstacle calls)
            angle.data = 0;

            //raise wrist
            wristAnglePublish.publish(angle);

	    if(IWasLost)
	    {
		IWasLost = false;
		searchController.AmILost(false);

		maxedCenterArray = false;
	        centerIndex = 0;    			

		//add current position to center array
	    }

	    //store one GPS x and y
	    CenterXCoordinates[centerIndex] = currentLocationMap.x;
    	    CenterYCoordinates[centerIndex] = currentLocationMap.y;

	    CNMAVGCenter();

	    //store one ODOM x and y
	    CenterYCoordinates[centerIndex] = currentLocation.y;
	    CenterXCoordinates[centerIndex] = currentLocation.x;

	    CNMAVGCenter();

	    backUp = true;		    	    
	    cnmCanCollectTags = false;              	//Don't try to collect tags

	}

	//If we THINK we are in a position to drop off a tag
	else if(readyToDrop)
	{

            std_msgs::String msg;
            msg.data = "Squared up; Driving forward";
            infoLogPublisher.publish(msg);

	    //We drove forward onto the center parallel to the tags... reverse
	    if(cTagcount > 8)
	    {				
            	msg.data = "Tags greater than 8; Dropping anyway";
            	infoLogPublisher.publish(msg);

		dropNow = true;

		sendDriveCommand(-0.2, 0.0);

		//tryAgain = true;
		//readyToDrop = false;
	    }
	    else if(cTagcount > 3 && cTagcount <= 8)
	    {

            	double turnDirection = 0.0;
			
            	stringstream ss;
            	msg.data = "Tags between 3 and 8";
            	infoLogPublisher.publish(msg);

	        if(cTagcountLeft < (cTagcountRight - 6))
	        {
		    ss << "Turning Left:  " << cTagcountLeft << " > " << cTagcountRight;
		    turnDirection = 0.15;
	        }
	        else if(cTagcountLeft > (cTagcountRight - 6))
	        {
		    ss << "Turning Right:  " << cTagcountLeft << " < " << cTagcountRight;
		    turnDirection = -0.15;
	        }
	        else if((cTagcountLeft - 6) <= 0 && (cTagcountRight - 6) <= 0)
	        {
		    ss << "Tag Count is even" << cTagcountLeft << " = " << cTagcountRight;
	        }

            	sendDriveCommand(0.0, turnDirection);

            	msg.data = ss.str();
            	infoLogPublisher.publish(msg);
	    }
	    else
	    {
            	msg.data = "Tags less than 3; Dropping off!";
            	infoLogPublisher.publish(msg);
		
		sendDriveCommand(-0.2, 0.0);

            	dropNow = true;
	    }
	}

	//If we drove forward onto the center parallel with the tags and have reversed far enough ... reset
	else if(tryAgain && !readyToDrop)	    
	{
	    if(cTagcount > 5) { sendDriveCommand(-0.15, 0.0); }
	    else { readyGoForward = false; tryAgain = false; startDropOff = false;}
	}

    	//Once somewhat straightened out, go forward
	else if(readyGoForward)
	{
	    if(firstInForward)
	    {
//	    	std_msgs::String msg;
//            	msg.data = "Squared up; Driving forward";
//            	infoLogPublisher.publish(msg);
            	firstInForward = false;
	    }
	    
	    //Once Squared Up, trigger in position
	    sendDriveCommand(0.15, 0.0);
	    cnmDropOffDriveTimer.start();
	}

	else if(startDropOff)
	{	    	    
	    //check to see if we can move forward
	    readyGoForward = CNMCentered();
	}

	//if we see the center
	else if(centerSeen)
	{
	    if(firstCenterSeen)
	    {
            	std_msgs::String msg;
            	msg.data = "Found center; Dropping Off";
            	infoLogPublisher.publish(msg);

            	isDroppingOff = true;
            	firstCenterSeen = false;

                dropProtocolPub.publish(msg); 	//triggers waitToDrop	//dropOffProtocol
	    }
	    
            goalLocation = currentLocation;
	    startDropOff = true;
	}
	
	//If we are looking for the center
	else if(searchingForCenter)
	{
	    goalLocation = searchController.search(currentLocationMap);
	    stateMachineState = STATE_MACHINE_ROTATE;
	}

	//If we should have found the center by now
	else if(atCenter && !centerSeen)
	{
	    std_msgs::String msg;
            msg.data = "Where am I? I don't see the Nest! Better Look!";
            infoLogPublisher.publish(msg);

	    searchController.AmILost(true);
	    searchController.setCenterLocation(currentLocation);
	    IWasLost = true;

	    //Start Looking!
	    searchingForCenter = true;

	    stateMachineState = STATE_MACHINE_ROTATE;
	}
	else
 	{
	    static bool change = false;

	    std_msgs::String msg;

            //TEST:  MAP VS ODOM
	    if(change)
	    {

            	msg.data = "Can't Find Center, using GPS";
            	goalLocation.theta = atan2(cnmCenterLocation.y - currentLocationMap.y, cnmCenterLocation.x - currentLocationMap.x);
		change = false;
	    }
	    else
	    {
		msg.data = "Can't Find Center, using Odom";
		goalLocation.theta = atan2(cnmCenterLocation.y - currentLocation.y, cnmCenterLocation.x - currentLocation.x);
		change = true;
	    }

	    infoLogPublisher.publish(msg);

            // set center as goal position
            goalLocation.x = cnmCenterLocation.x;
            goalLocation.y = cnmCenterLocation.y;

            stateMachineState = STATE_MACHINE_ROTATE;
	}

    return true;

}

bool CNMDropoffCalc()
{

    // calculate the euclidean distance between
    // centerLocation and currentLocation
    float distToCenter = hypot(cnmCenterLocation.x - currentLocation.x, cnmCenterLocation.y - currentLocation.y);
    //float distToCenter = hypot(cnmCenterLocation.x - currentLocationMap.x, cnmCenterLocation.y - currentLocationMap.y);

    float visDistToCenter = 0.8;

    if(distToCenter > visDistToCenter) { return false; }
    else { return true; }
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

        goalLocation = currentLocation;
        sendDriveCommand(0.0, 0.0);

        firstTimeInBoot = false;

        //START TIMER to move forward
        cnmInitialPositioningTimer.start();

        //START OBSTACLE DETECTION TIMER
        cnmTimeBeforeObstDetect.start();
    }

    //IF WE SEE CENTER, BREAK EVERYTHING
    if(centerSeen)
    {
        cnmFirstBootProtocol = false;
        cnmHasTurned180 = true;

        cnmInitialPositioningTimer.stop();
        cnmForwardTimer.stop();
        cnmInitialWaitTimer.stop();
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
//              std_msgs::String msg;
//              msg.data = "Finishing 180, waiting before starting search";
//              infoLogPublisher.publish(msg);

                firstIn180 = false;
            }

            //IF we HAVEN'T finished turning 180
            if(!cnmHasTurned180)
            {
                static bool firstInWait = true;

                if(firstInWait)
                {
//                  std_msgs::String msg;
//                  msg.data = "Finishing 180, waiting before starting search";
//                  infoLogPublisher.publish(msg);
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
//              std_msgs::String msg;
//              msg.data = "Starting Timer To Turn 180";
//              infoLogPublisher.publish(msg);
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

    //if(cnmCentering)
    //{
//	cnmCentering = false;
//	cnmFinishedCenteringTimer.stop();
//    }

}

void CNMStartReversing()
{
    //Pass Info to Info Log
    //---------------------------------------------
//    std_msgs::String msg;
//    msg.data = "STARTING REVERSE TIMER";
//    infoLogPublisher.publish(msg);

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
    //Try to keep targets in the left
    if(numTargLeft > numTargRight) { goalLocation.theta = currentLocation.theta - (M_PI/6); }
    else { goalLocation.theta = currentLocation.theta + (M_PI/6); }

    //select new position 25 cm from current location
    goalLocation.x = currentLocation.x + (AVOIDTARGDIST * cos(goalLocation.theta));
    goalLocation.y = currentLocation.y + (AVOIDTARGDIST * sin(goalLocation.theta));

//    stateMachineState = STATE_MACHINE_ROTATE;

    searchController.obstacleWasAvoided();
}

//Rotation for squaring up on center
bool CNMCentered()
{
    float linearSpeed, angularSpeed;

    //VARIABLES
    //-----------------------------------
    const int amountOfTagsToSee = 5;
    bool seenEnoughTags = false;
    bool right, left;

    if(!cnmCentering) 
    { 
	cnmCentering = true; 
    }
    else
    {
// 	  cnmFinishedCenteringTimer.stop();
//        cnmFinishedCenteringTimer.start();
    }

    goalLocation = currentLocation;

    if(!cnmReverse)
    {

        if (cTagcountRight > 0) { right = true; }
        else { right = false; }

        if (cTagcountLeft > 0) { left = true; }
        else { left = false; }

        if(cTagcount > amountOfTagsToSee) { seenEnoughTags = true;}
        else { seenEnoughTags = false; }

        float turnDirection = 1;

        if (seenEnoughTags) //if we have seen enough tags
        {
            if ((cTagcountLeft - 5) > cTagcountRight) //and there are too many on the left
            {
            	right = false; //then we say none on the right to cause us to turn right
            }
            else if ((cTagcountRight - 5) > cTagcountLeft)
            {
            	left = false; //or left in this case
            }

            //otherwise turn till tags on both sides of image then drive straight
	    if(confirmedCenter && left && right)
	    {
		confirmedCenter = false;
		centered = true;
        	cnmFinishedCenteringTimer.start();
		return true;
	    }
            else if(left && right) 
	    { 
		if(isDroppingOff)
		{
		    cnmConfirmCentered.start();

		    goalLocation = currentLocation;
		    sendDriveCommand(0.0, 0.0);
		    return false;
		}
		else
		{
        	    cnmFinishedCenteringTimer.start();
		    return true;
		}
	    }
        }
	
	cnmConfirmCentered.stop(); 
	cnmFinishedCenteringTimer.stop();

        if (right)
    	{
            linearSpeed = 0.15;
            angularSpeed = -0.15;
    	}
        else if (left)
    	{
            linearSpeed = -0.15;
            angularSpeed = 0.15;
        }
        else
        {
	    if(isDroppingOff) { linearSpeed = 0.25; }
	    else { linearSpeed = 0.15; }
	
	    angularSpeed = 0.0;
        }

        sendDriveCommand(linearSpeed, angularSpeed);
    }

    return false;

}

//Seeing Center Behavior

void CNMFirstSeenCenter()
{
    //Print out to the screen we found the nest for the first time
    std_msgs::String msg;
    msg.data = "Found Initial Nest Location";
    infoLogPublisher.publish(msg);

    //change bool
    cnmLocatedCenterFirst = true;
    searchController.AmILost(false);

    if(cnmInitialPositioningComplete)
    {
        msg.data = "Search Pattern Expanding";
        infoLogPublisher.publish(msg);
    }

    CNMProjectCenter();

    CNMStartReversing();
}

void CNMRefindCenter()
{
    std_msgs::String msg;
    msg.data = "Refound center, updating location";
    infoLogPublisher.publish(msg);

    CNMProjectCenter();

    CNMStartReversing();
}

//CNM MAP BUILDING

void CNMProjectCenter()
{
    //NOTES ON THIS FUNCTION:
    //- Takes current point and projects it out to where the center SHOULD be
    //- Calls CNMAVGCenter to avg new center location searchController new avg center

    if(resetMap)
    {
	resetMap = false;
	maxedCenterArray = false;
	centerIndex = 0;
    }

    //NORMALIZE ANGLEf
    double normCurrentAngle = angles::normalize_angle_positive(currentLocationMap.theta);

    //CenterXCoordinates[centerIndex] = currentLocation.x + (CENTEROFFSET * (cos(normCurrentAngle)));
    CenterXCoordinates[centerIndex] = currentLocationMap.x + (CENTEROFFSET * (cos(normCurrentAngle)));
    //CenterYCoordinates[centerIndex] = currentLocation.y + (CENTEROFFSET * (sin(normCurrentAngle)));
    CenterYCoordinates[centerIndex] = currentLocationMap.y + (CENTEROFFSET * (sin(normCurrentAngle)));

    CNMAVGCenter();
}

void CNMAVGCenter()
{   

    if(resetMap)
    {
	resetMap = false;
	maxedCenterArray = false;
	centerIndex = 0;
    }

//    std_msgs::String msg;
//    msg.data = "Averaging Center Locations";
//    infoLogPublisher.publish(msg);

    if(centerIndex >= ASIZE)
    {
        if(!maxedCenterArray) {  maxedCenterArray = true; }
        centerIndex = 0;
    }
    else
    {
        centerIndex++;
    }

    float avgX = 0;
    float avgY = 0;

    if(!maxedCenterArray)
    {
        for(int i = 0; i < centerIndex; i++)
        {
            avgX += CenterXCoordinates[i];
            avgY += CenterYCoordinates[i];
        }

        avgX = (avgX / centerIndex);
        avgY = (avgY / centerIndex);
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

    //UPDATE CENTER LOCATION
    //---------------------------------------------
    cnmCenterLocation.x = (avgX);
    cnmCenterLocation.y = (avgY);

    //send to searchController
    //---------------------------------------------
    searchController.setCenterLocation(cnmCenterLocation);
}

bool CNMCurrentLocationAVG()
{
    static int index = 0;

    if(index < ASIZE)
    {

	avgCurrentCoordsX[index] = currentLocationMap.x;
    	avgCurrentCoordsY[index] = currentLocationMap.y;

	index++;

	return false;
    }
    else
    {
	float x = 0, y = 0;
	for(int i = 0; i < ASIZE; i++)
	{
	    x += avgCurrentCoordsX[i];
	    y += avgCurrentCoordsY[i];
	}

	x = x/ASIZE;
	y = y/ASIZE;

	cnmAVGCurrentLocation.x = x;
	cnmAVGCurrentLocation.y = y;

	index = 0;
	return true;
    }
}

void CNMReactToCenterProtocol()
{

    static int countLocStored = 0;
    static bool gotEnoughPoints = false;


    //If trying to reverse and done a 180
    if(cnmReverse && cnmReverseDone) 
    { 
	//stop
        CNMReverseReset();
        goalLocation = currentLocation;        
    }

    //
    if(countLocStored > 4) { gotEnoughPoints = true; }	                

    CNMProjectCenter();

    countLocStored++;

    if(cnmCenteringFirstTime)
    {
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

    if(!targetCollected && gotEnoughPoints && !isDroppingOff)
    {
	//if we are not centered
        if(!centered)
	{
	    //keep trying
	    centered = CNMCentered();
	}
	//if we are finally centered
	else
	{
            //If we haven't seen the center before
            //---------------------------------------------
            if(!cnmLocatedCenterFirst) { CNMFirstSeenCenter(); }

            //If we have seen the center before
            //---------------------------------------------
            else if(cnmLocatedCenterFirst && cnmInitialPositioningComplete) { CNMRefindCenter(); }

	    //reset our variables
            countLocStored = 0;
            gotEnoughPoints = false;
            
	    //reverse       
            CNMReverseReset();
            CNMStartReversing();
	}
    }
}

//CNM PUB/SUB HANDLERS
//-----------------------------------

void dropProtocolHandler(const std_msgs::String& msg)		//dropOffProtocol
{
    waitToDrop = true;

    if(waitToDrop == true && isDroppingOff == true && targetCollected == true)	//allow to continue	//dropOffProtocol
    {
    	std_msgs::String msg;
        msg.data = "allowed to proceed";
        infoLogPublisher.publish(msg);
    }

    if(waitToDrop == true && isDroppingOff == false && targetCollected == true)  // stop movement here, resume when waitToDrop = false
    {
    	std_msgs::String msg;
        msg.data = "I should be waiting";
        infoLogPublisher.publish(msg);
    }

    if(waitToDrop == true && targetCollected == false) // allow to continue, but notified swarmie at center
    {
    	std_msgs::String msg;
        msg.data = "I don't care.";
        infoLogPublisher.publish(msg);
    }
}

void finishedProtocolHandler(const std_msgs::String& msg) //give instruction if have block, waitToDrop = true
{
    waitToDrop = false;

    std_msgs::String msgd;
        msgd.data = "waitToDrop turned to false";
        infoLogPublisher.publish(msgd);
}


//CNM TIMER FUNCTIONS
//-----------------------------------

//INITIAL TIMERS

void CNMInitPositioning(const ros::TimerEvent &event)
{
//    std_msgs::String msg;
//    msg.data = "Initial Wait Time Complete, Driving Forward";
//    infoLogPublisher.publish(msg);

    goalLocation.theta = currentLocation.theta;

    //select position 25 cm from the robots location before attempting to go into search pattern
    goalLocation.x = currentLocation.x + (.45 * cos(goalLocation.theta));
    goalLocation.y = currentLocation.y + (.45 * sin(goalLocation.theta));

    cnmHasWaitedInitialAmount = true;
}

void CNMForwardInitTimerDone(const ros::TimerEvent& event)
{

//    std_msgs::String msg;
//    msg.data = "Finished Driving; Turning 180";
//    infoLogPublisher.publish(msg);

    cnmHasMovedForward = true;

    //set NEW heading 180 degrees from current theta
    goalLocation.theta = currentLocationMap.theta + M_PI;  //was currentLocation

    //APPROX 45 cm away
    goalLocation.x = currentLocationMap.x + (.45 * cos(goalLocation.theta));  //was currentLocation
    goalLocation.y = currentLocationMap.y + (.45 * sin(goalLocation.theta));  //was currentLocation

    cnmForwardTimer.stop();
}

void CNMInitialWait(const ros::TimerEvent &e)
{

//    std_msgs::String msg;
//    msg.data = "Finished 180, Starting Search Pattern";
//    infoLogPublisher.publish(msg);

    cnmHasTurned180 = true;
    cnmInitialPositioningComplete = true;
    cnmFirstBootProtocol = false;

    stringstream ss;

    searchController.doAnotherOctagon();

    //Continue an interrupted search pattern
    //---------------------------------------------
    goalLocation = searchController.continueInterruptedSearch(currentLocation, goalLocation);

    //ROTATE!!!
    //---------------------------------------------
    stateMachineState = STATE_MACHINE_ROTATE;

    int position;
    double distance;
    int rotations;
    bool hasRotated = searchController.getHasDoneRotation();

    goalLocation = searchController.search(currentLocation);

    position = searchController.cnmGetSearchPosition();

    distance = searchController.cnmGetSearchDistance();

    rotations = searchController.cnmGetNumRotations();

    ss << "Traveling to point " << position << " at " << distance << " meters. " << " Rotations: " << rotations;

    if(hasRotated) { ss << " " << "TRUE"; }
    else { ss << " " << "FALSE"; }

    std_msgs::String msg;
    msg.data = ss.str();
    infoLogPublisher.publish(msg);

    cnmInitialWaitTimer.stop();
}

//OBSTACLE TIMERS

void CNMAvoidObstacle(const ros::TimerEvent &event)
{
    std_msgs::String msg;
    msg.data = "Obstacle Avoidance Initiated";
    infoLogPublisher.publish(msg);
    cnmAvoidObstacle = true;

    cnmAvoidObstacleTimer.stop();
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
//    std_msgs::String msg;
//    msg.data = "REVERSE TIMER DONE, STARTING TURN180";
//    infoLogPublisher.publish(msg);

    cnmTurn180Timer.start();

    //set NEW heading 180 degrees from current theta
    goalLocation.theta = currentLocation.theta + M_PI;

    double searchDist = searchController.cnmGetSearchDistance();

    //select position however far away we are currently searching from the robots location before attempting to go into search pattern
    goalLocation.x = currentLocation.x + ((searchDist / 2) * cos(goalLocation.theta));
    goalLocation.y = currentLocation.y + ((searchDist / 2) * sin(goalLocation.theta));

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

    CNMReverseReset();

    if(!cnmInitialPositioningComplete) { cnmInitialPositioningComplete = true; }

    searchController.doAnotherOctagon();

    //Continue an interrupted search pattern
    //---------------------------------------------
    goalLocation = searchController.continueInterruptedSearch(currentLocation, goalLocation);

    //ROTATE!!!
    //---------------------------------------------
    stateMachineState = STATE_MACHINE_ROTATE;

    //SPIT OUT NEXT POINT AND HOW FAR OUT WE ARE GOING
    //---------------------------------------------
    int position;
    double distance;
    int rotations;
    bool hasRotated = searchController.getHasDoneRotation();

    goalLocation = searchController.search(currentLocationMap);

    position = searchController.cnmGetSearchPosition();

    distance = searchController.cnmGetSearchDistance();

    rotations = searchController.cnmGetNumRotations();

    ss << "Traveling to point " << position << " at " << distance << " meters. " << " Rotations: " << rotations;

    if(hasRotated) { ss << " " << "TRUE"; }
    else { ss << " " << "FALSE"; }
 
    msg.data = ss.str();
    infoLogPublisher.publish(msg);

    // move back to transform step
    stateMachineState = STATE_MACHINE_TRANSFORM;

    cnmTurn180Timer.stop();
}

//CENTERING TIMERS

void CNMCenterTimerDone(const ros::TimerEvent& event)
{
    cnmCentering = false;
    centered = false;
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

    std_msgs::String msg;
    msg.data = "Finished Pick Up";
    infoLogPublisher.publish(msg);

    cnmFinishedPickUp = true;
    numTagsCarrying = numTargets + 1;
    //isDroppingOff = true;
    cnmAfterPickUpTimer.stop();
}

void CNMWaitBeforeDetectObst(const ros::TimerEvent &event)
{
    cnmStartObstDetect = true;
    cnmTimeBeforeObstDetect.stop();
}

void CNMWaitToCollectTags(const ros::TimerEvent &event)
{
    cnmCanCollectTags = true;

    if(isDroppingOff)
    {
	isDroppingOff = false;
    }

    cnmWaitToCollectTagsTimer.stop();
}

void CNMDropOffDrive(const ros::TimerEvent &event)
{

    readyToDrop = true;
    cnmDropOffDriveTimer.stop();

    if(seeMoreTargets)
    {
        std_msgs::String msg;
        msg.data = "See More Tags, dropping here!";
        infoLogPublisher.publish(msg);

        dropNow = true;
    }
}

void CNMDropReversed(const ros::TimerEvent &event)
{
    dropOffReset = true;
    cnmDropOffReverse.stop();
}

void CNMWaitToConfirm(const ros::TimerEvent& event)
{
    confirmedCenter = true;   
    cnmConfirmCentered.stop();
}
