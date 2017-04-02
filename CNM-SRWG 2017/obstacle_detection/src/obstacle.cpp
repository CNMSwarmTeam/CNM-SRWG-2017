#include <ros/ros.h>

//ROS libraries
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

//ROS messages
#include <std_msgs/UInt8.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/String.h>

using namespace std;

//Globals
double collisionDistance = .5; //meters the ultrasonic detectors will flag obstacles
string publishedName;
char host[128];


//Timers
ros::Duration cnm1SecTime(0.20);
ros::Timer obstacleTimer;
ros::Timer publish_heartbeat_timer;


void IsAnObstacle(const ros::TimerEvent& event);


float heartbeat_publish_interval = 2;

//Publishers
ros::Publisher heartbeatPublisher;

bool leftObst = false;
bool rcObst = false;
bool blockObst = false;
bool timerDone = false;

float left_0 = 0, center_0 = 0, right_0 = 0; //PW sonar distance readings now
float left_1 = 0, center_1 = 0, right_1 = 0; //PW sonar distance readings one inverval back in time
bool falseJump = false;

//Publishers
ros::Publisher obstaclePublish;

//Callback handlers
void sonarHandler(const sensor_msgs::Range::ConstPtr& sonarLeft, const sensor_msgs::Range::ConstPtr& sonarCenter, const sensor_msgs::Range::ConstPtr& sonarRight);

void publishHeartBeatTimerEventHandler(const ros::TimerEvent& event);

int main(int argc, char** argv) {
    gethostname(host, sizeof (host));
    string hostname(host);
    
    if (argc >= 2) {
        publishedName = argv[1];
        cout << "Welcome to the world of tomorrow " << publishedName << "! Obstacle module started." << endl;
    } else {
        publishedName = hostname;
        cout << "No name selected. Default is: " << publishedName << endl;
    }

    ros::init(argc, argv, (publishedName + "_OBSTACLE"));
    ros::NodeHandle oNH;
    
    obstaclePublish = oNH.advertise<std_msgs::UInt8>((publishedName + "/obstacle"), 10);
    heartbeatPublisher = oNH.advertise<std_msgs::String>((publishedName + "/obstacle/heartbeat"), 1, true);
    
    message_filters::Subscriber<sensor_msgs::Range> sonarLeftSubscriber(oNH, (publishedName + "/sonarLeft"), 10);
    message_filters::Subscriber<sensor_msgs::Range> sonarCenterSubscriber(oNH, (publishedName + "/sonarCenter"), 10);
    message_filters::Subscriber<sensor_msgs::Range> sonarRightSubscriber(oNH, (publishedName + "/sonarRight"), 10);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Range, sensor_msgs::Range, sensor_msgs::Range> sonarSyncPolicy;

    message_filters::Synchronizer<sonarSyncPolicy> sonarSync(sonarSyncPolicy(10), sonarLeftSubscriber, sonarCenterSubscriber, sonarRightSubscriber);
    sonarSync.registerCallback(boost::bind(&sonarHandler, _1, _2, _3));

    publish_heartbeat_timer = oNH.createTimer(ros::Duration(heartbeat_publish_interval), publishHeartBeatTimerEventHandler);

    obstacleTimer = oNH.createTimer(cnm1SecTime, IsAnObstacle, true);
    obstacleTimer.stop();


    ros::spin();

    return EXIT_SUCCESS;
}

void sonarHandler(const sensor_msgs::Range::ConstPtr& sonarLeft, const sensor_msgs::Range::ConstPtr& sonarCenter, const sensor_msgs::Range::ConstPtr& sonarRight) {
	std_msgs::UInt8 obstacleMode;

//1 Obstacle right
//2 Obstacle left/center
//3 Obstacle Too Close
//4 blockBlock

	if ((sonarLeft->range > collisionDistance) && (sonarCenter->range > collisionDistance) && (sonarRight->range > collisionDistance)) 		{
		obstacleMode.data = 0; //no collision
	}
	else if ((sonarLeft->range > collisionDistance) && (sonarRight->range < collisionDistance)) 
	{
            if(0.40 > sonarRight->range >= 0.13)
	    {
		obstacleMode.data = 3;
	    }
	    else		
	    {
 	        obstacleMode.data = 1; //collision on right side
	    }
	}
	else
	{
	    if(0.40 > sonarLeft->range >= 0.13)
	    {
	        obstacleMode.data = 3;
	    }		
	    else  
	    {
	        obstacleMode.data = 2; //collision in front or on left side
	    }
	}

	if (sonarCenter->range < 0.12) //block in front of center unltrasound.
	{
		obstacleMode.data = 4;
	}

        obstaclePublish.publish(obstacleMode);
}

void IsAnObstacle(const ros::TimerEvent& event)
{

    timerDone = true;
    obstacleTimer.stop();

}

void publishHeartBeatTimerEventHandler(const ros::TimerEvent&) {
    std_msgs::String msg;
    msg.data = "";
    heartbeatPublisher.publish(msg);
     ROS_INFO("yes");
}
