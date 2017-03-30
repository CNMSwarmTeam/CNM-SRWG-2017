#include <ros/ros.h>

//ROS libraries
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

//ROS messages
#include <std_msgs/UInt8.h>
#include <sensor_msgs/Range.h>

using namespace std;

//Globals
double collisionDistance = 0.6; //meters the ultrasonic detectors will flag obstacles
string publishedName;
char host[128];


ros::Duration cnm1SecTime(0.25);
ros::Timer obstacleTimer;
void IsAnObstacle(const ros::TimerEvent& event);

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
    
    obstacleTimer = oNH.createTimer(cnm1SecTime, IsAnObstacle, true);
    obstacleTimer.stop();
    
    message_filters::Subscriber<sensor_msgs::Range> sonarLeftSubscriber(oNH, (publishedName + "/sonarLeft"), 10);
    message_filters::Subscriber<sensor_msgs::Range> sonarCenterSubscriber(oNH, (publishedName + "/sonarCenter"), 10);
    message_filters::Subscriber<sensor_msgs::Range> sonarRightSubscriber(oNH, (publishedName + "/sonarRight"), 10);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Range, sensor_msgs::Range, sensor_msgs::Range> sonarSyncPolicy;

    message_filters::Synchronizer<sonarSyncPolicy> sonarSync(sonarSyncPolicy(10), sonarLeftSubscriber, sonarCenterSubscriber, sonarRightSubscriber);
    sonarSync.registerCallback(boost::bind(&sonarHandler, _1, _2, _3));

    ros::spin();

    return EXIT_SUCCESS;
}

void sonarHandler(const sensor_msgs::Range::ConstPtr& sonarLeft, const sensor_msgs::Range::ConstPtr& sonarCenter, const sensor_msgs::Range::ConstPtr& sonarRight) {
	std_msgs::UInt8 obstacleMode;

//1 Obstacle right
//2 Obstacle left/center
//3 Obstacle Too Close
//4 blockBlock

	static bool first = true;

	if(first)
	{
	    left_0 = sonarLeft->range;
	    center_0 = sonarCenter->range;
	    right_0 = sonarRight->range;
	    first = false;
	}

	if ((sonarLeft->range > collisionDistance) && (sonarCenter->range > collisionDistance) && (sonarRight->range > collisionDistance)) 		{
		obstacleMode.data = 0; //no collision
		obstacleTimer.stop();
		timerDone = false;
	}
	else
	{
	left_1 = left_0; //left sonar reading a moment ago
	center_1 = center_0;
	right_1 = right_0;

	left_0 = sonarLeft->range; //left sonar reading now
	center_0 = sonarCenter->range;
	right_0 = sonarRight->range;

	if((left_1 - left_0) > 2)
	{
	    falseJump = true;
	}
	else if((right_1 - right_0) > 2)
	{
	    falseJump = true;
	}
	else if((center_1 - center_0) > 2)
	{
	    falseJump = true;
	}
	else
	{
	    falseJump = false;
	} 

	if(!falseJump)
	{

	if ((sonarLeft->range > collisionDistance) && (sonarRight->range < collisionDistance)) 
	{
		if(0.45 > sonarRight->range > 0.25)
		{
		    obstacleMode.data = 3;
		    timerDone = true;
		    obstacleTimer.stop();
		}		
		else if(timerDone) 
		{
		    obstacleMode.data = 1; //collision on right side
		}
		else
		{
		    obstacleTimer.start();
		    obstacleMode.data = 0;
		}
	}
	else
	{
		if(0.45 > sonarLeft->range > 0.25)
		{
		    obstacleMode.data = 3;
		    timerDone = true;
		    obstacleTimer.stop();
		}		
		else if(timerDone)  
		{
		    obstacleMode.data = 2; //collision in front or on left side
		}
		else
		{
		    obstacleTimer.start();
		    obstacleMode.data = 0;
		}
	}

	if (sonarCenter->range < 0.12) //block in front of center unltrasound.
	{
		obstacleMode.data = 4;
	}
	}
	}
        obstaclePublish.publish(obstacleMode);

	falseJump = false;
}

void IsAnObstacle(const ros::TimerEvent& event)
{

    timerDone = true;
    obstacleTimer.stop();

}
