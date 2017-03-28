#ifndef SEARCH_CONTROLLER
#define SEARCH_CONTROLLER

#include <geometry_msgs/Pose2D.h>
#include <random_numbers/random_numbers.h>
#include <angles/angles.h>

/**
 * This class implements the search control algorithm for the rovers. The code
 * here should be modified and enhanced to improve search performance.
 */

class SearchController
{

  public:

    //METHODS
    //--------------------------------------

    //CONSTRUCTOR
    SearchController();

    //GETS
    bool getCenterSeen(){ return cnmCenterSeen; }

    bool getHasDoneRotation() { return doneOneFullRotation; }

    bool cnmIsAlternating() { return reverseSearch; }

    int cnmGetSearchPosition();

    int cnmGetNumRotations() { return cnmNumRotations; }

    double cnmGetSearchDistance();

    //SETS
    bool doAnotherOctagon() { doneOneFullRotation = false; cnmNumRotations = 0;}

    void AmILost(bool answer);

    void setCenterSeen(bool answer);							//sets centerSeen bool

    void setCenterLocation(geometry_msgs::Pose2D newLocation);

    void obstacleWasAvoided(){ avoidedObstacle = true; numTimesAvoidedObst++; }

    void setSearchDist(int id) { cnmSearchCounterDistance = 1.25 + (id * 0.45); }

    //SEARCH ALOGIRTHM CALLS
    geometry_msgs::Pose2D search(geometry_msgs::Pose2D currentLocation);		// performs search pattern
    geometry_msgs::Pose2D continueInterruptedSearch(geometry_msgs::Pose2D currentLocation, geometry_msgs::Pose2D oldGoalLocation);	// continues search pattern after interruption

  private:

    //PRIMITIVES
    //--------------------------------------

    //Target Search
    int searchLoop;             	//int of 0 - 9 for search positions
    int minNumRotations;		//MINIMUM number of rotations before we consider it having done enough of an octagon to increment
    int cnmNumRotations;		//Number of times the rover has gone to next point before hitting point 0
    int numTimesAvoidedObst;		//Number of times rover had to avoid an obstacle before eventually moving on to a new point

    double searchDist;			//Distance to add to search distance when incrementing (added to searchCounter)
    double searchCounter;		//Distance to search from center
    double cnmSearchCounterDistance;	//Stores ORIGINAL search distance value


    //BOOLEAN TRIGGERS
    bool cnmCenterSeen;			//If we have seen the center yet
    bool avoidedObstacle;		//Triggered true by obstacleWasAvoided

    bool doneFirstRotation;		//If we have finished our first rotation.  Implemented so we always complete ONE FULL LOOP
    bool doneOneFullRotation;		//If we have managed to finish another full rotation

    bool reverseSearch;			//If we are going to alternate from left and right

    //OTHER VARIABLES
    //--------------------------------------

    random_numbers::RandomNumberGenerator* rng;
    geometry_msgs::Pose2D cnmCenterLocation;

    //METHODS
    //--------------------------------------

    void RightSearch(double absLocationAngle);
    void LeftSearch(double absLocationAngle);\

    void checkIfIncrementing();


    //performs alternating left/right search pattern
    geometry_msgs::Pose2D SearchLeft(geometry_msgs::Pose2D currentLocation);
    geometry_msgs::Pose2D SearchRight(geometry_msgs::Pose2D currentLocation);

    //gets closest point to go to
    geometry_msgs::Pose2D cnmGetPositionInSearchPattern(geometry_msgs::Pose2D currentLocation);

};

#endif /* SEARCH_CONTROLLER */
