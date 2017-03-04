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

    SearchController();

    //sets centerSeen bool
    void setCenterSeen(bool answer);

    void setCenterLocation(geometry_msgs::Pose2D newLocation);

    void cnmSetRotations(int num);

    int cnmGetSearchPosition();
    double cnmGetSearchDistance();

    // performs search pattern
    geometry_msgs::Pose2D search(geometry_msgs::Pose2D currentLocation);

    // continues search pattern after interruption
    //geometry_msgs::Pose2D continueInterruptedSearch(geometry_msgs::Pose2D currentLocation, geometry_msgs::Pose2D oldGoalLocation);
    geometry_msgs::Pose2D continueInterruptedSearch(geometry_msgs::Pose2D currentLocation, geometry_msgs::Pose2D oldGoalLocation);

    bool getCenterSeen(){ return cnmCenterSeen; }

  private:
    //VARIABLES
    //--------------------------------------
    random_numbers::RandomNumberGenerator* rng;

    //CNM VARIABLES
    //--------------------------------------
    geometry_msgs::Pose2D cnmCenterLocation;

    //BOOLEAN TRIGGERS
    //--------------------------------------
    bool cnmCenterSeen;
    bool cnmHasReset;

    //PRIMITIVES
    //--------------------------------------

    //Target Search
    int searchLoop;             //int of 0 - 8

    double searchDist;
    double searchCounter;
    double cnmSearchCounterDistance;

    int cnmNumRotations;

    //gets closest point to go to
    geometry_msgs::Pose2D cnmGetPositionInSearchPattern(geometry_msgs::Pose2D currentLocation);

    int RightSearch(double absLocationAngle);
    int LeftSearch(double absLocationAngle);

    //performs alternating left/right search pattern
    geometry_msgs::Pose2D SearchLeft(geometry_msgs::Pose2D currentLocation);
    geometry_msgs::Pose2D SearchRight(geometry_msgs::Pose2D currentLocation);
};

#endif /* SEARCH_CONTROLLER */
