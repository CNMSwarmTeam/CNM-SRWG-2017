#include "SearchController.h"

//CONSTANTS
//--------------------------------------
double const MIN_DIST_SEARCH = .7;
double const MAX_DIST_SEARCH = 2.0;

bool hasDoneFirstRotation;

int numTimesAvoidedObst = 0;

SearchController::SearchController() 
{    
    rng = new random_numbers::RandomNumberGenerator();

    //CNM VARIABLES
    //--------------------------------------
    searchLoop = 0;//rng->uniformInteger(0, 8);                 //DEFAULT TO 0, uncomment for random point in loop between 0 and 8
    searchCounter = .5;                                         //default value for search distance is .5
    searchDist = .2;                                            //how much to add to our search pattern (.2 is roughly the width of a swarmie)

    //random distance from center to start searching
    cnmSearchCounterDistance = 1;//rng->uniformReal(MIN_DIST_SEARCH, MAX_DIST_SEARCH);

    cnmNumRotations = 0;

    cnmCenterSeen = false;                                      //we start off never seeing the center, these are set to false to reflect that
    cnmHasReset = true;                                         //initialized to true for first boot. Once running, if the the robot loses center,
                                                                //should be triggered back to true
    cnmCenterLocation.theta = 0;
    cnmCenterLocation.x = 0;                                    //set default center to (0,0)
    cnmCenterLocation.y = 0;

    hasDoneFirstRotation = false;
    reverseSearch = false;
    avoidedObstacle = false;
}

geometry_msgs::Pose2D SearchController::search(geometry_msgs::Pose2D currentLocation)
{

    if(avoidedObstacle)
    {
        //if we have called obst avoidance 4+ times in a row
        if(numTimesAvoidedObst >= 4)
        {
            //increment search loop to next point
            searchLoop++;
        }
    }
    else
    {
        numTimesAvoidedObst = 0;
    }

    geometry_msgs::Pose2D goalLocation;

    int dist = floor(searchCounter);

    if(dist % 2 == 0 && dist > 2)
    {
        reverseSearch = true;
    }
    else
    {
        reverseSearch = false;
    }

    //Check to see if we are alternating directions
    //---------------------------------------------
    if(reverseSearch)
    {
        goalLocation = SearchRight(currentLocation);
    }
    else
    {
        goalLocation = SearchLeft(currentLocation);
    }

    //RESET variable
    avoidedObstacle = false;

    return goalLocation;
}

geometry_msgs::Pose2D SearchController::SearchRight(geometry_msgs::Pose2D currentLocation)
{

    geometry_msgs::Pose2D goalLocation;

      //if we found the center and have had to reset our center location/found the center for the first time:
          //this bool will be set to true only if search controller is passed a false to cnmCenterSeen, once it runs
          //search again, and if it has seen the center, this will start us off with the original RNG distance
          //to start searching from center location.  (Allows us to avoid preset magic numbers)
      //---------------------------------------------
      if(cnmCenterSeen && cnmHasReset)
      {
          //reset search counter
          searchCounter = cnmSearchCounterDistance;              //random distance from center to start searching
          cnmHasReset = false;
          hasDoneFirstRotation = false;
      }

      //if for some reason searchLoop goes out of bounds, reset
      //---------------------------------------------
      if (searchLoop < 0 || searchLoop > 9)
      {
              searchLoop = 0;
      }

      //This algorithm uses the trigonometic coordinates for the unit circle to navigate around a central point
      //using the center location as the point where we pivot around, as opposed to the current location, allows
      //us to be more precise

      //between 11PI/6 and 2PI
      //---------------------------------------------
      if (searchLoop == 0)
      {

              if(hasDoneFirstRotation == false)
              {
                  hasDoneFirstRotation = true;
              }
              else
              {
                  searchCounter = searchCounter + searchDist;         //Increment at 0.2 for best results
              }

              if(!avoidedObstacle) { searchLoop++; }

              goalLocation.x = cnmCenterLocation.x + searchCounter;
              goalLocation.y = cnmCenterLocation.y + searchCounter / 2;
              goalLocation.theta = atan2((goalLocation.y - currentLocation.y), (goalLocation.x - currentLocation.x));
  /*
              goalLocation.theta = (15 * M_PI)/8;
              goalLocation.x = (cnmCenterLocation.x + searchCounter) * cos(goalLocation.theta);
              goalLocation.y = (cnmCenterLocation.y + searchCounter) * sin(goalLocation.theta);
  */
        }
        else if (searchLoop == 9)
        {

          if(!avoidedObstacle) { searchLoop = 0; }

          goalLocation.x = cnmCenterLocation.x + searchCounter;
          goalLocation.y = cnmCenterLocation.y - searchCounter / 2;
          goalLocation.theta = atan2((goalLocation.y - currentLocation.y), (goalLocation.x - currentLocation.x));
        /*
          goalLocation.theta = (15 * M_PI)/8;
          goalLocation.x = (cnmCenterLocation.x + searchCounter) * cos(goalLocation.theta);
         goalLocation.y = (cnmCenterLocation.y + searchCounter) * sin(goalLocation.theta);
        */
          //cnmOriginalSearchDistance = searchCounter;

          }
          //between 0 and PI/6
          //---------------------------------------------
          else if (searchLoop == 8)
          {
              if(!avoidedObstacle) { searchLoop++; }

              goalLocation.x = cnmCenterLocation.x + searchCounter;
              goalLocation.y = cnmCenterLocation.y + searchCounter / 2;
              goalLocation.theta = atan2((goalLocation.y - currentLocation.y), (goalLocation.x - currentLocation.x));
  /*
              goalLocation.theta = M_PI/8;
              goalLocation.x = (cnmCenterLocation.x + searchCounter) * cos(goalLocation.theta);
              goalLocation.y = (cnmCenterLocation.y + searchCounter) * sin(goalLocation.theta);
  */
          }
          //between 2PI/6 and PI/2
          //---------------------------------------------
          else if (searchLoop == 7)
          {
              if(!avoidedObstacle) { searchLoop++; }

              goalLocation.x = cnmCenterLocation.x + searchCounter / 2;
              goalLocation.y = cnmCenterLocation.y + searchCounter;
              goalLocation.theta = atan2((goalLocation.y - currentLocation.y), (goalLocation.x - currentLocation.x));
  /*
              goalLocation.theta = (3 * M_PI)/8;
              goalLocation.x = (cnmCenterLocation.x + searchCounter) * cos(goalLocation.theta);
              goalLocation.y = (cnmCenterLocation.y + searchCounter) * sin(goalLocation.theta);
  */
          }
          //between PI/2 and 4PI/6
          //---------------------------------------------
          else if (searchLoop == 6)
          {
              if(!avoidedObstacle) { searchLoop++; }

              goalLocation.x = cnmCenterLocation.x - searchCounter / 2;
              goalLocation.y = cnmCenterLocation.y + searchCounter;
              goalLocation.theta = atan2((goalLocation.y - currentLocation.y), (goalLocation.x - currentLocation.x));
  /*
              goalLocation.theta = (5 * M_PI)/8;
              goalLocation.x = (cnmCenterLocation.x + searchCounter) * cos(goalLocation.theta);
              goalLocation.y = (cnmCenterLocation.y + searchCounter) * sin(goalLocation.theta);
  */
          }
          //between 5PI/6 and PI
          //---------------------------------------------
          else if (searchLoop == 5)
          {
              if(!avoidedObstacle) { searchLoop++; }

              goalLocation.x = cnmCenterLocation.x - searchCounter;
              goalLocation.y = cnmCenterLocation.y + searchCounter / 2;
              goalLocation.theta = atan2((goalLocation.y - currentLocation.y), (goalLocation.x - currentLocation.x));
  /*
              goalLocation.theta = (7 * M_PI)/8;
              goalLocation.x = (cnmCenterLocation.x + searchCounter) * cos(goalLocation.theta);
              goalLocation.y = (cnmCenterLocation.y + searchCounter) * sin(goalLocation.theta);
  */
          }
          //between PI and 7PI/6
          //---------------------------------------------
          else if (searchLoop == 4)
          {
              if(!avoidedObstacle) { searchLoop++; }

              goalLocation.x = cnmCenterLocation.x - searchCounter;
              goalLocation.y = cnmCenterLocation.y - searchCounter / 2;
              goalLocation.theta = atan2((goalLocation.y - currentLocation.y), (goalLocation.x - currentLocation.x));
  /*
              goalLocation.theta = (9 * M_PI)/8;
              goalLocation.x = (cnmCenterLocation.x + searchCounter) * cos(goalLocation.theta);
              goalLocation.y = (cnmCenterLocation.y + searchCounter) * sin(goalLocation.theta);
  */
          }
          //between 7PI/6 and 8PI/6
          //---------------------------------------------
          else if (searchLoop == 3)
          {
              if(!avoidedObstacle) { searchLoop++; }

              goalLocation.x = cnmCenterLocation.x - searchCounter / 2;
              goalLocation.y = cnmCenterLocation.y - searchCounter;
              goalLocation.theta = atan2((goalLocation.y - currentLocation.y), (goalLocation.x - currentLocation.x));
  /*
              goalLocation.theta = (11 * M_PI)/8;
              goalLocation.x = (cnmCenterLocation.x + searchCounter) * cos(goalLocation.theta);
              goalLocation.y = (cnmCenterLocation.y + searchCounter) * sin(goalLocation.theta);
  */
          }
          //between 9PI/6 and 10PI/6
          //---------------------------------------------
          else if (searchLoop == 2)
          {
              if(!avoidedObstacle) { searchLoop++; }

              goalLocation.x = cnmCenterLocation.x + searchCounter / 2;
              goalLocation.y = cnmCenterLocation.y - searchCounter;
              goalLocation.theta = atan2((goalLocation.y - currentLocation.y), (goalLocation.x - currentLocation.x));
  /*
              goalLocation.theta = (13 * M_PI)/8;
              goalLocation.x = (cnmCenterLocation.x + searchCounter) * cos(goalLocation.theta);
              goalLocation.y = (cnmCenterLocation.y + searchCounter) * sin(goalLocation.theta);
  */
          }
          //between 10PI/6 and 11PI/6
          //---------------------------------------------
          else if (searchLoop == 1)
          {
              if(!avoidedObstacle) { searchLoop++; }

              goalLocation.x = cnmCenterLocation.x + searchCounter;
              goalLocation.y = cnmCenterLocation.y - searchCounter / 2;
              goalLocation.theta = atan2((goalLocation.y - currentLocation.y), (goalLocation.x - currentLocation.x));
  /*
          goalLocation.theta = (15 * M_PI)/8;
          goalLocation.x = (cnmCenterLocation.x + searchCounter) * cos(goalLocation.theta);
          goalLocation.y = (cnmCenterLocation.y + searchCounter) * sin(goalLocation.theta);
  */
              //cnmOriginalSearchDistance = searchCounter;

          }

      return goalLocation;

}

geometry_msgs::Pose2D SearchController::SearchLeft(geometry_msgs::Pose2D currentLocation)
{
    geometry_msgs::Pose2D goalLocation;

    goalLocation.theta = 0;

      //if we found the center and have had to reset our center location/found the center for the first time:
          //this bool will be set to true only if search controller is passed a false to cnmCenterSeen, once it runs
          //search again, and if it has seen the center, this will start us off with the original RNG distance
          //to start searching from center location.  (Allows us to avoid preset magic numbers)
      //---------------------------------------------
      if(cnmCenterSeen && cnmHasReset)
      {
          //reset search counter
          searchCounter = cnmSearchCounterDistance;              //random distance from center to start searching
          cnmHasReset = false;
          hasDoneFirstRotation = false;
      }

      //if for some reason searchLoop goes out of bounds, reset
      //---------------------------------------------
      if (searchLoop < 0 || searchLoop > 9)
      {
              searchLoop = 0;
      }

      //This algorithm uses the trigonometic coordinates for the unit circle to navigate around a central point
      //using the center location as the point where we pivot around, as opposed to the current location, allows
      //us to be more precise

      //between 11PI/6 and 2PI
      //---------------------------------------------
      if (searchLoop == 0)
      {
          if(!avoidedObstacle) { searchLoop++; }

          if(hasDoneFirstRotation == false)
          {
              hasDoneFirstRotation = true;
          }
          else
          {
              searchCounter = searchCounter + searchDist;         //Increment at 0.2 for best results
          }

          goalLocation.x = cnmCenterLocation.x + searchCounter;
          goalLocation.y = cnmCenterLocation.y + searchCounter / 2;
          goalLocation.theta = atan2((goalLocation.y - currentLocation.y), (goalLocation.x - currentLocation.x));
  /*
              goalLocation.theta = (15 * M_PI)/8;
              goalLocation.x = (cnmCenterLocation.x + searchCounter) * cos(goalLocation.theta);
              goalLocation.y = (cnmCenterLocation.y + searchCounter) * sin(goalLocation.theta);
  */

          }

          //between 0 and PI/6
          //---------------------------------------------
          else if (searchLoop == 1)
          {
              if(!avoidedObstacle) { searchLoop++; }

              goalLocation.x = cnmCenterLocation.x + searchCounter;
              goalLocation.y = cnmCenterLocation.y + searchCounter / 2;
              goalLocation.theta = atan2((goalLocation.y - currentLocation.y), (goalLocation.x - currentLocation.x));
  /*
              goalLocation.theta = M_PI/8;
              goalLocation.x = (cnmCenterLocation.x + searchCounter) * cos(goalLocation.theta);
              goalLocation.y = (cnmCenterLocation.y + searchCounter) * sin(goalLocation.theta);
  */

          }
          //between 2PI/6 and PI/2
          //---------------------------------------------
          else if (searchLoop == 2)
          {
              if(!avoidedObstacle) { searchLoop++; }

              goalLocation.x = cnmCenterLocation.x + searchCounter / 2;
              goalLocation.y = cnmCenterLocation.y + searchCounter;
              goalLocation.theta = atan2((goalLocation.y - currentLocation.y), (goalLocation.x - currentLocation.x));
  /*
              goalLocation.theta = (3 * M_PI)/8;
              goalLocation.x = (cnmCenterLocation.x + searchCounter) * cos(goalLocation.theta);
              goalLocation.y = (cnmCenterLocation.y + searchCounter) * sin(goalLocation.theta);
  */
          }
          //between PI/2 and 4PI/6
          //---------------------------------------------
          else if (searchLoop == 3)
          {
              if(!avoidedObstacle) { searchLoop++; }

              goalLocation.x = cnmCenterLocation.x - searchCounter / 2;
              goalLocation.y = cnmCenterLocation.y + searchCounter;
              goalLocation.theta = atan2((goalLocation.y - currentLocation.y), (goalLocation.x - currentLocation.x));
  /*
              goalLocation.theta = (5 * M_PI)/8;
              goalLocation.x = (cnmCenterLocation.x + searchCounter) * cos(goalLocation.theta);
              goalLocation.y = (cnmCenterLocation.y + searchCounter) * sin(goalLocation.theta);
  */
          }
          //between 5PI/6 and PI
          //---------------------------------------------
          else if (searchLoop == 4)
          {
              if(!avoidedObstacle) { searchLoop++; }

              goalLocation.x = cnmCenterLocation.x - searchCounter;
              goalLocation.y = cnmCenterLocation.y + searchCounter / 2;
              goalLocation.theta = atan2((goalLocation.y - currentLocation.y), (goalLocation.x - currentLocation.x));
  /*
              goalLocation.theta = (7 * M_PI)/8;
              goalLocation.x = (cnmCenterLocation.x + searchCounter) * cos(goalLocation.theta);
              goalLocation.y = (cnmCenterLocation.y + searchCounter) * sin(goalLocation.theta);
  */
          }
          //between PI and 7PI/6
          //---------------------------------------------
          else if (searchLoop == 5)
          {
              if(!avoidedObstacle) { searchLoop++; }

              goalLocation.x = cnmCenterLocation.x - searchCounter;
              goalLocation.y = cnmCenterLocation.y - searchCounter / 2;
              goalLocation.theta = atan2((goalLocation.y - currentLocation.y), (goalLocation.x - currentLocation.x));
  /*
              goalLocation.theta = (9 * M_PI)/8;
              goalLocation.x = (cnmCenterLocation.x + searchCounter) * cos(goalLocation.theta);
              goalLocation.y = (cnmCenterLocation.y + searchCounter) * sin(goalLocation.theta);
  */
          }
          //between 7PI/6 and 8PI/6
          //---------------------------------------------
          else if (searchLoop == 6)
          {
              if(!avoidedObstacle) { searchLoop++; }

              goalLocation.x = cnmCenterLocation.x - searchCounter / 2;
              goalLocation.y = cnmCenterLocation.y - searchCounter;
              goalLocation.theta = atan2((goalLocation.y - currentLocation.y), (goalLocation.x - currentLocation.x));
  /*
              goalLocation.theta = (11 * M_PI)/8;
              goalLocation.x = (cnmCenterLocation.x + searchCounter) * cos(goalLocation.theta);
              goalLocation.y = (cnmCenterLocation.y + searchCounter) * sin(goalLocation.theta);
  */
          }
          //between 9PI/6 and 10PI/6
          //---------------------------------------------
          else if (searchLoop == 7)
          {
              if(!avoidedObstacle) { searchLoop++; }

              goalLocation.x = cnmCenterLocation.x + searchCounter / 2;
              goalLocation.y = cnmCenterLocation.y - searchCounter;
              goalLocation.theta = atan2((goalLocation.y - currentLocation.y), (goalLocation.x - currentLocation.x));
  /*
              goalLocation.theta = (13 * M_PI)/8;
              goalLocation.x = (cnmCenterLocation.x + searchCounter) * cos(goalLocation.theta);
              goalLocation.y = (cnmCenterLocation.y + searchCounter) * sin(goalLocation.theta);
  */
          }
          //between 10PI/6 and 11PI/6
          //---------------------------------------------
          else if (searchLoop == 8)
          {

            if(!avoidedObstacle) { searchLoop++; }

            goalLocation.x = cnmCenterLocation.x + searchCounter;
            goalLocation.y = cnmCenterLocation.y - searchCounter / 2;
            goalLocation.theta = atan2((goalLocation.y - currentLocation.y), (goalLocation.x - currentLocation.x));
  /*
          goalLocation.theta = (15 * M_PI)/8;
          goalLocation.x = (cnmCenterLocation.x + searchCounter) * cos(goalLocation.theta);
          goalLocation.y = (cnmCenterLocation.y + searchCounter) * sin(goalLocation.theta);
  */
          }

      //between 0 and PI/6
      //---------------------------------------------
      else if (searchLoop == 9)
      {
          if(!avoidedObstacle) { searchLoop = 0; }

          goalLocation.x = cnmCenterLocation.x + searchCounter;
          goalLocation.y = cnmCenterLocation.y + searchCounter / 2;
          goalLocation.theta = atan2((goalLocation.y - currentLocation.y), (goalLocation.x - currentLocation.x));
/*
          goalLocation.theta = M_PI/8;
          goalLocation.x = (cnmCenterLocation.x + searchCounter) * cos(goalLocation.theta);
          goalLocation.y = (cnmCenterLocation.y + searchCounter) * sin(goalLocation.theta);
*/

      }

      return goalLocation;
}

/**
 * Continues search pattern after interruption. For example, avoiding the
 * center or collisions.
 */
geometry_msgs::Pose2D SearchController::continueInterruptedSearch(geometry_msgs::Pose2D currentLocation, geometry_msgs::Pose2D oldGoalLocation)
{   
    //absLocationAngle adjusts the incoming angle
    //---------------------------------------------
    double absLocationAngle;

    geometry_msgs::Pose2D newGoalLocation;

    absLocationAngle = angles::normalize_angle_positive(currentLocation.theta);

    //Check to see if we are alternating directions
    //---------------------------------------------
    if(reverseSearch)
    {
        RightSearch(absLocationAngle);
    }
    else
    {
        LeftSearch(absLocationAngle);
    }

    //Pass to search
    //---------------------------------------------
    newGoalLocation = search(currentLocation);

    return newGoalLocation;
}

int SearchController::LeftSearch(double absLocationAngle)
{
    if (absLocationAngle <= angles::from_degrees(30))
    {
        searchLoop = 1;
    }
    else if (absLocationAngle <= angles::from_degrees(60))
    {
        searchLoop = 1;
    }
    else if (absLocationAngle <= angles::from_degrees(90))
    {
        searchLoop = 2;
    }
    else if (absLocationAngle <= angles::from_degrees(120))
    {
        searchLoop = 3;
    }
    else if (absLocationAngle <= angles::from_degrees(150))
    {
        searchLoop = 4;
    }
    else if (absLocationAngle <= angles::from_degrees(180))
    {
        searchLoop = 4;
    }
    else if (absLocationAngle <= angles::from_degrees(210))
    {
        searchLoop = 4;
    }
    else if (absLocationAngle <= angles::from_degrees(240))
    {
        searchLoop = 5;
    }
    else if (absLocationAngle <= angles::from_degrees(270))
    {
        searchLoop = 6;
    }
    else if (absLocationAngle <= angles::from_degrees(300))
    {
        searchLoop = 6;
    }
    else if (absLocationAngle <= angles::from_degrees(330))
    {
        searchLoop = 7;
    }
    else if (absLocationAngle <= angles::from_degrees(360))
    {
        searchLoop = 8;
    }
}

int SearchController::RightSearch(double absLocationAngle)
{
    if (absLocationAngle <= angles::from_degrees(30))
    {
        searchLoop = 8;
    }
    else if (absLocationAngle <= angles::from_degrees(60))
    {
        searchLoop = 8;
    }
    else if (absLocationAngle <= angles::from_degrees(90))
    {
        searchLoop = 7;
    }
    else if (absLocationAngle <= angles::from_degrees(120))
    {
        searchLoop = 6;
    }
    else if (absLocationAngle <= angles::from_degrees(150))
    {
        searchLoop = 5;
    }
    else if (absLocationAngle <= angles::from_degrees(180))
    {
        searchLoop = 5;
    }
    else if (absLocationAngle <= angles::from_degrees(210))
    {
        searchLoop = 5;
    }
    else if (absLocationAngle <= angles::from_degrees(240))
    {
        searchLoop = 4;
    }
    else if (absLocationAngle <= angles::from_degrees(270))
    {
        searchLoop = 3;
    }
    else if (absLocationAngle <= angles::from_degrees(300))
    {
        searchLoop = 2;
    }
    else if (absLocationAngle <= angles::from_degrees(330))
    {
        searchLoop = 2;
    }
    else if (absLocationAngle <= angles::from_degrees(360))
    {
        searchLoop = 1;
    }
}

void SearchController::setCenterSeen(bool answer)
{
    cnmCenterSeen = answer;

    if(!cnmCenterSeen)
    {
        cnmHasReset = true;
        searchCounter = 0.5;
    }
}

void SearchController::setCenterLocation(geometry_msgs::Pose2D newLocation)
{
    cnmCenterLocation = newLocation;
}

void SearchController::cnmSetRotations(int num)
{
    cnmNumRotations = num;
}

int SearchController::cnmGetSearchPosition()
{
    if(searchLoop == 0)
    {
        return 8;
    }
    else
    {
        return searchLoop - 1;
    }
}

double SearchController::cnmGetSearchDistance()
{
    return searchCounter;
}

void SearchController::obstacleWasAvoided()
{
    avoidedObstacle = true;
    numTimesAvoidedObst++;
}


//OLD RANDOM WALK
//remainingGoalDist avoids magic numbers by calculating the dist
// double remainingGoalDist = hypot(oldGoalLocation.x - currentLocation.x, oldGoalLocation.y - currentLocation.y);

//this of course assumes random walk continuation. Change for diffrent search methods.
//newGoalLocation.theta = oldGoalLocation.theta;
//newGoalLocation.x = currentLocation.x + (0.20 * cos(oldGoalLocation.theta)); //(remainingGoalDist * cos(oldGoalLocation.theta));
//newGoalLocation.y = currentLocation.y + (0.20 * sin(oldGoalLocation.theta)); //(remainingGoalDist * sin(oldGoalLocation.theta));


//OLD CODE FOR FUTURE REFERENCE MATERIAL:
    //DO NOT UNCOMMENT!!!!
//---------------------------------------------

/*
void SearchController::CNMCenterSearch(geometry_msgs::Pose2D currentLocation, geometry_msgs::Pose2D goalLocation)
{
    if (centerLoop < 0 || centerLoop > 8)
    {
        centerLoop = 8;
    }

    if(centerLoop == 0) //octigonal search pattern
    {
        centerLoop = 8; //
        goalLocation.x = goalLocation.x + centerSearch / 2; //drive east
        goalLocation.y = goalLocation.y - centerSearch;
        goalLocation.theta = atan2((goalLocation.y - currentLocation.y), (goalLocation.x - currentLocation.x));
    }
    else if (centerLoop == 1)
    {
        centerLoop-=1; //?
        goalLocation.x = goalLocation.x + centerSearch; //sets drive location to the east branch ammount
        goalLocation.y = goalLocation.y + centerSearch / 2; //and north half that ammount
        goalLocation.theta = atan2((goalLocation.y - currentLocation.y), (goalLocation.x - currentLocation.x));
    }
    else if (centerLoop == 2) //by now we have driven north branch ammount in a location branch east
    {
        centerLoop-=1; //?
        goalLocation.x = goalLocation.x + centerSearch / 2; //drive northwest
        goalLocation.y = goalLocation.y + centerSearch;
        goalLocation.theta = atan2((goalLocation.y - currentLocation.y), (goalLocation.x - currentLocation.x));
    }
    else if (centerLoop == 3)
    {
         centerLoop-=1; //?
         goalLocation.x = goalLocation.x - centerSearch / 2; //drive west
         goalLocation.y = goalLocation.y + centerSearch;
         goalLocation.theta = atan2((goalLocation.y - currentLocation.y), (goalLocation.x - currentLocation.x));
    }
    else if (centerLoop == 4)
    {
         centerLoop-=1; //?
         goalLocation.x = goalLocation.x - centerSearch; //drive south west
         goalLocation.y = goalLocation.y + centerSearch / 2;
         goalLocation.theta = atan2((goalLocation.y - currentLocation.y), (goalLocation.x - currentLocation.x));
    }
    else if (centerLoop == 5)
    {
         centerLoop-=1; //?
         goalLocation.x = goalLocation.x - centerSearch; //drive south
         goalLocation.y = goalLocation.y - centerSearch / 2;
         goalLocation.theta = atan2((goalLocation.y - currentLocation.y), (goalLocation.x - currentLocation.x));
    }
    else if (centerLoop == 6)
    {
         centerLoop-=1; //?
         goalLocation.x = goalLocation.x - centerSearch / 2; //drive south east
         goalLocation.y = goalLocation.y - centerSearch;
         goalLocation.theta = atan2((goalLocation.y - currentLocation.y), (goalLocation.x - currentLocation.x));
    }
    else if (centerLoop == 7)
    {
         centerLoop-=1; //?
         goalLocation.x = goalLocation.x + centerSearch / 2; //drive east
         goalLocation.y = goalLocation.y - centerSearch;
         goalLocation.theta = atan2((goalLocation.y - currentLocation.y), (goalLocation.x - currentLocation.x));
    }
    else if (centerLoop == 8)
    {
         centerLoop-=1; //?
         centerSearch = centerSearch + searchDist * 2; //select a new radius to search
         goalLocation.x = goalLocation.x + centerSearch;
         goalLocation.y = goalLocation.y - centerSearch / 2;
         goalLocation.theta = atan2((goalLocation.y - currentLocation.y), (goalLocation.x - currentLocation.x));
    }
}

*/
//Jeff Schlindwein, snippet of code from last years mobility


/*//INITIAL RUN FUNCTION:  Drive forward to see if you find the center, 
void CNMInitial()
{
	first = false;
	rotBool = false;
	goalLocation.theta = currentLocation.theta;
	//drive forward 50cm
	goalLocation.x = currentLocation.x + (0.5 * cos(goalLocation.theta));
	goalLocation.y = currentLocation.y + (0.5 * sin(goalLocation.theta));
	circleLocation.x = 0;
	circleLocation.y = 0;
	driftCompensation.x = 0;
	driftCompensation.y = 0;
	//IF so go to a point in the search pattern
	if (currentLocation.theta <= 45 * M_PI / 180)
	{
        searchLoop = 3;
	}
	else if (currentLocation.theta <= 90 * M_PI / 180)
	{
        searchLoop = 2;
	}
	else if (currentLocation.theta <= 135 * M_PI / 180)
	{
        searchLoop = 1;
	}
	else if (currentLocation.theta <= 180 * M_PI / 180)
	{
        searchLoop = 8;
	}
	else if (currentLocation.theta <= 225 * M_PI / 180)
	{
        searchLoop = 7;
	}
	else if (currentLocation.theta <= 270 * M_PI / 180)
	{
        searchLoop = 6;
	}
	else if (currentLocation.theta <= 310 * M_PI / 180)
	{
        searchLoop = 5;
	}
	else if (currentLocation.theta <= 360 * M_PI / 180)
	{
        searchLoop = 4;
	}
}
*/
