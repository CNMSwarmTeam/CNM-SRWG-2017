#include "SearchController.h"

//CONSTANTS
//--------------------------------------
double const MIN_DIST_SEARCH = 1.25;
double const MAX_DIST_SEARCH = 3.0;

SearchController::SearchController() 
{    
    rng = new random_numbers::RandomNumberGenerator();

    //CNM VARIABLES
    //--------------------------------------
    searchLoop = 1;//rng->uniformInteger(0, 8);                 //DEFAULT TO 0, uncomment for random point in loop between 0 and 8
    searchCounter = .5;                                         //default value for search distance is .5
    searchDist = .35;                                           //how much to add to our search pattern (.2 is roughly the width of a swarmie)

    numTimesAvoidedObst = 0;

    //random distance from center to start searching
    //cnmSearchCounterDistance = 0;  //2;

    cnmNumRotations = 0;
    minNumRotations = 4;

    cnmCenterLocation.theta = 0;
    cnmCenterLocation.x = 0;                                    //set default center to (0,0)
    cnmCenterLocation.y = 0;

    reverseSearch = false;
    avoidedObstacle = false;

    doneFirstRotation = false;
    doneOneFullRotation = false;
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

    if(dist % 2 != 0 && dist > 2)
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

    cnmNumRotations++;

    //RESET variable
    avoidedObstacle = false;

    return goalLocation;
}

geometry_msgs::Pose2D SearchController::SearchRight(geometry_msgs::Pose2D currentLocation)
{

    geometry_msgs::Pose2D goalLocation;
 
      //if for some reason searchLoop goes out of bounds, reset
      //---------------------------------------------
    if (searchLoop < 0 || searchLoop > 9) { searchLoop = 0; }

    //This algorithm uses the trigonometic coordinates for the unit circle to navigate around a central point
    //using the center location as the point where we pivot around, as opposed to the current location, allows
    //us to be more precise

    //Same as point 1, but increments outwards
    //---------------------------------------------
    if (searchLoop == 0)
    {

	checkIfIncrementing();

        goalLocation.x = cnmCenterLocation.x + searchCounter;
        goalLocation.y = cnmCenterLocation.y + searchCounter / 2;
	goalLocation.theta = atan2((goalLocation.y - currentLocation.y), (goalLocation.x - currentLocation.x));

    }
    else if (searchLoop == 9)
    {

	if(!avoidedObstacle) { searchLoop = 0; }

        goalLocation.x = cnmCenterLocation.x + searchCounter;
        goalLocation.y = cnmCenterLocation.y + searchCounter / 2;
        goalLocation.theta = atan2((goalLocation.y - currentLocation.y), (goalLocation.x - currentLocation.x));

    }
    //between 0 and PI/6
    //---------------------------------------------
    else if (searchLoop == 8)
    {
	if(!avoidedObstacle) { searchLoop = 0; }

        goalLocation.x = cnmCenterLocation.x + searchCounter;
        goalLocation.y = cnmCenterLocation.y + searchCounter / 2;
        goalLocation.theta = atan2((goalLocation.y - currentLocation.y), (goalLocation.x - currentLocation.x));

    }
    //between 2PI/6 and PI/2
    //---------------------------------------------
    else if (searchLoop == 7)
    {
	if(!avoidedObstacle) { searchLoop++; }

        goalLocation.x = cnmCenterLocation.x + searchCounter / 2;
        goalLocation.y = cnmCenterLocation.y + searchCounter;
        goalLocation.theta = atan2((goalLocation.y - currentLocation.y), (goalLocation.x - currentLocation.x));

    }
    //between PI/2 and 4PI/6
    //---------------------------------------------
    else if (searchLoop == 6)
    {
	if(!avoidedObstacle) { searchLoop++; }

        goalLocation.x = cnmCenterLocation.x - searchCounter / 2;
        goalLocation.y = cnmCenterLocation.y + searchCounter;
        goalLocation.theta = atan2((goalLocation.y - currentLocation.y), (goalLocation.x - currentLocation.x));

    }
    //between 5PI/6 and PI
    //---------------------------------------------
    else if (searchLoop == 5)
    {
	if(!avoidedObstacle) { searchLoop++; }

        goalLocation.x = cnmCenterLocation.x - searchCounter;
        goalLocation.y = cnmCenterLocation.y + searchCounter / 2;
        goalLocation.theta = atan2((goalLocation.y - currentLocation.y), (goalLocation.x - currentLocation.x));

    }
    //between PI and 7PI/6
    //---------------------------------------------
    else if (searchLoop == 4)
    {
	if(!avoidedObstacle) { searchLoop++; }

        goalLocation.x = cnmCenterLocation.x - searchCounter;
        goalLocation.y = cnmCenterLocation.y - searchCounter / 2;
        goalLocation.theta = atan2((goalLocation.y - currentLocation.y), (goalLocation.x - currentLocation.x));

    }
    //between 7PI/6 and 8PI/6
    //---------------------------------------------
    else if (searchLoop == 3)
    {
	if(!avoidedObstacle) { searchLoop++; }

        goalLocation.x = cnmCenterLocation.x - searchCounter / 2;
        goalLocation.y = cnmCenterLocation.y - searchCounter;
        goalLocation.theta = atan2((goalLocation.y - currentLocation.y), (goalLocation.x - currentLocation.x));

    }
    //between 9PI/6 and 10PI/6
    //---------------------------------------------
    else if (searchLoop == 2)
    {
	if(!avoidedObstacle) { searchLoop++; }

        goalLocation.x = cnmCenterLocation.x + searchCounter / 2;
        goalLocation.y = cnmCenterLocation.y - searchCounter;
        goalLocation.theta = atan2((goalLocation.y - currentLocation.y), (goalLocation.x - currentLocation.x));

    }
    //between 10PI/6 and 11PI/6
    //---------------------------------------------
    else if (searchLoop == 1)
    {
	if(!avoidedObstacle) { searchLoop++; }

        goalLocation.x = cnmCenterLocation.x + searchCounter;
        goalLocation.y = cnmCenterLocation.y - searchCounter / 2;
        goalLocation.theta = atan2((goalLocation.y - currentLocation.y), (goalLocation.x - currentLocation.x));

    }

    return goalLocation;

}

geometry_msgs::Pose2D SearchController::SearchLeft(geometry_msgs::Pose2D currentLocation)
{
    geometry_msgs::Pose2D goalLocation;

    //if for some reason searchLoop goes out of bounds, reset
    //---------------------------------------------
    if (searchLoop < 0 || searchLoop > 9) { searchLoop = 0; }

    //This algorithm uses the trigonometic coordinates for the unit circle to navigate around a central point
    //using the center location as the point where we pivot around, as opposed to the current location, allows
    //us to be more precise

    //Same as point 1, but increments outwards
    //---------------------------------------------
    if (searchLoop == 0)
    {
	checkIfIncrementing();

        goalLocation.x = cnmCenterLocation.x + searchCounter;
        goalLocation.y = cnmCenterLocation.y + searchCounter / 2;
        goalLocation.theta = atan2((goalLocation.y - currentLocation.y), (goalLocation.x - currentLocation.x));

	//Should SKIP point 1 entirely and go to point 2.  Point 1 and point 0 are the same, retained only for
	//rovers continueInterruptedSearch (so they don't try to increment when picking a point).  This is ONLY 
	//valid for the left rotation as the right rotation has a different first point

    }

    //between 0 and PI/6
    //---------------------------------------------
    else if (searchLoop == 1)
    {
	if(!avoidedObstacle) { searchLoop++; }

        goalLocation.x = cnmCenterLocation.x + searchCounter;
        goalLocation.y = cnmCenterLocation.y + searchCounter / 2;
        goalLocation.theta = atan2((goalLocation.y - currentLocation.y), (goalLocation.x - currentLocation.x));
    }

    //between 2PI/6 and PI/2
    //---------------------------------------------
    else if (searchLoop == 2)
    {
	if(!avoidedObstacle) { searchLoop++; }

        goalLocation.x = cnmCenterLocation.x + searchCounter / 2;
        goalLocation.y = cnmCenterLocation.y + searchCounter;
        goalLocation.theta = atan2((goalLocation.y - currentLocation.y), (goalLocation.x - currentLocation.x));

    }
    //between PI/2 and 4PI/6
    //---------------------------------------------
    else if (searchLoop == 3)
    {
	if(!avoidedObstacle) { searchLoop++; }

        goalLocation.x = cnmCenterLocation.x - searchCounter / 2;
        goalLocation.y = cnmCenterLocation.y + searchCounter;
        goalLocation.theta = atan2((goalLocation.y - currentLocation.y), (goalLocation.x - currentLocation.x));

    }
    //between 5PI/6 and PI
    //---------------------------------------------
    else if (searchLoop == 4)
    {
	if(!avoidedObstacle) { searchLoop++; }

        goalLocation.x = cnmCenterLocation.x - searchCounter;
        goalLocation.y = cnmCenterLocation.y + searchCounter / 2;
        goalLocation.theta = atan2((goalLocation.y - currentLocation.y), (goalLocation.x - currentLocation.x));

    }
    //between PI and 7PI/6
    //---------------------------------------------
    else if (searchLoop == 5)
    {
	if(!avoidedObstacle) { searchLoop++; }

	goalLocation.x = cnmCenterLocation.x - searchCounter;
        goalLocation.y = cnmCenterLocation.y - searchCounter / 2;
        goalLocation.theta = atan2((goalLocation.y - currentLocation.y), (goalLocation.x - currentLocation.x));

    }
    //between 7PI/6 and 8PI/6
    //---------------------------------------------
    else if (searchLoop == 6)
    {
	if(!avoidedObstacle) { searchLoop++; }

        goalLocation.x = cnmCenterLocation.x - searchCounter / 2;
        goalLocation.y = cnmCenterLocation.y - searchCounter;
        goalLocation.theta = atan2((goalLocation.y - currentLocation.y), (goalLocation.x - currentLocation.x));

    }
    //between 9PI/6 and 10PI/6
    //---------------------------------------------
    else if (searchLoop == 7)
    {
	if(!avoidedObstacle) { searchLoop++; }

        goalLocation.x = cnmCenterLocation.x + searchCounter / 2;
        goalLocation.y = cnmCenterLocation.y - searchCounter;
        goalLocation.theta = atan2((goalLocation.y - currentLocation.y), (goalLocation.x - currentLocation.x));

    }
    //between 10PI/6 and 11PI/6
    //---------------------------------------------
    else if (searchLoop == 8)
    {

	if(!avoidedObstacle) { searchLoop++; }

        goalLocation.x = cnmCenterLocation.x + searchCounter;
        goalLocation.y = cnmCenterLocation.y - searchCounter / 2;
        goalLocation.theta = atan2((goalLocation.y - currentLocation.y), (goalLocation.x - currentLocation.x));

    }
    //between 0 and PI/6
    //---------------------------------------------
    else if (searchLoop == 9)
    {
	if(!avoidedObstacle) { searchLoop = 0; }

        goalLocation.x = cnmCenterLocation.x + searchCounter;
        goalLocation.y = cnmCenterLocation.y + searchCounter / 2;
        goalLocation.theta = atan2((goalLocation.y - currentLocation.y), (goalLocation.x - currentLocation.x));
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

void SearchController::LeftSearch(double absLocationAngle)
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

void SearchController::RightSearch(double absLocationAngle)
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

void SearchController::checkIfIncrementing()
{

    //check to see if we have done enough parts of the octagon to warrant incrementing
    if(cnmNumRotations > 6) { doneOneFullRotation = true; }

    //if we haven't tried to avoid an obstacle or can increment
    if(!avoidedObstacle && doneOneFullRotation) 
    {
	searchCounter = searchCounter + searchDist;	//increment search counter	
    }

    int dist = floor(searchCounter);			//check to see if incrementing will have us start alternating

    if(dist % 2 != 0 && dist > 2) { searchLoop++; }	//right search is fine to increment +1
    else { searchLoop += 2; } 				//left search needs to start at point 2 because point 0 and 1 are the SAME point

    //check to see if we need reset value
    if(doneOneFullRotation == true) { doneOneFullRotation = false; }

    //reset rotations
    cnmNumRotations = 0;
}

void SearchController::setCenterLocation(geometry_msgs::Pose2D newLocation)
{
    cnmCenterLocation = newLocation;
}

int SearchController::cnmGetSearchPosition()
{
    if(searchLoop == 0)
    {
        return 9;
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

void SearchController::AmILost(bool answer)
{
    //Reset search controller variables
    //---------------------------------------------
    reverseSearch = false;
    avoidedObstacle = false;
    doneFirstRotation = false;
    doneOneFullRotation = false;
    cnmNumRotations = 0;

    //If WE LOST CENTER
    if(answer == true)
    {
	searchLoop = rng->uniformInteger(1, 10);	//Start at random place in octagon(NOT 0)
	searchCounter = .5;				//Reduce searchCounter to 0.5
    }
    else
    {	
	searchCounter = cnmSearchCounterDistance;	//Returns searchCounter to ORIGINAL STATE
    }
}

void SearchController::setSearchDist(int id)
{ 
    
   if(id < 3)
   {
       cnmSearchCounterDistance = 1.25 + id;
   }
   else
   {
       cnmSearchCounterDistance = 3.25 + (.35 * (id - 2));
   }

   searchCounter = cnmSearchCounterDistance;
 
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
