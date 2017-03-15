MOBILITY

--------3/4/2017--------
EDITED BY: Jeff Schlindwein

FIXED OBSERVATIONS BY:  Steve Lindsey

- Observation A
  PROBLEM:    If 2 rovers turn towards eachother at start, they get tangled up in obstacle detection
  SOLUTION:   - Rovers must wait an amount of time after seeing center to prevent calling obstacle detection
		on eachother.  Can use a timer to correct the issue.
	      - ADDED:  Timer to not allow rovers to enter a detect obstacle state until after 10 seconds from boot
	      - MODIFIED:  Made all rovers turn left as opposed to changing turning direction based on search pattern
		+ Rovers facing opposing directions would turn into eachother and cause a massive traffic jam
----------------------------------------------------------------------------------------------------------------------
  POTENTIAL PROBLEMS:  Rovers MAY run into eachother at start... must test further for general reactions
----------------------------------------------------------------------------------------------------------------------

--------3/6/2017 - 3/11/2017--------
EDITED BY:  Jeff Schlindwein

- Commented and Re-Commented Code
  + Most variables, functions, and timers have all been revisited with comments
    - Lines 190 - 356
  + ALL Functions Have Been Organized an Labeled Appropriately
    - Lines 1080 - 2113
    
- ADDED:
  + CNMAVGMap Function (VOID) - (PROTOTYPE: LINE 327; DEFINITION: LINE 1824) 
    - This function will only run once the rover has completed a full
        rotation around the octagon and HAS found the center.  At each
        point, the rover will collect a GPS and Odom location and blend
        them together by averaging them.
    - Created 4 Arrays for this function
      + GPS ARRAY  (LINE:  176 & 177)
        - mapCenterXCoordinates
        - mapCenterYCoordinates
      + ODOM ARRAY  (LINE:  180 & 181)
        - mapOdomXCoordinates
        - mapOdomYCoordinates
 + CNMAVGCenter Function (VOID) - (PROTOTYPE: LINE 325; DEFINITION: LINE 1755)
  - Takes a derived center point, puts it in an array of other
      center points and averages them together... allowing us to
      build a more dynamic center location (able to adjust with drift)
  - Created 2 Arrays for this function
    + Center Array (LINE:  172 & 173)
      - CenterXCoordinates
      - CenterYCoordinates
      
- MODIFIED:
  + Initial Center Search
    - Broke Up Into 3 Timers/Handlers
      + Reworked all 3 timers  (LINES:  251 - 266)
        - Changed all Timers to run on 10 second delays
      + Reworked Handlers 
        - CNMInitPositioning
          + Waits for 10 second timer to finish before giving a point to drive forward too
        - CNMForwardInitTimerDone
          + Waits 10 seconds after previous timer before telling the rover to turn 180 Degrees
        - CNMInitialWait
          + Waits 10 seconds after the 180 Degree turn before continuing an interrupted search for
            the center.
            - Changed this behavior to make initial center searching more dynamic
            - Will pivot at 0.5 until it finds the center            
    - Reworked Base Function
      + CNMFirstBoot (VOID) - (PROTOTYPE: LINE 310; DEFINITION: LINE 1452; CALLED: LINE 505)
        - This Function wraps up 3 other functions and timers
          + Runs through multiple nested IF statements and will break if the rover sees the center
            at any point
          + Check Boolean triggers for changes in timer states
            - Global Variables Instantiated LINES: 216 - 220
          + Runs ONLY on initial boot, and will require rover being restarted to re-initiate behavior

  + OBSTACLE AVOIDANCE
    - Altered behavior for obstacle avoidance (WORKING WELL)
    ---BEHAVIOR NOTES---
      + Was very basic and needed improvement
      + Most Handling Done in Obstacle Handler (LINE: 915)
        - When rovers see Obstacle they still stop for 10 seconds
          + If the object moves, it continues on previous path
          + If the object does NOT move, it tries to rotate until
            obstacle no longer is being observed from its ultrasound
        - If they must rotate to avoid obstacle
          + tell search controller we avoided an obstacle
            - begin counting obstacle calls
            - if it receives too many obstacle calls
              + try going to the next point
          + Check to see if we are alternating (AKA Searching Left or Searching Right)
            - This changes how the Rover Responds to the obstacle
            - Will always try to rotate INWARDS based on its search pattern
              + Derives a point based on this information to rotate to
        - Once It has rotated
          + Continue to rotate 30 degrees
            - This was implemented to further reduce obstacle calls
          + Continue on to appropriate point    
    ---BEHAVIOR LOCATIONS---
      + Global Variables Instantiated LINES: 224 - 225
      + Almost all handling is done IN Obstacle Handler (LINE: 915)
        - 2 static bools (limits global scope variables)
          + Static allows the variables to retain their values after function runs
          + Used for keeping track of states
        - Wait 10 Second Behavior
          + Obstacle Timer Triggered LINE: 927
            - Triggers bool cnmAvoidObstacle to true after 10 seconds
            - If Obstacle Moves
              + Code resets state LINE: 966 & 967
        - Rotates
          + If obstacle doesn't move, it triggers rotation
          + Tells searchController of the change (LINE: 949)
          + Chooses which way to rotate (LINES: 954 - 959)
            - Uses sendDriveCommand
        - Continues to Rotate 30 Degrees, Drive, and Reset (LINES: 969 - 1003)

  + Reverse Behavior
    - REWORKED REVERSE BEHAVIOR
      + Changed Timer Times/Handlers
      + Added Function
        - Instead of changing the state of a bool, made a function to do all the work
          + CNMStartReversing (VOID) - (PROTOTYPE: LINE 313; DEFINITION: LINE 1567)
            - Kicks off initial timers and states
        - ADDED cnmReverseReset (VOID) - PROTOTYPE: LINE 314; DEFINITION: LINE 1548)
          + Resets all variables to their default state
          + Useful for stopping and starting reverse behavior   
        - Still FORCES rover to reverse (no elegant solution) LINE: 507

  + HAVE HEAVILY MODIFIED TARGET HANDLER LINE 642
    - Now counts number of targets it sees (not just center tags)
    - Figues out how many targets are on the left and right
      + Uses this info for choosing which way to turn for target avoidance (BELOW)
    - Works For Specific States
      + Roving and Looking For Tags
        - Counts center tags, how many, and reacts (NOT NEW)
        - Counts target tags, how many, and reacts (NEW)
      + Responds to the center tags
        - Squares p
        - Plots center point
        - Does reverse behavior
      + If sees a tag and not currently carrying one
        - WORK IN PROGRESS!!!!
        - If we see a target
          + Behavior is dependant on several factors
            - Have a tag already
              + Avoid
            - Haven't found the center
              + Avoid
          + If it can pick it up
            - Tell the rover it can pick it up
            - Switch States
            - Built Member Function CNMTargetPickup (VOID) - (PROTOTYPE: LINE ; DEFINITION: LINE )
              + Sets gripper and wrist positions based on results of pickup attempts
                - Set from pickUpController
              + Is passed the pickup result to perform task
        - CHECK TARGET AVOIDANCE FOR FURTHER INFORMATION ON TARGET HANDLER
  
  + Center Finding Behavior
    - Used DropOff Code for squaring the rover up to the nest
      + Code is in CNMCentered (BOOL) - (PROTOTYPE: LINE 323; DEFINITION: LINE 1632)
        - returns true only if it is squared up
      + If CNMCentered returns true
        - projects center point
        - stores it in AVGCenter
      + Reverses
    - Has 2 seperate member functions
      + CNMFirstSeenCenter (VOID) - (PROTOTYPE:  LINE 316; DEFINITION: LINE 1696)
        - Initial Center Find        
      + CNMRefindCenter (VOID) - (PROTOTYPE: LINE 317; DEFINITION: LINE 1729)
        - Refind Center
    - BOTH member functions act the same currently, but can be manipulated to do seperate things

  + Target Avoidance (STILL W.I.P.)
    - Target Avoidance Still Needs LOVE
    ---TARGET AVOIDANCE NOTES---
    - This behavior is quite complex
      + Based on multiple factors
        - If it has a target and seen center
          + Dropping Off
          + Driving TO Center
        - If it hasn't seen the center
      + CURRENT PROGRESS:
        - WORKING BEHAVIOR:
          + Rover has target and Dropping Off

CURRENT END OF FILE:  2113

--------3/4/2017--------
EDITED BY:  Jeff Schlindwein, Steve Lindsey, Kaily Young, Juan Rueda, Paul Ward

- FIXED:  Target avoidence protocol
  + Implemented rough sketch of working behavior for handling other targets when carrying one
    - ADDED:  Bool called cnmRotate
      + is called when we see another block in our camera, calls behavior chain
        - turns to the right for 4 seconds
        - drive forward for .35 meters
        - continues on previous path
- ADDED:  CENTEROFFSET double
  + is a constant for changing the offset we place the center at when we see it
-Cleaned up Mobility

--------3/1/2017--------
EDITED BY:  Jeff Schlindwein, Steve Lindsey, Kaily Young, Juan Rueda, Paul Ward

- UPDATED Timer Information
  + added new Timer for initial beginning behavior (turn 180 degrees and drive to get out of the center) 
    (LINES: 197, 198, 274, 500, 848, 1091)
  + added publisher for when we see the nest for the first time
    (LINE 492)
  + working on code that offsets center location from where robot last see's tag 256
    - currently works when robot is spinning 180, but gets messy after that.
      + loop gets noticeably disorganized
- TODO:
  + add better behavior for navigating after seeing center
  + work on behavior for spreading out robots
  + start delving into pickup and drop off behavior
  
- ISSUES:
  + When running rovers only, the SIMULATION will acknowledge the existance of the new initial TIMER
    - Behavior is noticeably different.  Will have to start watching real rovers for further code changes.
      + robots act erratically for no reason
  + When running rovers AND targets, the SIMULATION REFUSES to acknowledge there is a TIMER OR a BOOLEAN
    trigger for the init behavior.
      - must test on REAL ROVERS to see actual behavior

--------2/19/2017--------
EDITED BY:  Jeff Schlindwein

- changed obstacle detection series to run off a timer as opposed to running off of counter  
  + (LINES:  191, 262 , 600, 627, 1033)
  + increases accuracy and more easily understood time increments (set at line 189)
  + allows us to follow this timer model for other behavior
- added publisher messages for when avoiding obstacles.  Watch the log box in the rover GUI for the updates

--------2/18/2017--------
EDITED BY:  Jeff Schlindwein

- Moved beginning initialization into a CNM INIT file.  This is not a permenant place, but this can be manipulated to add beginning behavior before conducting search pattern

- Full changes to Obstacle Handling goes as follows:
  + Under void ObsacleHandler() (LINE 545)
    - First change is a bool called cnmAvoidObstacle
      + initially set to false, only triggered true if an obstacle is detected past its 45 counter limit (read below for more details).
      + if this is triggered to be true, find a point 90 degrees to the left and drive to it
      + Once the sensors report no obstacle, trigger cnmAvoidObstacle back to false
    - Second change is in the main if statement (if !targetdetected || !collected)
      + if we see an obstacle wait for a counter to increment to 45 (based on cycle rate of the program)
      + once it increments to 45 we have been sitting for ~20-30 seconds
      + change behavior by triggering cnmAvoidObstacle bool
      
- Full changes to how searchController works as follows:
  + if we are driving from point a to point b, call searchController.search to get the next point
  + if we come across a target, obstacle, or anything that changes our previous point, call interrutedSearch function
    + check searhController.cpp readme for interruptedSearch and Search function information.
    
- if we see the center location and are not carrying anything
  + back up until it is no longer in view
  + call interruptedsearch function.

  TODO:
  - REPLACE the current counter with a timer that is set for more precise behavior

--------2/16/2017--------
EDITED BY:  Jeff Schlindwein

- Reworked obstacle detection
  + added boolean triggers and re-implemented old obstacle design
  + made the robot treat all obstacles the same (turn left)

--------2/15/2017--------
EDITED BY:  Jeff Schlindwein

- Organized all variable and class object instantiations.
  + #defines
  + Class Objects
  + standard variables
  + Functions
  + Publishers/Subscribers/Timers
  + Callback Handlers
  + CNM Code
- Moved rest of the main MOBILITY STATE MACHINE into dedicated functions
  + functions are under CNM Code Follows:
- Finished cleaning curly braces for readability


--------2/8/2017--------
EDITED BY:  Jeff Schlindwein, Steve Lindsey, Kaily Young, Juan Rueda, David Kirby, Rudy Garcia, Kristin Markle, Paul Ward

CHANGES:
- Created a section for CNM Code to go
  + line 163
- Moved speed variable to CNM Code Section to find more easily
  + float searchVelocity = 0.4 // originally .2
- Created Function void CNMPickUpReturn()
  + line 442
  + Code for the STATE_MACHINE_PICKUP in MOBILITYSTATEMACHINE
- Moved PickUpController to cnmPickupReturn()
  + line 743
- Created bool variable cnmPickupReturn
  + line 170
  + Default instantiated to FALSE
  + used to keep track of if the code needs to send a return statement

TODO:
  - Move other MOBILITYSTATEMACHINE code in switch statement to seperate functions
