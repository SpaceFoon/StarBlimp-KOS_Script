


//--------------- TIME FORMATS---------------------------------------------------
//https://www.reddit.com/r/Kos/comments/4bh15w/program_simple_code_to_convert_mission_time_into/
FUNCTION padZ { PARAMETER t, l is 2.
    RETURN (""+t):PADLEFT(l):REPLACE(" ","0").
}

// returns elapsed time in the format "[T+YY-DDD HH:MM:SS]"
FUNCTION formatMET
{
  LOCAL ts IS TIME + MISSIONTIME - TIME:SECONDS.
  RETURN "[T+" 
    + padZ(ts:YEAR - 1) + "-" // subtracts 1 to get years elapsed, not game year
    + padZ(ts:DAY - 1,3) + " " // subtracts 1 to get days elapsed, not day of year. What is the 3 for?
    + padZ(ts:HOUR) + ":"
    + padZ(ts:MINUTE) + ":"
    + padZ(ROUND(ts:SECOND))+ "]".
}
// print formatMET.

FUNCTION formatUNI
{
  LOCAL ts IS TIME.
  RETURN "[Y" 
    + round(ts:YEAR) + ", D"
    + padZ(ts:DAY) + ", "
    + padZ(ts:HOUR) + ":"
    + padZ(ts:MINUTE) + ":"
    + padZ(ROUND(ts:SECOND))+ "]".
}
// print formatUNI.

//---------------------------Steering stuff-------------------------------------

//The KSC (fully upgraded) runway's (Latitude, Longitude) starts from about (-0.0485997, -74.724375) and ends at about (-0.0502119, -74.489998).
//https://www.reddit.com/r/KerbalSpaceProgram/comments/5wl42h/kos_runway_centerline_geoposition_analysis/

//--------steers with only wheels while on runway. Holds center line. Somehow works even when you are not on the runway.

set CENTERLINE_EQ_A to 142.13236295.    // latitude coefficient for the linear equation
set CENTERLINE_EQ_B to 1.               // longitude coefficient
set CENTERLINE_EQ_C to 81.62849024.     // constant term
set RUNWAY_EAST_THRESHOLD_LAT to -0.0502118560109606. //start of runway
set RUNWAY_EAST_THRESHOLD_LNG to -74.4899977802028. //start of runway
set CENTERLINE_PID_OUTPUT_LIMIT to 2.

//steering pid
set centerline_pid to pidloop(1, .2, 2, -CENTERLINE_PID_OUTPUT_LIMIT, CENTERLINE_PID_OUTPUT_LIMIT).
set centerline_pid:setpoint to 0.

lock current_heading_target to centerline_pid:output + 90.

// set current_pitch_target to SHIP:FACING:PITCH.
lock centerline_angular_deviation to (CENTERLINE_EQ_A * ship:geoposition:lat + CENTERLINE_EQ_B * ship:geoposition:lng + CENTERLINE_EQ_C) / sqrt(CENTERLINE_EQ_A^2 + CENTERLINE_EQ_B^2).
lock centerline_linear_deviation to -2 * constant:pi * KERBIN:RADIUS * centerline_angular_deviation / 360.

//flying pid
function setPID {
    parameter axis, Kp, Ki, Kd.

    if axis = "pitch" {
        set steeringmanager:pitchpid:Kp to Kp.
        set steeringmanager:pitchpid:Ki to Ki.
        set steeringmanager:pitchpid:Kd to Kd.
    }
    else if axis = "roll" {
        set steeringmanager:rollpid:Kp to Kp.
        set steeringmanager:rollpid:Ki to Ki.
        set steeringmanager:rollpid:Kd to Kd.
    }
    else if axis = "yaw" {
        set steeringmanager:yawpid:Kp to Kp.
        set steeringmanager:yawpid:Ki to Ki.
        set steeringmanager:yawpid:Kd to Kd.
    }
    printTimestamped(axis + " PID settings updated:").
    print "  Kp: " + Kp.
    print "  Ki: " + Ki.
    print "  Kd: " + Kd.
}
// Set PID values for pitch, roll, and yaw
// setPID("pitch", 5, 0.5, .5).
// setPID("roll", 3, 0.1, 0.5).
// setPID("yaw", 3, 0.3, 0.8).

//put steering wheels in list for later use.
set wheels to LIST().
for PART in SHIP:PARTS {
    if PART:HASMODULE("ModuleWheelSteering") {
        wheels:ADD(PART).
    }
}
//Function to shut down engines and intakes properly when they are out of juice.
function monitorEngines {

    list engines in engList.  //list of all engines
    for eng in engList {
        if eng:NAME = "WBILargeElectricPart"{
            print "Engine: " + eng:NAME + " Thrust: " + ROUND(eng:THRUST) + " kN".
            if eng:THRUST < 20 {
                print "Engine thrust too low, shutting down: " + eng:NAME.
                eng:SHUTDOWN().
                SET PROPSDONE TO TRUE.
            }
        }
        if eng:NAME = "turboFanEngine" {
            print "Engine: " + eng:NAME + " Thrust: " + ROUND(eng:THRUST)  + " kN".
            if eng:THRUST < 30 {
                print "Engine thrust too low, shutting down: " + eng:NAME.
                eng:SHUTDOWN().
                INTAKES OFF.
                SET JETSDONE TO TRUE.
            }
        }
    }
}

//-----------------------------Pretty Stuff------------------------------------//
// Function to print a formatted message with a timestamp
function printTimestamped {
    parameter msg.
    print " " + formatMET + " " + msg.
}
function printDivider {
    parameter title.
    print "==========================================".
    print "= " + title + printTimestamped("").
    print "==========================================".
}

function printWelcome {
    set terminal:height to 46.
    set terminal:width to 80.
    clearScreen.
    print "============================================================================".
    print "====================              WELCOME TO THE            ================".
    print "====================       AIRSHIP LAUNCH CONTROL SYSTEM    ================".
    print "============================================================================".
    PRINT"                                 _..--=--..._        ".
    PRINT"                              .-'            '-.  .-.".
    PRINT"                             /.'   BLIMPS!    '.\/  /".
    PRINT"                            |=-                -=| ( ".
    PRINT"                             \'.              .'/\  \".
    PRINT"                              '-.,_____ _____.-'  '-'".
    PRINT"                                   [_____]=8         ".
    print "".
        wait 1.
    print "============================================================================".
    print "====================    PROGRAM 1: SINGLE STAGE TO EVE      ================".
    print "============================================================================".
    print " .                        .       ___---___                    .           .".
    print "                .              .--\        --.     .     .         .        ".
    print "                             ./.;_.\     __/~ \.                            ".
    print "    .                       /;  / `-'  __\    . \                           ".
    print "                   .       / ,--'     / .   .;   \        |                 ".
    print "                          | .|       /       __   |      -O-       .        ".
    print "         .               |__/    __ |  . ;   \ | . |      |                 ".
    print "                         |      /  \\_    . ;| \___|                 |      ".
    print "            .    o       |      \  .~\\___,--'     |                -O-     ".
    print "                          |     | . ; ~~~~\_    __|                  |      ".
    print " .           |             \    \   .  .  ; \  /_/   .                      ".
    print "            -O-        .    \   /         . |  ~/                  .        ".
    print "             |    .          ~\ \   .      /  /~          o                 ".
    print "           .                   ~--___ ; ___--~                             .".
    print "                          .          ---         .                       -JT".
    print "============================================================================".
    print "".
    TERMINAL:REVERSE.
    WAIT .2.
    TERMINAL:REVERSE.
}
//----------- Steering and Runway Loop-----------------//

//brake till stopped then go foward
SET FLAPSLVL TO 0.
function lstage1 {
        //reverse fan
        TOGGLE AG4.
        WAIT.1.
        //jets
        TOGGLE AG3.
        WAIT.1.
        //just to make sure
        intakes ON.
        SET BRAKES TO true.
        WAIT until SHIP:groundspeed < 1.
        SET BRAKES TO false.
        WAIT.1.
    }
//jump
function lstage2 {
        toggle ag5.
        set terminal:height to 44.
        set terminal:width to 74.
        wait.1.
        
        // fill baloon
        printDivider("Dem Duke boys...").
            PRINT "                                              yeehaw!     ".
            PRINT "          ".
            PRINT "                                                /     ".
            PRINT "                          !        _____     ".
            PRINT "                          | _.=-t-=''''''===--.     ".
            PRINT "                        _.=' '|'\\___```_______\_ _______     ".
            PRINT "                       //_____'--^''''-------_.-------'''''----___     ".
            PRINT "                 _.--'''' _  ,   __..__ ---''T ----- [ ```  ]---.7|     ".
            PRINT "             _.-'     __ |.|/| ''   /__'\    ['=======------======]     ".
            PRINT "            /_.---__''   |||||     |/ _ \\ _.`-------'------'__,'     ".
            PRINT "            \    / _\    '-''      |: ^!| '  _____..----' ` |     ".
            PRINT "       `-`   \___!/  \___..-------' |'_'| |''          \ `` / mozz     ".
            PRINT "     +`     `     |(|| |----..---''-\  _/ /             '--'     ".
            PRINT "       ` ^` `  `  \  / /             '--'`     ".
            PRINT "      '-`  `+`     '''`     ".
            PRINT "          ".
            PRINT "          ".
            PRINT "          ______________________________     ".
            PRINT "        _/\ ------- ---------- -------- \     ".
            PRINT "           \  ```` ``  ``` ``` ``` ```   '--.._____     ".
            PRINT "            '.  ``````` ````` ````` ``` ````       ''''------..__   ".
                    wait 1.5.
        //TOGGLE ag50. Fireworks are broken. They doin't go anywhere and break themselvs
        wait.2.
        set terminal:height to 20.
        set terminal:width to 49.
        SET FLAPSLVL TO 1.
        return FLAPSLVL.
    }

//Re-enable Rudder control.
function enableCS {
    for PART in SHIP:PARTS {
       if PART:NAME = "hl10rudder" {
            // print "Turning on part: " + PART:NAME.
          PART:GETMODULE("ModuleControlSurface"):SETFIELD("authority limiter", 32).
        }
    }
}
//With Helium!
function fillBaloon {
    for PART in SHIP:PARTS {
     if PART:HASMODULE("HLEnvelopePartModule") {
        // print "part: " + part:name.
        PART:GETMODULE("HLEnvelopePartModule"):DOACTION("buoyancy max", true).
    }
}
}

//Dont fill this one as much for balance.
function lvlBaloon {
    for PART in SHIP:PARTS {
        if PART:NAME = "hl10NoseCone" {
            // Loop to press the "buoyancy --" button 39 times
            set buttonPresses to 0.
            until buttonPresses >= 39 {
                PART:GETMODULE("HLEnvelopePartModule"):DOACTION("buoyancy --", true).
                clearScreen.
                print "lEVELING BALOONS: "+ ROUND(buttonPresses * 2.631) + " / 100".
                set buttonPresses to buttonPresses + 1.
                WAIT 0.02.
            }
        }
    }
}

function mediumCS {
    for PART in SHIP:PARTS {
       if PART:NAME = "hl10rudder" {
            // print "Turning on part: " + PART:NAME.
          PART:GETMODULE("ModuleControlSurface"):SETFIELD("authority limiter", 20).
        }
    }
}

//https://forum.kerbalspaceprogram.com/topic/181894-orbital-mechanics-of-circularization/
// circularizes at the next apsis
function CIRCLE {
set burn_at_periapsis to TRUE.
if APOAPSIS > 0 { // if apoapsis is negative, time to apoapsis is infinite
	if ETA:APOAPSIS < ETA:PERIAPSIS { SET burn_at_periapsis to FALSE. }
}
if burn_at_periapsis { set which_apsis_text to "Periapsis". } else { set which_apsis_text to "Apoapsis". }

if burn_at_periapsis {
	set node_time to TIME:SECONDS + ETA:PERIAPSIS.
	set otherapsis to APOAPSIS.
	set burnapsis to PERIAPSIS.
} ELSE {
	set node_time to TIME:SECONDS + ETA:APOAPSIS.
	set otherapsis to PERIAPSIS.
	set burnapsis to APOAPSIS.
}

set v_old to sqrt(BODY:MU * (2/(burnapsis+BODY:RADIUS) -
                             1/SHIP:OBT:SEMIMAJORAXIS)).
set v_new to sqrt(BODY:MU * (2/(burnapsis+BODY:RADIUS) -
			     1/(BODY:RADIUS+burnapsis))).
set dv to v_new - v_old.

set MyNode to NODE(node_time, 0,0,dv).
ADD MyNode.

printTimestamped ("Setting up node at " + which_apsis_text).
printTimestamped ("ETA=" + ROUND(node_time - TIME:SECONDS,1) + ", dV=" + ROUND(dv,1)).
printTimestamped ("Old eccentricity was " + ROUND(SHIP:ORBIT:ECCENTRICITY,4) + ", new eccentricity is " + ROUND(MyNode:ORBIT:ECCENTRICITY,4)).
PRINT MyNode:ORBIT:APOAPSIS.  // apoapsis after maneuver
PRINT MyNode:ORBIT:PERIAPSIS. // periapsis after maneuver
}

// Function to calculate burn time, thanks chat gipty
function calculateBurnTime {

    set totalDeltaV to mynode:DELTAV.

    set initialMass to ship:MASS.

    set totalThrust to 0.
    set totalISP to 0.
    LIST ENGINES IN engList.
    for eng in engList {
        if eng:IGNITION {
            set totalThrust to totalThrust + eng:MAXTHRUST.
            set totalISP to totalISP + (eng:ISP * eng:MAXTHRUST).
        }
    }
    set averageISP to totalISP / totalThrust.

    // Calculate the final mass after the burn using the Tsiolkovsky rocket equation
    set finalMass to initialMass / exp(totalDeltaV / (averageISP * CONSTANT:G0)).

    set massFlowRate to totalThrust / (averageISP * CONSTANT:G0).

    set burnTime to (initialMass - finalMass) / massFlowRate.

    return burnTime.
}

// Function to execute a burn at a maneuver node
function executeManeuver {

    lock steering to mynode:burnvector.
    print "Steering locked to maneuver node.".

    // Wait until the maneuver node's time minus half the burn time
    wait until eta:nextnode < (burnTime / 2).

    lock throttle to 1.
    print "Executing burn...".

    // Wait for the burn duration
    wait burnTime.

    lock throttle to 0.
    print "Burn complete.".

    // Remove this maneuver node
    remove nextnode.
}

// Function to set a Kerbal Alarm Clock alarm for the next maneuver node

set alarmTitle to "Forgot to set".
set alarmNotes to "".
function setKACAlarmForNextNode{
    parameter kacleadtime.
    parameter alarmTitle.
    parameter alarmNotes.
 addAlarm("Manuver", NEXTNODE:TIME + kacleadtime, alarmTitle, alarmNotes).
}


//for long term travel radiation management. Needs to point away from the sun for protectino.

function antisun {
    
    WAIT 5.
    // Calculate vector away from the sun
    SET sunVector TO SHIP:POSITION - BODY("Sun"):position.
    SET awayFromSun TO -sunVector:NORMALIZED.

    LOCK STEERING TO VCRS(awayFromSun, UP:STARVECTOR, SHIP:FACING:FOREVECTOR).

    // Optional: Add a delay or loop to maintain the orientation

    // Wait or loop to keep the ship pointed away from the sun
    WAIT UNTIL FALSE. // Infinite loop to hold orientation (press Ctrl+C in kOS console to exit)

    // Alternatively, you could use a loop with a check to maintain the orientation:
    // UNTIL <condition> {
    //     SET sunVector TO SHIP:POSITION - BODY("Sun"):POSITION.
    //     SET awayFromSun TO -sunVector:NORMALIZED.
    //     LOCK STEERING TO VCRS(awayFromSun, UP:STARVECTOR, SHIP:FACING:FOREVECTOR).
    //     WAIT 0.1. // Small delay to prevent script from running too frequently
    // }

}
//https://www.reddit.com/r/Kos/comments/4kk0gd/coming_out_of_time_warp/
function warpHelper {
    set warp to 1.
    wait until not ship:unpacked. // warp is actually engaged
    // wait until warp time
    set warp to 0.
    wait until ship:unpacked. // warp is actually disengaged
// vessel should imediately be responsive
}


function AreoBrakeAssist{
    SET periapsisAlt TO SHIP:OBT:PERIAPSIS - SHIP:BODY:ATM:HEIGHT. // Altitude of periapsis above atmosphere
    SET atmoEntryAlt TO 90000. // Altitude at which the atmosphere starts for Eve
    SET velocityAtPeri TO VANG(SHIP:VELOCITY:SURFACE:NORTH, SHIP:VELOCITY:SURFACE:EAST).

    SET descentDistance TO periapsisAlt - atmoEntryAlt.

    SET descentTime TO descentDistance / velocityAtPeri.

    SET safetyMargin TO 0.1. // 10% safety margin
    SET adjustedDescentTime TO descentTime * (1 + safetyMargin).

}

//---------------------------------------------------------//
// ---------------------- STARTS HERE ---------------------//-------------------------
//---------------------------------------------------------//


wait until ship:unpacked.
//starts fillballons now becuase it takes a while.
fillBaloon().
PRINT"===============================".
PRINT"".
PRINT"" + SHIP:NAME + " SSTE Config".
PRINT"".
PRINT"===============================".
PRINT"".

printDivider("FILLING BALOONS").

printTimestamped("Applying brakes and raising legs!").
brakes on.
TOGGLE AG1.

printWelcome().

printDivider("WARNING: DON'T TOUCH SAS, RCS, OR STAGING.").

PRINT "Press any key to confirm you will die if you do that...".
UNTIL terminal:input:haschar {
    TERMINAL:REVERSE.
    set tcolor to false.
    WAIT .3.
    TERMINAL:REVERSE.
    set tcolor to true.
}
if tcolor = true {
    TERMINAL:REVERSE.
}

//------------- no turning back--------
SAS OFF.
printTimestamped("SAS off").
wait .2.
printDivider("LOCKING CONTROLS").
WAIT .2.
LOCK STEERING TO HEADING(90, .4, 0).
printTimestamped("Yaw, Pitch and Roll Locked").
WAIT .2.
printTimestamped("Steering wheels locked").
WAIT .2.
printTimestamped("Control surfaces locked").

// Find and turn off all hl10rudder parts which is all of the crafts aero control surface. They will slow us down while driving.
for PART in SHIP:PARTS {
    if PART:NAME = "hl10rudder" {
        // print "Turning off part: " + PART:NAME.
        // Assuming we need to set the control surface authority to zero
        PART:GETMODULE("ModuleControlSurface"):SETFIELD("authority limiter", 0).
    }
}
WAIT .2.
RCS ON.
WAIT .2.
printTimestamped("RCS ON").
WAIT .4.
printDivider("STARTING FAN SYSTEM!").
wait .2.
printTimestamped("Opening Intakes...").
wait .2.
printTimestamped("Starting Compressors...").
wait .2.
printTimestamped("Starting Nuclear Reactor...").
wait .2.
toggle ag2.
printTimestamped("Powering up Fans...").
// reversefans
toggle ag4.
wait 1.5.

// Throttle up and release the brakes
printDivider("BACK UP SEQUENCE").

printTimestamped("Throttling up Reverse Fans!").
lock throttle to 1.
WAIT .8.
printTimestamped("Fan thrust is nominal").
BRAKES off.
printTimestamped("Brakes RELEASED!").
wait 1.

lvlBaloon().

// Driving part
set terminal:height to 20.
set terminal:width to 49.
set lstage to 0.
until lstage >= 5 {
    set myGroundSpeed to VDOT(FACING:VECTOR, VELOCITY:SURFACE).
    SET radarAltitude TO alt:radar.//old way, use vessel.
    if myGroundSpeed <= -0.05 {
        if current_heading_target >= 90{
            set equals to current_heading_target - 90.
            set reverseHeading to (90 -equals).
        }
        if current_heading_target < 90{
            set equals to ABS(current_heading_target - 90).
            set reverseHeading to (90 + equals).
        }
        lock WHEELSTEERING to reverseHeading.
        lock steering to heading(reverseHeading, .4, 0).
    } else {
        lock WHEELSTEERING to current_heading_target.
        lock steering to heading(current_heading_target, .4, 0).
    }
    //Reverse thrusters (to foward), hit brakes and fire jets. When stopped release brakes and go.
    if myGroundSpeed < -33.5 {
        set lstage to 1.
           if myGroundSpeed < -34 {
               set lstage to 2.
               lstage1().
            }
    }
 
    //flaps lvl 1 and fireworks
    //This means we hit the jump
    if radarAltitude > 4 and lstage = 2 {
        lstage2().
        rcs off.
        set lstage to 3.
        fillBaloon().
    }.
    
    //flaps lvl 2 (takeoff)
    if myGroundSpeed > 100{
        
        IF FLAPSLVL < 2{
        toggle ag5.
        SET FLAPSLVL TO 2.
        }
        
        set lstage to 4.
        //reenable control surfaces
        if myGroundSpeed > 108 {
            IF myGroundSpeed > 115{
                set lstage to 5.
            }
            enableCS().
             
        }
    }
    
    //--------------------   Steering Limiter for driving --------------------
    // Instead of tyring to make pid adjustments at different speeds, I just limit
    // the maximum steering power as you go faster. Easy way to make steering work
    // at all speeds with no pid.
    if ABS(myGroundSpeed) < 14 {
    set AngSet to 30.
    }else {
        set AngSet to ABS(40 / ABS(myGroundSpeed)).
        FOR WHEEL IN WHEELS
        {
            WHEEL:GETMODULE("ModuleWheelSteering"):SETFIELD("steering angle limiter", AngSet).
        }
    }

       clearScreen.
    // printDivider("Launch stage: " + lstage).
    if lstage <= 0{
        printDivider("Launch stage: "+LSTAGE+", Back it up!").
    } else if lstage <= 1{
        printDivider("Launch stage: "+LSTAGE+", Fire Jets").
    } else if lstage <= 2{
        printDivider("Launch stage: "+LSTAGE+", Finally Forward!").
    } else if lstage <= 3{
        printDivider("Launch stage: "+LSTAGE+", Dem Duke Boys!").
    } else if lstage <= 4{
        printDivider("Launch stage: "+LSTAGE+", DO OR DIE!").
    }

    if myGroundSpeed >= 0 {
        print "Moving Forward >>>>>>>".
        print "Target heading: " + round(current_heading_target, 2).
        }else if myGroundSpeed < -.05{
            print "Moving Backwards <<<<<<.".
            print "Target heading: " + round(reverseHeading, 2).
    }
    // SET pitch TO SHIP:FACING:PITCH.
    // PRINT "Pitch angle: " + round(pitch, 2).
    print "Centerline deviation: " + round(centerline_linear_deviation, 4).
    print "Ground Speed: "+ round(myGroundSpeed, 2) + " m/s".
    print "Wheel turn limit:" + round(AngSet, 2).
    PRINT "Radar altitude: " + round(radarAltitude, 2).

// Check if the vessel is off the ground
    // IF radarAltitude > 3 {
    //     PRINT "The vessel is off the ground.".
    // } ELSE {
    //     PRINT "The vessel is on the ground.".
    // }

    centerline_pid:update(time:seconds, centerline_linear_deviation).

    // if ship:geoposition:lng > RUNWAY_EAST_THRESHOLD_LNG {
    //     set lstage to 3.
    // }
}
CLEARSCREEN.
rcs on.
printDivider("Launch stage: 4, DO OR DIE!").
UNLOCK ALL.  // Releases ALL locks on steering and other vars not using anymore.
setPID("pitch", 5, 0.1, 4.5).
setPID("roll", 3, 0.2, 2.5).
setPID("yaw", 3, 0.3, 3).
// this number is very important. At low speeds it gets tippy and this will bad things.
SET STEERINGMANAGER:MAXSTOPPINGTIME TO 3.

printTimestamped("yaw, 3, 0.3, 3").
// LOCK STEERING TO HEADING(90.2, 0, 0).

WAIT UNTIL alt:radar > 5.
setPID("pitch", 1, 0.1, 3).
LOCK STEERING TO HEADING(90.2, 3.25, 0).
printTimestamped("RADAR > 5").
printDivider("Launch stage: 5, LIFT OFF, WE HAVE LIFTOFF!!!").
UNLOCK WHEELSTEERING.
printTimestamped("UNLOCK WHEELSTEERING.").

// printTimestamped("LOCK TO HEADING(90.2, -1, 0).").//90.2 because the runway is .4 and we want go 0 so I split the difference to avoid to much yaw at takeoff

// Hardest part: Lost alt, bounce off ground, lost alt, take a dip inthe water.
// Then finally gain alt or die in the water.
// This should happen after tail gear strike. 
WAIT UNTIL alt:radar > 15.
printTimestamped("radar > 15. Wait 3").
//Pull up very hard
setPID("pitch", 5, 0.1, 6).
LOCK STEERING TO HEADING(90.2, 14, 0).
// printTimestamped("LOCK TO HEADING(90.2, 14, 0).").
wait .5.
printTimestamped("Gear Up").
GEAR OFF.
wait 2.5.
// so we can stop pulling up...
lvlBaloon().

// Finaly gaining alt
WAIT UNTIL SHIP:verticalspeed > 5.
printTimestamped("verticalspeed > 5").
printTimestamped("LOCK TO HEADING(90, 7.25, 0).").
LOCK STEERING TO HEADING(90, 7.25, 0).
WAIT UNTIL SHIP:airspeed > 150.
// make damn sure the flaps are up.
// had problems, probably don't need this anymore.
printTimestamped("Flaps up.").
toggle AG6.
WAIT.1.
toggle AG6.
WAIT.1.
toggle AG6.
WAIT UNTIL SHIP:airspeed > 160.
toggle AG6.
printTimestamped("Flaps up.").
WAIT UNTIL SHIP:airspeed > 170.
printTimestamped("Flaps up.").
toggle AG6.
WAIT UNTIL SHIP:airspeed > 180.
toggle AG6.
printTimestamped("Flaps up.").
WAIT UNTIL SHIP:airspeed > 190.
toggle AG6.
printTimestamped("Flaps up damnt!").

//225 m/s is enough to start climbing.
//this will keep you climbing without going to fast to save on fuel.
WAIT UNTIL SHIP:airspeed > 225.
PrintTimestamped("airspeed > 225 HEADING(90, 18, 0)").
LOCK STEERING TO HEADING(90, 18, 0).
WAIT UNTIL alt:radar > 500.
printTimestamped("radar > 500").
SET CLIMB TO 0.
UNTIL CLIMB > 0{
    SET TOTALTHRUST TO 0.
    wait .3.
    CLEARSCREEN.
    printDivider("Launch stage: 6, Slow Climb.").
    PRINT "Airspeed: " + round(airspeed, 2) + " m/s".
    PRINT "Vertical Speed: " + ROUND(SHIP:verticalspeed, 2) + " m/s".
    list engines in engList.
    for eng in engList {
        SET TOTALTHRUST TO TOTALTHRUST + ENG:THRUST.
    }
    PRINT "Total Thrust: " + ROUND(TOTALTHRUST) + " kN".

 IF SHIP:airspeed > 250{
    printTimestamped("LOCK TO HEADING(90, 25, 0).").
    LOCK STEERING TO HEADING(90, 25, 0).
 }
 IF SHIP:airspeed > 240 AND SHIP:airspeed < 250{
    printTimestamped("LOCK TO HEADING(90, 22, 0).").
    LOCK STEERING TO HEADING(90, 22, 0).
 }
  IF SHIP:airspeed < 240 AND SHIP:airspeed > 230 {
    printTimestamped("LOCK TO HEADING(90, 20, 0).").
    LOCK STEERING TO HEADING(90, 20, 0).
 }
  IF SHIP:airspeed < 230 AND SHIP:airspeed > 220{
    printTimestamped("LOCK TO HEADING(90, 18, 0).").
    LOCK STEERING TO HEADING(90, 18, 0).
 }
 IF SHIP:airspeed < 220{
    printTimestamped("LOCK TO HEADING(90, 15, 0).").
    LOCK STEERING TO HEADING(90, 15, 0).
 }
 //Save RCS
 IF ship:altitude > 5500 {
    //save rcs
    rcs off.
    printTimestamped("RCS OFF").
    set CLIMB TO 1.
 }
}

// We have alt, now we need speed. A lot of speed. This will take awhile...
clearscreen.
printDivider("Launch stage: 7, Gain Speed!").
setPID("pitch", 2, 0.1, 3).
LOCK STEERING TO HEADING(90, 7.5, 0).
printTimestamped("HEADING(90, 7.5, 0).").

// Turn fans off and close intakes when they become useless.
WAIT UNTIL altitude > 6300.
SET PROPSDONE TO FALSE.
UNTIL PROPSDONE = TRUE {
    monitorEngines().
    WAIT .5.
    IF PROPSDONE = TRUE {
        TOGGLE AG2.
    }
    monitorEngines().
    WAIT .5.
}
//90747 liquid fuel needed for rockets.

until SHIP:LIQUIDFUEL <= 91427{
    WAIT .5.
    CLEARSCREEN.
    printDivider("Launch stage: 7, Gain Speed!").
    PRINT "Airspeed: " + round(airspeed, 2) + " m/s".
    PRINT "Vertical Speed: " + ROUND(SHIP:verticalspeed, 2) + " m/s".
    print("Liquid Fuel Left: " + ROUND(SHIP:LIQUIDFUEL)).
    if SHIP:LIQUIDFUEL <= 91500{
        PRINT "|=== GET READY TO BURN! In: " + ROUND((SHIP:LIQUIDFUEL - 91427)).
        //TERMINAL:REVERSE.
    }
}
// TERMINAL:REVERSE.
// WAIT .2.
// TERMINAL:REVERSE.
printTimestamped("TIME TO BURN").
rcs on.
printTimestamped("RCS ON").
setPID("pitch", 5, 0.1, 6).
// printTimestamped("PID: 5, 0.1, 6").
LOCK STEERING TO HEADING(90, 38, 0).
printTimestamped("HEADING(90, 38, 0)").
// Start pitching up then fire rockets.
// We start the burn at 91300 because the jets will still be working a bit longer.
WAIT until SHIP:LIQUIDFUEL <= 91300.
printDivider("Launch stage: 8, Pitch and BURN!").
stage.
wait 5.
rcs off.

//Start heading to space!
SET CLIMB2 TO 0.
set JETSDONE to false.
UNTIL CLIMB2 > 0{
    SET TOTALTHRUST to 0.
    wait .3.
    CLEARSCREEN.
    printDivider("Launch stage: 8, Pitch and BURN!").
    PRINT "Heading: " + ROUND(HEADING, 2) + ROUND(ship:heading, 2).
    PRINT "Airspeed: " + round(airspeed, 2) + " m/s".
    PRINT "Vertical Speed: " + ROUND(SHIP:verticalspeed, 2) + " m/s".

    PRINT "Jet Shutdown: " + JETSDONE.
    list engines in engList.
    for eng in engList {
        SET TOTALTHRUST TO TOTALTHRUST + ENG:THRUST.
    }
    PRINT "Thrust: " + ROUND(TOTALTHRUST) + " kN".
    IF SHIP:OBT:ETA:APOAPSIS > 40{
        LOCK STEERING TO HEADING(90, 35, 0).
    }
    IF SHIP:OBT:ETA:APOAPSIS > 49{
        LOCK STEERING TO HEADING(90, 30, 0).
    }
    IF JETSDONE = FALSE {
    monitorEngines().
    WAIT .3.
    }
    IF SHIP:OBT:ETA:APOAPSIS > 54{
        LOCK STEERING TO PROGRADE.
    }
    IF SHIP:apoapsis >= 75000{
        LOCK THROTTLE TO 0. 
    }
}
clearScreen.

//------------------------------- MAKE ORBIT------------------------------

printDivider("Launch stage: 9, Coast to Space!").
wait until ship:altitude = 70100.
printDivider("Launch stage: 10, Blimps in SPAAAAAACE!").
wait.5.

// mediumCS().
printTimestamped("SET STEERINGMANAGER:MAXSTOPPINGTIME TO 4.").
setPID(pitch, 3, 0.1, 6).
setPID(roll, 3, 0.1, 5).
setPID(yaw, 2, 0.1, 4).
SET STEERINGMANAGER:MAXSTOPPINGTIME TO 4.
wait .5.
//Calculate when it is time to burn, set an alarm and do it.
CIRCLE().
wait 1.
calculateBurnTime().
wait 1.
setKACAlarmForNextNode(15, "Burn to Orbit!", "This burn was brought to you by Snacky Smores. Ride the Walrus!").
wait .5.
executeManeuver().
SET ROLL_ANGLE TO 180.
SET STEERINGMANAGER:MAXSTOPPINGTIME TO 8.
antisun().


//------------------- IN ORBIT ------------------------------------//
wemadeit().
wait 5.
//open hanger
TOGGLE AG43.
wait 3.
//lights on
TOGGLE AG14.
TOGGLE AG15.
TOGGLE AG17.
TOGGLE AG13.
wait 4.
//unlock solor motors
TOGGLE AG39.
wait 1.
//deply side solar.
TOGGLE AG30.
wait 5.
//unlock boom motors
TOGGLE AG19.
TOGGLE AG20.
wait 1.
//depoly boom
TOGGLE AG22.
wait 15.
//lock motors
TOGGLE AG19.
TOGGLE AG20.
wait 2.
//depoly comms
TOGGLE AG24.
//deploy boom solar

// depoly science

//---------------------Set Course for Eve!-------------------------//
wait 1.
addons:astrogator:create(eve).
wait 1.
calculateBurnTime().
wait 1.
setKACAlarmForNextNode(600, "Burn to Eve!!!", "This burn was brought to you by Snacky Smores. Ride the Walrus!").
wait 1.
executeManeuver().
wait 1.
antisun().


//-------------------Open up the craft for long journey------------//

//open hanger
TOGGLE AG43.
//lights on
TOGGLE AG14.
TOGGLE AG15.
TOGGLE AG17.
TOGGLE AG13.
//unlock solor motors
TOGGLE AG39.
//deply side solar.
TOGGLE AG30.
//unlock motors
TOGGLE AG19.
TOGGLE AG20.
//depoly boom
TOGGLE AG22.
//lock motors
// TOGGLE AG.
//depoly comms
TOGGLE AG24.
// point away from sun

// Cooling System
TOGGLE AG36.




wait until JETSDONE = false.















function wemadeit{
            set terminal:height to 94.
        set terminal:width to 170.
print "".
print "⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣀⡀⠤⠠⠐⠒⠀⠉⣉⣉⠀⠁⠀⠀⠄⠤⠤⠬⠤⠭⢈⡉⠍⢁⠒⡒⠀⠤⠀⣀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀".
print "⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⣀⠤⠐⠒⣋⢁⡀⢀⢲⣔⣂⣀⣒⣊⣴⣚⢁⡤⣍⣻⣛⣒⣺⠶⣬⣁⣤⣍⡋⠝⢒⠴⠭⢦⣀⡒⣤⣉⡐⠂⠤⢀⡀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀".
print "⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⡀⠤⢒⠈⠀⣀⣤⣴⠿⠖⠯⠹⠛⠓⠒⠛⠛⢿⠿⣷⠾⢿⢾⡿⢯⣙⣑⣋⠛⣽⠛⢛⡩⣟⣟⣢⣽⣶⣦⣤⣀⣛⣏⠉⠛⡓⠦⢄⣁⡒⠠⢀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀".
print "⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣀⠄⢂⣁⣴⠞⠒⣊⡝⡋⢁⣠⣠⣤⣴⣴⡴⣿⢲⡖⣮⣍⡛⠙⢺⡿⣷⡤⠶⣏⢿⣻⣿⠾⣞⣓⣛⣛⠻⠟⠛⠛⢿⣻⠛⣗⣩⣭⣷⡟⢻⡉⢛⠲⢤⣀⡈⠐⡠⢀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀".
print "⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⡠⠐⣩⣴⢞⣩⣤⣍⣁⡐⡺⡆⠴⠟⠋⠉⠭⢛⠻⣿⣟⣿⣛⣿⡱⣯⡵⣶⢫⣿⡷⣍⣸⣟⣫⠭⠿⠛⠉⠉⠉⢁⠤⠤⢤⡆⣽⣿⢟⣛⣁⣁⣤⣄⣜⣿⣽⣶⣼⣿⣴⣭⣆⢈⠂⢄⡀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀".
print "⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⡀⠔⣨⣴⣿⣯⠰⢟⣟⠭⡤⣲⣏⡔⠒⢂⣤⡦⣶⣶⢶⡔⠙⣿⣾⣽⣷⣿⣳⣿⣽⢿⡽⠖⠋⢁⣀⣤⣀⣔⣀⠤⢄⣿⣿⡟⣶⣦⣾⣿⠿⣮⣿⣯⣉⣿⣿⣿⣟⠋⠉⠉⠛⠿⣯⣿⣧⣝⣞⠯⢆⢄⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀".
print "⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⠔⠊⠠⣼⣷⣿⣷⢿⣞⣯⢆⡍⣻⡿⣙⣳⣦⠾⣿⣿⣭⣷⣎⣄⣴⣿⣟⡿⣿⣿⡿⢿⣛⣭⡀⣠⣦⣵⣟⡿⣴⣿⠞⠃⠀⠹⣿⣿⣿⣿⣿⣹⣿⣿⣿⣿⣝⢼⢻⡿⣿⣷⣖⣤⣐⣄⢍⠻⣿⣾⣝⢿⣯⣜⡈⠢⡀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀".
print "⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣀⠔⠁⢀⣤⣾⣿⣿⣯⢖⣋⣍⣼⣟⣩⡞⣿⣛⣿⣶⠿⢻⣿⡿⣯⡻⣝⠿⣞⡾⣽⣿⣭⠞⠨⣋⠁⢋⣿⣿⣯⣿⣿⣿⣿⣿⢶⣄⣦⣿⣿⣿⡉⢈⠓⣿⡁⠈⢉⠙⡏⣿⡿⡿⠋⠉⢽⣯⣿⢭⣧⣴⡽⣿⣿⣶⣯⣷⡆⠈⣤⢄⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀".
print "⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⡠⠊⠀⣠⣴⣿⣿⣿⣿⡿⣱⣭⡿⣳⣿⣋⣩⣿⣟⣳⡳⡏⣁⡚⣩⣷⣳⢳⡯⣟⠺⠹⣞⣳⢶⣿⣶⣤⡾⣿⣿⢻⡗⣯⣿⣿⣿⡿⢾⣿⣿⣿⣿⣿⣿⣆⣽⣸⣄⣠⣿⣡⣷⣽⡅⠀⢠⠀⢀⣈⢻⣿⣷⣿⣽⣿⣿⣿⣿⣶⣼⣿⣹⣷⢕⢄⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀".
print "⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠞⠀⠐⣿⣿⣿⣿⣿⣻⣟⣴⣿⣿⣳⣟⣳⣿⣿⡟⡒⠙⣛⡽⣿⠋⣽⢧⢯⣳⢿⣭⣛⣟⡼⡵⠹⣖⣫⠷⣾⣿⣳⢟⣶⣿⣿⡿⠟⣻⣾⣿⣷⣯⣿⣿⣿⣿⣿⣷⣽⣿⣿⣿⣿⣿⣿⣳⡌⠦⣄⠋⣮⡿⣿⣿⣿⣿⣻⣿⣯⣿⣿⣿⣷⣿⣮⣷⢕⢄⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀".
print "⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⠔⠁⠐⣠⣼⣿⣿⣿⣿⡧⣿⣧⣦⣿⣯⢵⣻⣹⣿⣾⠛⣡⣩⡲⠧⣷⣾⡹⣞⡧⢟⡽⣲⡽⢾⣽⢟⡯⠔⠒⣼⣟⣰⣟⡿⣿⠋⠁⣠⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣮⣶⣄⡀⢘⣿⣽⠿⣿⣿⣿⣿⣿⡿⣿⣿⣿⣿⡽⡻⣿⣱⡤⡀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀".
print "⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⡠⢃⢤⣿⣻⣿⣿⣿⣿⡿⣿⣾⣯⢙⠿⢿⣷⣞⣩⣿⣭⡴⣓⣼⠯⢙⣪⠵⡼⣋⠿⡽⢊⡴⢧⣟⣳⢮⢣⢌⢦⡊⠴⣨⣿⣿⣿⣧⣶⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣎⣿⣿⣿⣟⣿⣿⣿⣿⣿⣿⣿⣭⡺⣳⣶⣮⠢⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀".
print "⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⠌⢠⣷⣏⣿⣿⣿⣿⢻⣿⠐⠋⣹⡏⠈⣦⣾⢻⢋⣵⣾⣷⢿⠟⠩⢈⣟⢌⠲⡱⣯⡟⣶⢫⡞⡽⡲⢧⠏⣞⡸⣆⢳⣿⣿⣿⣿⣾⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡷⡑⡄⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀".
print "⠀⠀⠀⠀⠀⠀⠀⠀⠀⡠⢂⣼⣽⣿⡿⠻⠋⣿⣾⣽⡧⢒⣽⡟⣼⡇⣿⣿⣿⢿⣾⢿⢫⣧⣀⡿⢏⢫⠚⣩⣵⣶⣬⣵⣧⣼⣿⣿⣿⣿⣿⣷⢲⢹⣿⠟⣿⣿⣿⣿⣾⣿⣿⣿⣿⣿⣿⣿⣿⣿⣻⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣟⣱⣾⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣷⠽⣎⢆⠀⠀⠀⠀⠀⠀⠀⠀⠀".
print "⠀⠀⠀⠀⠀⠀⠀⠀⡔⢀⣾⣟⡾⣿⣦⡔⡚⢿⣾⣉⣴⣿⣿⠟⣻⣷⢿⣿⡿⣼⣻⢆⣿⣿⡋⢤⣲⣼⣿⣿⣿⣿⣿⣿⣿⣿⣿⠿⠿⢟⡿⣟⣈⣽⠿⠉⣰⣾⣿⣿⣿⣿⣿⢿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣷⢻⣧⠢⠀⠀⠀⠀⠀⠀⠀⠀".
print "⠀⠀⠀⠀⠀⠀⠀⠜⣠⣾⣿⣽⣿⣿⣿⣿⣄⣩⠟⣡⣿⣿⣿⣷⣭⣿⣏⢿⣯⣿⣷⢎⣽⢷⣿⣐⢤⣿⣿⣿⣿⣿⣿⣿⢟⣿⣥⣶⢹⢽⣿⣿⡿⣍⣶⡟⢫⣿⣻⣿⣿⣿⣿⡦⣿⣍⡿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡿⣿⣿⣿⣿⡻⣾⣷⡡⠀⠀⠀⠀⠀⠀⠀".
print "⠀⠀⠀⠀⠀⠀⠜⣰⣿⣿⡿⣿⣿⣿⣿⣭⣿⣷⣿⣿⣿⣿⠟⡣⠺⠟⠻⣿⣿⣼⣿⡗⢻⣽⣿⣽⣿⣿⣿⣿⣾⣿⣟⡿⢻⣏⣠⣾⣽⣿⣿⢿⡿⠿⣯⣷⣄⣻⣝⣿⡿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡿⡟⣿⣿⣿⣧⣿⢯⣷⣡⠀⠀⠀⠀⠀⠀".
print "⠀⠀⠀⠀⠀⠌⣀⣿⣛⣽⠃⠻⠿⠿⢿⠋⢘⣿⣻⡿⢳⣵⠞⠓⠫⣕⣦⣌⢻⣿⣿⢲⡽⣷⣟⡼⣿⣿⣿⣾⣿⣿⣿⣿⣛⣽⢻⣿⣿⣷⣿⣽⣿⣿⣷⣤⢮⡻⠿⣿⣿⣿⣿⣹⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡿⣿⣿⣿⡿⠹⣿⣏⠉⡟⣯⢿⣻⣷⡡⠀⠀⠀⠀⠀".
print "⠀⠀⠀⠀⡘⡰⣿⡿⣿⣿⣧⣄⠴⣀⣜⣠⣿⣿⠛⢀⡟⢡⡤⠶⡛⠪⣯⢻⣈⢿⣫⣗⣻⣼⣿⢶⣿⣾⣿⣿⣿⣿⢯⢾⠽⢻⣿⣿⠟⠟⣿⣿⣿⣿⣿⣿⣿⣷⣿⣷⣽⣿⣻⡿⣿⡿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣽⣿⣿⣿⣽⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣼⣿⣿⣿⣿⣿⣿⣿⠀⢷⡜⢿⣿⣿⣵⢡⠀⠀⠀⠀".
print "⠀⠀⠀⢠⢡⣷⣿⣷⣨⣿⣿⣿⣿⣿⣿⣿⣿⣿⣟⠘⠏⣿⠐⡀⡇⣄⣿⣞⣯⠸⡿⠳⣿⣿⢿⣾⡷⡽⠛⣿⣝⣮⣟⢣⡶⣿⣿⡇⢰⣾⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣾⣷⣶⣿⣯⣹⣟⣿⣟⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣶⣗⣳⡺⣿⣿⣻⣧⠃⠀⠀⠀".
print "⠀⠀⢀⠃⣾⣿⣿⣿⢿⣮⡙⠻⣿⣽⣿⣿⣿⢿⣿⡐⠀⠉⠊⡜⣰⣿⡿⣿⡯⣷⣷⡄⣻⣹⣯⣯⣻⣿⣿⡻⣾⡷⠮⣵⣶⡿⣿⣿⣿⠶⢿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣷⣿⣿⣿⣿⣿⣿⡿⠟⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣇⡟⢿⣹⣿⣯⡟⡀⠀⠀".
print "⠀⠀⡜⣸⡟⢿⡿⠃⣾⣿⣿⣷⣾⣿⣿⣿⣿⣿⣿⣷⣖⣤⣾⣼⣷⣗⠃⢠⣿⣻⣿⣿⣿⣭⣿⣿⣿⣿⣿⣿⣦⣹⡻⣿⣛⣧⣊⣡⣯⣤⣼⣯⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣾⣿⣏⢙⣿⣿⣿⣿⣿⣷⣶⣾⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣷⣿⣇⣿⣿⣿⣿⡽⣱⠀⠀".
print "⠀⢠⢡⣿⣷⡾⣧⠀⣿⣿⣿⣿⣿⣿⡟⠿⣿⣿⣿⣿⣵⣷⣿⣿⣿⣧⠖⣿⣿⣷⡿⢻⣿⣿⣿⣿⣿⣿⣿⢿⣿⣽⣿⣿⣿⣾⣟⣿⠿⢻⣍⣻⣿⣿⣿⣿⡿⣿⣿⣿⣿⣿⣿⠿⣿⣿⣿⣿⣿⣿⣿⣿⣽⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡽⣯⣽⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣾⣿⣿⣿⣿⡃⡄⠀".
print "⠀⡌⣸⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⠛⢯⡖⠘⠉⢿⣽⣿⣿⣿⢿⠟⢃⣴⣿⣿⣿⢿⢵⣿⣿⣿⣿⣿⣯⢞⣞⣹⡟⠧⠉⠛⠿⣿⣿⣮⡤⡹⠟⢿⣿⣿⣿⣿⣿⣿⣟⣛⣯⡷⣟⢫⡵⣯⣿⣿⣿⣿⣿⣿⣿⣿⣟⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣷⠠⠀".
print "⢀⠁⣿⣿⣿⣿⣿⣿⡿⣿⣿⣿⣿⣿⡎⢀⣴⠀⣿⣿⣿⣿⣰⣿⣶⣿⢝⣿⡿⢁⡾⠐⣿⣿⣿⣿⣿⣿⣟⡿⣿⣿⣷⣶⣶⣶⣩⣽⣷⣽⣧⣟⣁⢸⣯⣛⢿⡿⣛⣎⣝⣾⣟⡾⢛⣼⣯⢹⡻⢯⣿⣩⠻⡝⢯⣭⠿⣿⣿⣿⣿⣿⣷⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣻⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣯⣿⣿⣿⡀⡄".
print "⡞⢘⣿⣿⣿⣿⣿⣿⣾⣿⣿⣿⣿⣿⣿⣿⡣⢊⣽⣿⣿⣿⣿⣿⣿⣧⣞⣟⡃⣦⢇⣬⣿⡟⣿⣿⣏⣿⣻⣷⣦⠿⣿⣿⣿⠿⡿⠿⢿⠻⢿⣿⣧⣿⣿⡛⢿⢿⣿⣴⣾⣻⣿⢻⣿⣙⡶⠯⢭⡿⣤⣾⣧⣹⣿⣿⠿⣼⢿⢿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣯⣿⣿⣿⣿⣿⣿⣿⣿⣿⡟⣿⢯⣿⢩⣿⣿⡇⢁".
print "⡇⣸⣿⣿⣿⠏⣯⣿⣾⣿⣿⣿⡿⣿⣫⣟⣾⢛⣡⣾⣿⣿⣿⣿⣿⣿⣿⣧⣮⢾⡫⡽⠿⠓⢈⣿⡿⢻⣿⣿⣿⢻⣉⣽⣿⣶⣶⣶⣿⡷⠀⠙⠛⣿⠿⠃⢹⣯⣹⣿⣿⡿⣻⣽⢏⣬⢗⢻⡹⣷⣿⣿⣺⣹⣻⣿⣿⣿⣳⣎⡽⣟⠯⣝⡻⢿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣄⡿⣻⣿⢾⣿⣿⣧⠀".
print "⡇⠉⣿⠋⢻⡘⣿⣿⣿⣿⣿⣷⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣫⣹⢿⣿⣿⣏⢼⣡⢎⠏⡀⠰⢢⠊⠳⠟⣿⣿⣿⣿⣟⣿⣿⡟⣿⣿⣿⣿⠇⣤⣄⣀⣀⠀⠀⣹⣾⣿⡏⠐⡿⣡⣽⣾⣿⣯⣷⣾⣼⣿⣿⣿⣽⡷⠿⠾⣿⣳⢽⢾⣻⢽⣳⣧⡏⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣷⣾⣿⣿⣷⣿⣿⠀".
print "⡇⣰⡷⣠⣿⣷⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣽⣿⣯⣿⣿⣿⣿⣿⣿⡿⢜⠜⡤⣾⢸⣧⣷⣷⣿⣇⢈⣯⡄⣻⣿⣴⣿⠟⣀⡹⣾⠟⠟⣿⣻⣀⣾⣿⣯⣽⣵⣿⣿⣿⣬⣗⣝⣶⣯⡖⣯⣿⣽⣿⣻⢿⣴⣋⡷⢯⣯⠿⣯⡿⣧⣟⣾⡝⢯⠿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣾⣿⣿⣿⠀".
print "⡅⣟⣡⣾⣽⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣷⣿⣿⣿⣿⣿⣿⣿⣿⣿⣷⣾⣿⣿⣿⣿⣿⣿⣏⣈⣽⣯⣶⣿⡿⢻⢋⣽⣿⣿⣿⣖⣨⣾⣩⣯⢯⣿⣫⡗⣝⣿⣷⢏⡾⢓⡇⡧⣟⣟⣯⢏⣷⣻⣽⣷⢯⡿⢼⡇⢻⠻⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⠄".
print "⡇⣻⢃⣿⣿⢿⣿⣿⣮⣿⡿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡿⣿⣿⣿⣿⣿⡟⣿⣹⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡟⣿⣿⣿⣿⣿⣾⣷⣿⣿⣿⣿⣟⣃⣼⣼⢋⣿⡟⢻⣤⡿⣫⣎⡁⢴⡾⣵⢺⣭⠽⣬⢚⣶⣓⠾⣵⣛⢾⣭⡿⣧⣿⣟⣿⣞⡟⣾⡼⣽⢧⠧⣿⢿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡇⢩⣿⣿⣿⠂".
print "⡇⢳⣿⣿⣿⣾⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣷⢿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣯⣤⡾⡧⠿⢩⠎⠁⢲⣶⣎⡷⣦⣞⢷⡳⣏⣞⣻⡘⣯⢳⣭⡻⡵⢫⡟⢞⣓⡵⣬⣬⢷⡾⣝⣳⢏⡟⡎⣿⣍⡽⣭⣛⣿⢿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣥⣰⣿⣿⣿⠀".
print "⡇⠈⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣷⣿⣿⣿⠿⣿⣿⣿⣿⣇⣿⣿⣻⡷⢿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡫⢾⡏⠀⡟⢩⣾⠷⡀⠋⠿⣗⣿⣎⢷⡓⢮⠼⠓⣷⢶⣱⢲⣕⣞⣹⣜⣳⢏⣞⡵⣯⣻⡼⣯⣝⣯⣿⣽⣿⣷⢶⢯⡿⣬⡟⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⠀".
print "⡇⠀⣛⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣷⣽⣿⣿⣿⣿⣿⣹⣯⣿⣧⡩⡿⣿⣿⣿⣿⣿⣿⣿⣿⢷⡈⣷⣬⣒⣌⢻⣦⡴⣷⣬⠍⢻⣮⣓⣹⢋⡿⣛⡥⡿⢭⡻⣶⣝⠶⣍⡞⣯⢿⣹⢯⣽⣷⣯⣾⢿⣹⣿⠽⣭⣿⣿⣿⢅⣿⣿⣿⢏⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡟⠀".
print "⢣⢠⣿⣾⣿⣿⣿⣿⣿⣿⣿⣿⣷⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣦⣿⣻⣿⣿⣟⡿⡟⣿⣿⣿⣿⡟⣽⢝⣽⣿⠏⠓⠉⠉⣿⡻⣿⣿⣿⣜⠾⢦⢿⢷⣯⣷⢿⣛⠾⣯⣛⣶⢧⣟⣞⡳⡝⣽⢹⢯⡷⣯⣟⣷⣚⢧⣟⣼⢻⣶⢿⣳⣿⣿⢸⣧⢽⡾⣿⣿⣯⠟⣿⣿⢿⣾⣿⣿⣻⣿⣿⣿⣟⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡇⠀".
print "⠈⡄.⢻⣿⣯⡿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣷⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣯⣟⣿⣿⣿⣾⣬⣞⣻⣿⣶⣶⣴⣦⣸⡁⠙⢻⣿⣿⡞⡿⣷⣿⣿⡿⣏⠟⡿⣴⣛⢮⠻⢼⡝⣳⢽⢪⡽⣶⡽⣟⡖⣷⣻⣧⣟⠾⣿⣻⡿⣷⣿⢣⣿⣟⣍⣿⣿⢽⣧⣿⣿⣿⣿⣽⣿⣿⣿⢿⣿⣿⣿⣷⣽⣿⣿⣿⣿⣿⣿⣿⣿".
print "⠀⢃⢸⣿⣿⣷⣟⣿⢿⣿⣿⣿⣿⣿⣿⣻⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣻⣿⣿⡻⣿⣟⣻⣻⣿⣟⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣧⣠⣬⣿⣛⣿⣏⢆⠻⣿⢶⣽⣞⣻⢖⣊⢷⣹⠾⣝⣫⠾⣧⣳⣈⠿⣝⣞⣲⣟⢧⣾⣻⣿⣯⢷⣻⢷⢻⣟⣧⣾⣿⣯⣿⣷⣷⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣏⢼⣿⣿⣿⡟⠀⠀".
print "⠀⠘⡈⣿⣿⣿⣽⣿⣿⣻⣿⣽⣿⡿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣷⣿⣿⣯⣿⣿⣿⣾⣿⣿⣿⣷⣿⣿⣿⡿⣿⣿⣿⣿⣿⣿⢿⡿⣿⠛⡾⢷⡹⣿⣏⠻⣿⣧⡯⣝⡼⢚⠝⣿⠿⣧⣻⣗⡛⣾⣟⣿⢽⣻⠳⣿⣿⣵⢿⣿⣻⡿⣻⣿⣿⣦⣟⣲⣿⢛⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣷⣼⣶⣿⣿⣿⢇⠃⠀".
print "⠀⠀⢣⠹⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣽⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣯⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡿⣯⣇⠈⢻⣿⣼⣷⣍⡟⢻⣷⣧⣯⢷⣿⣭⣲⣸⣴⡟⣳⡿⣏⣷⣽⣬⣾⣿⡾⣏⣟⣴⣿⡏⣿⣵⣽⣣⣿⣾⣿⣿⣿⠍⠙⢿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡘⠀⠀".
print "⠀⠀⠈⡄⠽⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣾⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣻⣿⣿⣗⣿⣿⡛⢟⢿⠉⢛⣳⣯⠻⣿⣿⣦⡽⢿⣟⣿⣿⣟⣷⣽⣱⣟⣿⣱⢾⣽⣽⣓⡽⣿⣾⢿⣻⣟⣿⣷⣯⣿⣟⣽⣿⣿⠄⠠⡔⣤⣭⣛⠿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⢣⠁⠀⠀".
print "⠀⠀⠀⠐⢀⠹⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⢽⣿⣿⣿⣯⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣷⣿⣄⢐⣟⣿⣤⣤⣟⣿⣿⣎⢫⢿⡳⣿⣿⣾⠣⣜⠓⣿⣷⣻⢿⣍⣷⡾⢹⣿⣿⣏⣿⣿⡿⣿⣿⣿⡟⠁⠀⢠⡾⠛⠋⠛⠻⠶⢌⣹⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⠏⠂⠀⠀⠀".
print "⠀⠀⠀⠀⢡⠀⠙⢿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣻⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣟⣻⡟⠛⠙⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣷⣻⣷⡿⡗⡘⠿⣤⠈⣛⣿⡼⢪⣷⣹⣿⣿⡽⣏⢿⡼⢻⣿⣷⢞⣟⣿⣻⣽⢟⣿⢽⢾⣿⣿⣿⡏⠀⠞⡏⢧⣷⣿⣏⣐⣷⣶⡳⣍⡙⢿⢿⣿⣿⣿⣿⣿⣿⡟⡘⠀⠀⠀⠀".
print "⠀⠀⠀⠀⠀⢃⠱⣆⠣⡈⠻⢿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣷⣿⣿⣾⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣶⣶⣿⣿⣿⣿⣿⣿⣿⣿⣿⢿⣻⣿⣿⣾⣍⡳⣟⣿⡿⣤⠝⣧⡵⡿⣿⣪⣿⣿⣿⣿⠻⣜⠯⢞⣽⣳⡟⣾⣻⢻⢿⠛⣹⣾⣋⡾⣿⣿⣿⣀⢀⣾⣴⣿⣿⣿⣿⣿⣿⣿⡓⠼⣿⣪⣝⣿⣿⣿⣿⣿⡿⡑⠀⠀⠀⠀⠀".
print "⠀⠀⠀⠀⠀⠀⢣⠈⣧⡈⠢⠀⢨⠛⢯⢻⣟⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡿⣽⡻⢿⣿⣿⣿⣿⣿⣽⣗⣄⡛⣿⣿⢿⣶⣿⣿⢿⣇⣘⣯⣟⣿⣿⣻⣿⣿⢣⡽⢞⣽⣫⢞⣻⣿⣿⡿⣏⣧⣿⣿⣲⣷⣾⣽⣫⣾⣴⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣳⣯⣽⣿⣿⣿⣿⣿⡿⡑⠀⠀⠀⠀⠀⠀".
print "⠀⠀⠀⠀⠀⠀⠀⢢⠹⣿⣦⣄⠀⠁⠀⠡⠈⠛⢿⣿⣿⣿⣿⣿⣿⣿⣿⣿⢿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣧⣈⡁⢶⣾⣿⣿⣿⣿⣿⣿⣿⣿⣷⣿⣿⣿⣿⣷⣿⣿⣿⣿⣼⢏⣿⣿⣿⣿⣏⢷⡹⣟⣾⣵⢯⣷⣿⣿⣷⣿⣿⡿⣯⣿⣾⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡿⡑⠀⠀⠀⠀⠀⠀⠀".
print "⠀⠀⠀⠀⠀⠀⠀⠀⠢⡘⣿⣿⣷⣤⡀⠀⠀⠐⡀⢉⠻⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣻⡿⣿⣿⣿⣿⣿⣿⣿⣯⣿⣽⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⢓⡇⢀⣷⣿⡿⣿⣘⢧⡟⣵⢿⠞⠳⣾⠿⣿⣿⣿⣿⢿⡿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⠟⠔⠀⠀⠀⠀⠀⠀⠀⠀".
print "⠀⠀⠀⠀⠀⠀⠀⠀⠀⠁⠌⠻⣿⣿⣿⣦⡀⠀⢤⡀⠀⠈⢿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣇⡾⣿⣿⣿⢾⢿⣳⣟⣾⡹⣭⢿⣅⣤⣁⣊⣉⠍⠉⢀⡾⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⢋⠌⠀⠀⠀⠀⠀⠀⠀⠀⠀".
print "⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠈⢃⠘⢿⣿⣿⣷⣦⡀⠐⡄⠀⠈⠱⡿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣽⠃⣿⠿⣿⡟⣶⣿⡕⣮⢷⣭⣯⣽⣯⣎⣁⡀⢁⣤⠴⣿⣻⢿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡿⡱⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀".
print "⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠑⡌⠙⣿⣿⣿⣿⣦⣈⡀⠀⡀⠀⠙⢿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡻⣿⣿⣿⣿⣮⣿⠿⣽⣟⣿⣿⣿⣿⣽⡿⣿⣿⣿⣿⣿⣿⣿⣿⣿⠆⣿⣗⣿⣿⣿⢯⢿⣹⣞⡷⣿⣻⣿⣽⣿⡓⠶⣽⠹⠛⣵⣯⠿⣿⣽⡿⢩⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⢟⠔⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀".
print "⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠈⠢⡀⠛⢿⡻⣿⣿⣿⣤⣈⢄⡀⠀⠘⣿⣿⣿⣿⣿⣿⣿⣿⣿⡟⢡⣾⣿⣿⣿⣿⣿⣿⣿⣿⣿⣻⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⠿⠇⣸⣽⣿⣏⢞⡷⣯⣛⠷⣮⢻⣽⣾⣿⣿⣿⣿⣷⡆⠐⢠⣿⣭⡾⣿⣿⣥⣾⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⢟⠑⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀".
print "⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢦⠉⠻⣾⣿⣿⣿⣿⣷⣭⣦⡀⠈⠺⠻⣿⣿⣿⡿⣿⣿⡆⣤⣿⣿⡿⣭⠉⠳⣻⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡿⡘⣸⣿⡿⡏⠞⣽⢾⠝⣮⣿⣾⣿⣟⢻⠿⠿⢿⠿⣿⣧⡴⢯⣍⡵⠴⢿⣿⣿⣿⣿⡿⢿⣿⣿⣿⣿⣿⣿⣿⡟⡣⠂⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀".
print "⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠙⢦⠈⠻⣟⣿⣿⣿⣿⡿⣿⣦⣀⠀⠑⢽⡻⣿⣽⡙⢿⣮⣿⡉⠻⠿⢷⣦⣄⣈⡑⠻⠽⣛⣿⠿⢿⣿⣿⣷⣿⣿⣿⡿⡟⢲⣿⠛⠻⡟⢾⣭⣯⣿⣿⣏⣋⠙⢿⣶⡄⠀⠂⣀⣩⡝⣳⣶⢋⢤⣶⣿⢽⡿⢛⣏⡉⣾⣿⣿⣿⣿⣿⠟⡩⠊⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀".
print "⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠢⡈⠛⢽⢿⣿⣿⣮⣽⣿⣷⣦⠀⠻⣿⣿⣧⡀⠉⠉⠉⠀⠀⠓⠒⠲⢮⣝⣻⣻⢶⣮⡭⣓⡲⠿⣿⣫⡯⢟⣻⢏⣼⣿⣷⣿⠷⣿⣿⣿⣿⣿⣿⡿⢿⣶⡢⠤⣇⣁⣀⣩⡶⣫⣿⣟⣾⠿⣾⢿⣾⣛⣶⣾⣿⣿⣿⡿⢟⠁⠊⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀".
print "⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠈⠢⢄⠑⠳⣾⣿⣿⠻⣻⡷⡑⡀⠙⢿⣿⣿⣿⣶⣦⣄⡤⠀⠀⢀⣤⣾⡫⠉⠡⢶⢬⠝⠻⠫⠶⢌⠙⢿⠼⣵⠿⢿⣿⣷⣿⣿⣿⣿⣿⣷⣉⡛⢻⡿⢿⣭⣿⣯⣽⣽⣿⣿⣷⣜⣶⡶⣾⣯⣝⣫⣽⣿⣿⠟⣋⠔⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀".
print "⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠁⠢⣌⠙⠛⠿⣯⣿⠷⣱⣄⡀⠈⠹⠿⣿⡛⢿⠿⣷⣀⠘⢿⣟⠁⠀⢩⢃⣭⣻⣟⣳⣄⣀⠀⠘⠀⣶⣆⣛⠛⣷⣿⣿⠿⣿⡿⣷⣟⣿⣿⣟⣿⣶⣾⣿⣿⣿⣿⡿⠿⠻⣟⡿⣿⣿⣿⣿⠿⢋⠕⠊⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀".
print "⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠈⠑⠢⣀⠈⠻⣆⣫⣽⡿⣗⢦⡐⣤⣉⣈⡽⠚⢻⡿⠟⠚⡩⠔⣣⣾⣿⣋⣩⡍⠿⢿⣿⣦⡄⠿⣾⣧⣶⣍⣉⣛⣳⣥⣴⣮⡽⠿⠟⢻⠿⣛⣟⡏⣡⣴⣶⣿⣟⡵⣿⣽⣿⠿⢋⠅⠊⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀".
print "⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠈⠂⠤⡉⠉⠛⢋⣝⢪⠬⣑⣂⣠⣙⣩⣄⣶⣿⣶⣿⠿⠷⠟⠋⠩⣭⣿⣷⣿⣿⣿⣷⣄⠈⠉⠉⠉⢉⣀⣤⣴⣶⣾⣿⣿⢛⣛⣯⠳⠞⣛⣻⡿⣿⠻⢛⣋⠫⠐⠉⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀".
print "⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠁⠒⠄⢀⡀⠙⠥⣘⠛⠛⡛⢋⣉⢩⣫⣭⣵⣶⣶⡿⠿⠛⢏⠾⠾⠥⢒⠛⣛⠻⠟⠛⠛⠛⢚⡹⠋⡭⠒⠉⣀⠈⣀⣄⣳⠶⠟⢉⣉⠦⠒⠈⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀".
print "⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠉⠒⠀⠤⣀⠈⠈⠉⠉⠋⠉⠹⠩⠄⠀⠀⠀⠁⠀⠀⠀⠀⠉⠀⠀⠀⠀⠉⠈⠁⠀⢀⠀⣤⠤⡴⢛⣉⠱⠔⠚⠉⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀".
print "⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠉⠁⠒⠂⠤⠤⠄⣀⢀⣠⣀⣀⣀⣀⣀⣀⣀⣀⣈⣠⣤⣴⣂⣰⡮⠦⠔⠓⠈⠉⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀".
print "".
print "".
print "".
print "".
print "".
}









// PRINT SHIP:PARTSINGROUP("AG5").


// function getFlapStatus {
//     for part in ship:parts {
//         // Check if the part has the FAR module
//         if part:hasmodule("FARControllableSurface") {
//             // Get the FAR module
//             set farModule to part:getmodule("FARControllableSurface").
            
//             // Check if the part has a flap deflection field
//             if farModule:hasfield("Current flap setting") {
//                 set flapSetting to farModule:getfield("flap").
//                 print "Part: " + part:name + ", Flap Setting: " + flapSetting.
//             } else {
//                 print "Part: " + part:name + " does not have a flap setting field.".
//             }
//         }
//     }
// }
