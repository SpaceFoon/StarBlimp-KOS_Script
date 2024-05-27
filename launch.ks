//The KSC (fully upgraded) runway's (Latitude, Longitude) starts from about (-0.0485997, -74.724375) and ends at about (-0.0502119, -74.489998).
//https://www.reddit.com/r/KerbalSpaceProgram/comments/5wl42h/kos_runway_centerline_geoposition_analysis/
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

// Function to print a formatted message with a timestamp
function printTimestamped {
    parameter msg.
    print "[" + formatMET + "] " + msg.
}
function printDivider {
    parameter title.
    print "==========================================".
    print "= " + title.
    print "==========================================".
}



wait until ship:unpacked.
PRINT"===============================".
PRINT"".
PRINT"" + SHIP:NAME + " SSTE Config".
PRINT"".
PRINT"===============================".
PRINT"".

function printBlimp {
    PRINT"       _..--=--..._".
    PRINT"    .-'            '-.  .-.".
    PRINT"   /.'   BLIMPS!    '.\/  /".
    PRINT"  |=-                -=| (".
    PRINT"   \'.              .'/\  \".
    PRINT"    '-.,_____ _____.-'  '-'".
    PRINT"         [_____]=8".
}
function printWelcome {
    clearScreen.
    print "==========================================".
    print "=              WELCOME TO THE            =".
    print "=       AIRSHIP LAUNCH CONTROL SYSTEM    =".
    print "==========================================".
    printBlimp().
    print "==========================================".
    print "=    PROGRAM 1: SINGLE STAGE TO EVE      =".
    print "==========================================".
}
printWelcome().


set desiredHeading to 90.  // East
set desiredPitch to 0.    // Horizon
set desiredRoll to 0.      // Level

// Apply the brakes and trigger action group 1
printTimestamped("Applying brakes and raising legs!").
brakes on.
TOGGLE AG1.
printDivider("WARNING: TURN ON BALLON AUTO PITCH OR YOU WILL DIE").
printTimestamped("don't worry we figure that out! If your bouyancy is at -5600, your good!").
PRINT "Press any key to confirm you will die if you don't do that...".
WAIT .1.
WAIT UNTIL terminal:input:haschar.

printDivider("LOCKING CONTROLS").
SAS ON.
WAIT .2.
printTimestamped("SAS ON").
RCS ON.
WAIT .2.
printTimestamped("RCS ON").

printDivider("FILLING BALOONS").
//Find all baloons and fill them
for PART in SHIP:PARTS {
     if PART:HASMODULE("HLEnvelopePartModule") {
        print "part: " + part:name.
        PART:GETMODULE("HLEnvelopePartModule"):DOACTION("buoyancy max", true).
    }
}
//Dont fill this one as much to balance.
for PART in SHIP:PARTS {
    if PART:NAME = "hl10NoseCone" {
        // Loop to press the "buoyancy --" button 20 times
        set buttonPresses to 0.
        until buttonPresses >= 39 {
            PART:GETMODULE("HLEnvelopePartModule"):DOACTION("buoyancy --", true).
            print "buoyancy -- action triggered.".
            set buttonPresses to buttonPresses + 1.
            WAIT 0.1.  // Add a short delay to prevent rapid execution
        }

    }
}

// lock steering to heading(desiredHeading, desiredPitch) + R(0, 0, desiredRoll).
// Initialize variables
// printTimestamped("Setting desired heading to 90").
// set desiredHeading to 90.  // East, change this value as needed
// WAIT .1.
// // printTimestamped("Setting desired pitch to 0").
// // set desiredPitch to 0.    // Change this value as needed
// WAIT .1.

// printTimestamped("Setting desired roll to 0").
// set desiredRoll to 0.      // Level, change this value as needed
// WAIT .1.

// printTimestamped("Setting pitch threshold to 2").
// set pitchThreshold to 2.
// WAIT .1.

// printTimestamped("Setting KP to 0.1").
// set kp to 0.1.
// WAIT .1.

// printTimestamped("Setting KI to 0.1").
// set ki to 0.1.
// WAIT .1.

// printTimestamped("Setting KD to 0.05").
// set kd to 0.05.
// WAIT .1.

// printTimestamped("Initializing PID control variables").
// set latErrorPrev to 0.
// set timePrev to time:seconds.
// set runwayLongitude to ship:geoposition:lng. // Set this to the longitude of the center of the runway
wait 1.

printDivider("STARTING FAN SYSTEM!").
wait .1.
printTimestamped("Opening Intakes...").
wait .1.
printTimestamped("Starting Nuclear Reactor...").
wait .1.
printTimestamped("Powering up Fans...").
wait .1.
//does all that.
toggle ag2.
//reversefans
toggle ag4.
wait 2.



// Throttle up and release the brakes
printDivider("BACK UP SEQUENCE").
printTimestamped("Throttling up Reverse Fans!").
lock throttle to 1.
WAIT 2.4.
BRAKES off.
printTimestamped("Fan thrust is nominal").
printTimestamped("Brakes RELEASED!").
WAIT UNTIL SHIP:VELOCITY:SURFACE:MAG > 30.
printDivider("Firing Jet Engines!").
    TOGGLE AG4.
    TOGGLE AG3.


PRINT "There is " + ROUND(STAGE:LIQUIDFUEL) + " liquid fuel in this stage.".


// //
// print list(ship:geoposition:lat, ship:geoposition:lng).
// LIST of 2 items:
// [0] =
//   ["value"] = -0.0503427230378512
// [1] =
//   ["value"] = -74.5040000894026
// 47h
