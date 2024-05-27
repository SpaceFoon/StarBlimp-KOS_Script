
// Function to format time
// function formatTime {
//     parameter t.
//     set h to floor(t / 3600).
//     set m to floor(MOD(t, 3600) / 60).
//     set s to round(MOD(t, 60)).
//     return (h:toString():padleft(2,"0") + ":" + m:toString():padleft(2,"0") + ":" + s:toString():padleft(2,"0")).
// }
RUNPATH("../time.ks").

CLEARSCREEN.
PRINT "===============================".
PRINT "         BOOT SEQUENCE         ".
PRINT "===============================".
PRINT "Welcome to the Blimp Control System!".
PRINT "Initializing systems.".
WAIT .2.
PRINT "Initializing systems..".
WAIT .2.
PRINT "Initializing systems...".
WAIT .2.
PRINT "Initializing systems.".
WAIT .2.
PRINT "Initializing systems..".
WAIT .2.
PRINT "Initializing systems...".
WAIT .2.
PRINT "Done!".
WAIT .2.
// Function to print a section divider
function printDivider {
    parameter title.
    print "==========================================".
    print "= " + title.
    print "==========================================".
}

// Function to print a formatted message with a timestamp
function printTimestamped {
    parameter msg.
    set elapsedTime to time:seconds - startTime.
    print "[" + formatTime(elapsedTime) + "] " + msg.
}

// Function to print the blimp ASCII art
function printBlimp {
    print "       _..--=--..._ ".
    print "    .-'            '-.  .-. ".
    print "   /.'   BLIMPS!    '.\\/  / ".
    print "  |=-                -=| ( ".
    print "   \\'.              .'/\\  \\ ".
    print "    '-.,_____ _____.-'  '-' ".
    print "         [_____]=8) ".
}

// Function to print the welcome screen
function printWelcome {
    clearScreen.
    print "==========================================".
    print "=              WELCOME TO                =".
    print "=         THE AIRSHIP CONTROL SYSTEM      =".
    print "==========================================".
    printBlimp().
    print "==========================================".
    print "=              SYSTEM INFO               =".
    print "==========================================".
}

// Function to print ship status
function printShipStatus {
    print "==========================================".
    print "=             SHIP STATUS                =".
    print "==========================================".
    print "Crew: " + ship:crewcount.
    print "Fuel: " + round(ship:resources:amount("LiquidFuel")) + " / " + round(ship:resources:max("LiquidFuel")).
    print "Oxidizer: " + round(ship:resources:amount("Oxidizer")) + " / " + round(ship:resources:max("Oxidizer")).
    print "Food: " + round(ship:resources:amount("Supplies")) + " / " + round(ship:resources:max("Supplies")).
    print "==========================================".
}

// Start the timer
set startTime to time:seconds.

// Print booting message
printWelcome().
wait 2.

// Print initializing message
printDivider("INITIALIZING VARIABLES").
printTimestamped("Starting initialization...").
wait 1.

// Initialize variables
printTimestamped("Setting desired heading to 90").
set desiredHeading to 90.  // East, change this value as needed

printTimestamped("Setting desired pitch to 10").
set desiredPitch to 10.    // Change this value as needed

printTimestamped("Setting desired roll to 0").
set desiredRoll to 0.      // Level, change this value as needed

printTimestamped("Setting pitch threshold to 2").
set pitchThreshold to 2.

printTimestamped("Setting KP to 0.1").
set kp to 0.1.

printTimestamped("Setting KD to 0.05").
set kd to 0.05.

printTimestamped("Initializing PD control variables").
set latErrorPrev to 0.
set timePrev to time:seconds.
set runwayLongitude to ship:geoposition:longitude. // Set this to the longitude of the center of the runway
wait 1.

// Load functions from another file
printDivider("LOADING FUNCTIONS").
// RUNPATH("functions.ks").
printTimestamped("Loaded functions from functions.ks").
wait 1.

// Print ship status
printShipStatus().
wait 3.

// Start the main script
printDivider("RUNNING MAIN SCRIPT").
printTimestamped("Running main script...").
// RUNPATH("main.ks").

printTimestamped("Boot script completed.").
printDivider("END OF BOOT SCRIPT").

// Define desired heading, pitch, and roll
set desiredHeading to 90.  // East, change this value as needed
set desiredPitch to 10.    // Change this value as needed
set desiredRoll to 0.      // Level, change this value as needed

// set elapsedTime to time:seconds - startTime.
// print "[" + formatTime(elapsedTime) + "] Boot completed.".
