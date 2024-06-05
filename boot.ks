
// Function to format time
// function formatTime {
//     parameter t.
//     set h to floor(t / 3600).
//     set m to floor(MOD(t, 3600) / 60).
//     set s to round(MOD(t, 60)).
//     return (h:toString():padleft(2,"0") + ":" + m:toString():padleft(2,"0") + ":" + s:toString():padleft(2,"0")).
// }
// WE DID IT REDDIT!
//https://www.reddit.com/r/Kos/comments/31r8hj/time_and_easiest_way_to_produce_a_timespan_from/

// left-pad with zeroes. Assumes you want a length of 2 if not specified
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
//Ship Loaded fully.
wait until ship:unpacked.
//Open in game terminal.
CORE:PART:GETMODULE("kOSProcessor"):DOEVENT("Open Terminal").

// wait until rcs.
// rcs off.
// core:part:getmodule("kOSProcessor"):doevent("Open Terminal").

set startTime to time:seconds.
PRINT"===============================".
PRINT"".
PRINT"" + SHIP:NAME + " booting up...".
PRINT"".
PRINT"===============================".
PRINT"".
// PRINT "Press any key to continue...".
// WAIT .2.
// WAIT UNTIL terminal:input:haschar.

CLEARSCREEN.
function boot {
    parameter extradots.
    parameter startTime.
    CLEARSCREEN.
    PRINT "===============================".
    PRINT "         BOOT SEQUENCE         ".
    PRINT "===============================".
    PRINT"       _..--=--..._".
    PRINT"    .-'            '-.  .-.".
    PRINT"   /.'   BLIMPS!    '.\/  /".
    PRINT"  |=-                -=| (".
    PRINT"   \'.              .'/\  \".
    PRINT"    '-.,_____ _____.-'  '-'".
    PRINT"         [_____]=8".
    PRINT "Welcome to the Airship Operating System!".
    PRINT "The time is " + formatUNI.
    PRINT "Mission clock: " + formatMET.
    Print "CPU time is: " + (startTime - time:seconds).
    PRINT "Initializing systems." + extradots.
    }
set extradots to "".
set counter to 0.
until counter > 8 {
    hudtext("loading" + counter * 10.2 + "%", 2, 2, 50, yellow, false).
    set extradots to extradots + ".".
    boot(extradots, startTime).
    wait.2.
    set counter to counter + 1.
}

PRINT "Done!".
WAIT.1.
clearScreen.


// Function to print a section divider
function printDivider {
    parameter title.
    print "==========================================".
    print "= " + title.
    print "==========================================".
}
SET terminal:height TO 70.
SET terminal:width TO 200.


function printBlimp {
PRINT"░░████████████████████████                                                                                                                      ".
PRINT"▒▒      ░░░░░░░░        ░░██░░                                                                                                                  ".
PRINT"▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓██▒▒░░░░▒▒▒▒                                                                                                                ".
PRINT"  ▓▓▓▓▓▓▓▓▓▓▒▒▒▒▓▓██▓▓▓▓▓▓░░░░░░▓▓▓▓░░░░▒▒▒▒▒▒    ▒▒▒▒▒▒░░░░░░░░░░  ░░░░░░▒▒░░░░░░▒▒▒▒████░░                                                    ".
PRINT"    ██▓▓▓▓██████████████▓▓░░▓▓▒▒▓▓▒▒      ▒▒▒▒░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░▒▒░░░░░░░░░░░░░░░░  ▓▓▓▓                                            ".
PRINT"    ░░▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒░░░░▓▓▒▒▒▒    ░░▒▒░░░░░░░░░░▒▒░░░░░░░░░░░░██░░░░░░░░░░▒▒▒▒░░░░░░░░▒▒░░░░░░░░░░▒▒▓▓▒▒░░                                    ".
PRINT"        ▓▓▓▓████▓▓▓▓░░░░▒▒▒▒    ░░░░░░░░░░░░░░░░░░░░▒▒░░░░░░░░░░░░░░░░░░░░░░░░░░  ░░░░░░░░░░░░░░░░░░░░▒▒░░░░░░▒▒                                ".
PRINT"        ▒▒░░░░░░░░░░░░▓▓▒▒    ▓▓░░░░░░▒▒░░░░░░░░░░░░▒▒▓▓░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░▓▓▓▓░░░░░░▒▒░░                            ".
PRINT"                  ▒▒▒▒▒▒    ▒▒▒▒░░░░░░░░▒▒░░░░░░▒▒▒▒▒▒▒▒▒▒▒▒▓▓░░░░░░░░░░░░░░░░░░▒▒▒▒░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░▒▒▒▒                        ".
PRINT"                  ░░▒▒▓▓  ▒▒░░▓▓▒▒░░░░░░▓▓████████████▓▓▓▓░░░░▒▒▓▓░░░░░░░░░░░░░░▓▓▒▒░░░░░░░░░░░░░░░░░░░░▓▓▒▒░░░░▒▒░░░░░░▒▒                      ".
PRINT"                      ░░  ░░░░░░▒▒░░░░░░██████████████████████░░░░░░▓▓░░░░░░░░░░░░▒▒░░░░░░░░░░░░░░░░░░░░▓▓▒▒░░░░░░░░░░░░░░▓▓                    ".
PRINT"                    ▒▒  ▒▒░░░░░░▒▒▒▒░░░░░░▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓████▓▓▓▓░░░░▒▒░░░░░░░░░░▒▒░░░░░░░░░░░░░░░░░░░░░░██░░░░░░░░░░░░░░░░▒▒                  ".
PRINT"                    ▒▒░░░░░░░░░░▒▒▒▒░░░░░░▓▓▒▒▒▒▒▒▒▒▒▒▒▒▓▓▒▒▒▒▒▒▒▒▒▒░░░░░░░░░░░░░░▒▒▒▒░░░░░░░░░░░░░░░░░░░░▒▒░░░░░░░░░░░░░░░░░░░░                ".
PRINT"                    ▒▒██░░░░░░░░░░████▓▓▓▓██▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒░░░░████▓▓████▓▓░░░░░░░░░░░░░░░░░░░░▒▒░░░░░░░░░░░░░░░░░░▓▓                ".
PRINT"                    ░░▒▒░░░░░░░░░░▓▓██████▒▒▓▓▓▓▓▓██▓▓██▒▒▒▒▓▓██▓▓▒▒▓▓▒▒▓▓▓▓▓▓██▒▒▓▓▓▓░░░░░░░░░░░░░░░░░░░░▒▒▒▒░░░░░░░░░░░░░░▒▒▒▒▒▒              ".
PRINT"                  ██▒▒▓▓░░░░░░░░░░▓▓██░░░░░░░░░░▒▒░░▒▒▒▒██▓▓▒▒▒▒▒▒▒▒▒▒▒▒░░░░░░░░░░▓▓▓▓░░░░░░░░░░░░░░░░░░░░▒▒░░░░░░░░░░░░░░░░▒▒▒▒▓▓              ".
PRINT"                    ░░▒▒░░░░░░░░░░░░░░░░░░▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒░░░░░░░░░░░░░░░░░░░░░░░░░░░░▒▒░░░░░░░░░░░░░░░░▒▒▒▒▒▒░░              ".
PRINT"                    ▓▓██▓▓██████▒▒▓▓██▒▒▓▓▓▓▓▓▒▒▓▓██▒▒▓▓██▓▓▒▒▓▓▓▓████▒▒████████▒▒▓▓██▒▒▒▒▒▒▓▓████░░████▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▒▒██              ".
PRINT"                ░░  ▓▓▓▓░░░░░░░░░░▓▓▓▓░░░░░░░░░░░░░░░░░░▓▓██░░░░░░░░░░░░░░░░░░░░░░▓▓▓▓░░░░░░░░░░░░░░░░░░░░▒▒░░░░░░░░░░░░░░░░▒▒▒▒██              ".
PRINT"                ██▓▓▓▓▓▓░░░░░░░░░░▓▓▓▓░░░░░░░░░░░░░░░░░░██▒▒░░░░░░░░░░░░░░░░░░░░░░████░░░░░░░░░░░░░░░░░░░░░░▒▒░░░░░░░░░░░░░░░░▓▓██              ".
PRINT"                ▒▒▓▓▓▓▓▓░░░░░░░░▒▒▒▒▒▒░░░░░░░░░░░░░░░░░░▓▓▓▓░░░░░░░░░░░░░░░░░░░░░░▒▒██░░░░░░░░░░░░░░░░░░░░▒▒░░░░░░░░░░░░░░░░░░▓▓                ".
PRINT"                ▒▒▒▒▓▓░░░░░░░░░░▒▒▒▒░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░▒▒░░░░░░░░░░░░░░░░░░░░░░▒▒░░░░░░░░░░░░░░░░▒▒▓▓                ".
PRINT"                        ▓▓░░░░░░▓▓▓▓░░░░░░░░░░░░░░░░░░░░▒▒░░░░░░░░░░░░░░░░░░░░░░░░▒▒░░░░░░░░░░░░░░░░░░░░░░▒▒░░░░░░░░░░░░░░░░▒▒                  ".
PRINT"                        ▒▒▒▒░░░░▒▒░░░░░░░░░░░░░░░░░░░░▓▓▓▓░░░░░░░░░░░░░░░░░░░░░░▓▓▒▒░░░░░░░░░░░░░░░░░░░░▓▓▒▒░░░░░░░░░░░░░░░░██                  ".
PRINT"                        ▒▒▒▒░░▒▒░░░░░░░░░░░░░░░░░░░░░░▒▒░░░░░░░░░░░░░░░░░░░░░░░░██▒▒░░░░░░░░░░░░░░░░░░░░▓▓▒▒░░▒▒██▒▒░░▒▒░░▓▓                    ".
PRINT"                      ▒▒▒▒▒▒▓▓▒▒░░░░░░░░▒▒░░░░░░░░░░▒▒▒▒░░░░░░░░░░░░░░░░░░░░░░░░▒▒░░░░░░░░░░░░░░░░░░░░░░▒▒░░░░▒▒▒▒▒▒░░  ░░                      ".
PRINT"                    ░░░░░░▒▒▒▒▒▒▒▒░░░░▒▒██░░░░░░░░░░▒▒▒▒░░░░░░░░░░░░░░░░░░░░░░▒▒▓▓░░░░░░░░░░░░░░░░░░░░▒▒▒▒░░░░██▒▒      ▓▓                      ".
PRINT"                  ░░░░░░░░▒▒▒▒▒▒▓▓▓▓▒▒▓▓░░▒▒░░░░░░▒▒▒▒░░░░░░░░░░░░░░░░░░░░░░░░▒▒░░░░░░░░▓▓▓▓▒▒░░░░░░▓▓▓▓░░▒▒██        ▓▓                        ".
PRINT"              ▒▒▒▒░░▒▒▓▓▓▓░░▓▓░░▒▒▓▓▓▓▓▓▒▒░░▒▒░░▒▒▒▒░░░░░░░░░░████░░░░░░░░░░▓▓░░░░░░░░▒▒▓▓▓▓░░░░░░░░▓▓██▒▒  ▒▒      ▓▓                          ".
PRINT"        ░░▓▓░░░░░░░░░░░░░░░░▓▓░░▓▓██░░      ░░██▓▓░░░░░░░░▒▒▒▒░░▓▓░░░░░░░░▓▓▒▒░░░░░░▓▓▒▒░░▓▓▓▓▓▓░░░░  ░░▓▓░░          ██                        ".
PRINT"              ▓▓██▓▓▓▓██▓▓██  ▓▓  ▒▒          ░░░░    ▓▓██▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓████▓▓██░░        ▒▒            ████░░██░░░░██                        ".
PRINT"                                ░░              ▓▓  ▒▒            ░░      ▓▓  ▒▒            ░░      ██          ████▓▓██                        ".
PRINT"                              ██▒▒                ██              ▓▓    ▓▓  ▒▒                          ▒▒      ▒▒▓▓                            ".
PRINT"                              ▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓                ▒▒  ▒▒                  ▒▒            ░░▒▒▒▒██                            ".
PRINT"                              ▓▓▓▓▓▓▓▓▓▓▓▓████▓▓▓▓██▓▓▓▓            ████                        ▒▒  ▒▒░░  ▒▒  ░░░░░░░░                          ".
PRINT"                              ▓▓▓▓▓▓▓▓████▓▓██▓▓▓▓▓▓████████████████████                        ▒▒  ▒▒  ░░    ░░▒▒▒▒▒▒░░░░░░░░░░░░░░░░░░░░      ".
PRINT"                            ░░██▓▓▓▓▓▓██▓▓▓▓▓▓██▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓                        ▒▒██░░          ▒▒░░░░▒▒▒▒▒▒░░░░▒▒░░░░░░░░    ".
PRINT"                          ░░░░▓▓▒▒░░▓▓████████████████▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓  ░░  ▒▒░░▓▓▓▓▓▓░░▓▓  ██▒▒            ░░░░▓▓▒▒▒▒▒▒▒▒▒▒▒▒░░░░▒▒░░░░  ".
PRINT"                          ░░▓▓▓▓▓▓▓▓████▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓            ░░▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒  ".
PRINT"                          ░░▓▓▓▓██████▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓██▓▓▓▓██▓▓▓▓██▓▓                ░░░░▒▒░░░░▒▒░░░░▒▒░░░░▒▒░░░░  ".
PRINT"                        ▓▓▒▒▒▒██░░░░░░░░░░▒▒▒▒▒▒██████████████████████████████████████████▒▒                        ▒▒░░▒▒░░░░░░░░▒▒░░░░░░▒▒    ".
PRINT"                          ▓▓▒▒██░░░░░░░░▒▒▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓░░                        ░░▒▒▒▒▒▒  ▒▒▒▒▒▒▒▒▒▒▒▒░░░░░░    ".
PRINT"                          ░░▒▒▓▓░░░░░░▒▒▒▒████▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓██▓▓▓▓░░                              ░░▓▓▓▓▓▓    ░░    ░░░░      ".
PRINT"                          ░░░░▓▓▓▓░░▒▒      ░░████████████████████████████▒▒▒▒░░▒▒▒▒▒▒                              ▒▒▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓        ".
PRINT"                            ▒▒▒▒██▒▒▒▒        ░░▓▓▓▓▓▓▓▓██▓▓████████▓▓▓▓░░▒▒░░░░░░░░▒▒░░                            ▒▒░░▒▒▓▓▓▓▓▓▓▓▓▓            ".
PRINT"                                                ▓▓████████████████████▓▓    ▒▒▓▓░░░░░░░░▒▒▒▒▒▒                            ░░▒▒▒▒  ░░░░          ".



}

function printWelcome {
    clearScreen.
    printBlimp().
    print "======================================================================".
    print "===============           WELCOME TO AOS               ===============".
    print "===============       AIRSHIP OPERATING SYSTEM         ===============".
    print "======================================================================".
    
}

printWelcome().


// printDivider("LOADING FUNCTIONS").
// RUNPATH("functions.ks").
// printTimestamped("Loaded functions from functions.ks").
wait 2.


function printShipStatus {
    print "==========================================".
    print "=             SHIP STATUS                =".
    print "==========================================".
    print "Crew: " + SHIP:CREW:LENGTH + " / " + SHIP:crewcapacity.
    print SHIP:CREW().
    print SHIP.CONNECTION.
    print "Dry Mass: " + ROUND(SHIP:drymass).
    print "Wet Mass: " + ROUND(SHIP:wetmass).
    print "Actual Mass: " + ROUND(SHIP:mass).
    // print "Resources: " + SHIP:resources.
    print "==========================================".
}
printShipStatus().
wait .5.
PRINT "===============================".
PRINT "         BOOT COMPLETE         ".
PRINT "===============================".
PRINT "Welcome to the Blimp Control System!".
PRINT "The time is " + formatUNI.
PRINT "Mission clock: " + formatMET.
Print "CPU time is T+" + (-1*(startTime - time:seconds)) + " seconds".
PRINT "===============================".
wait .1.
PRINT "Switching to Archive 0...".
wait .1.
switch TO 0.
LIST FILES.
// Start the main script
// printDivider("RUNNING MAIN SCRIPT").
// printTimestamped("Running main script...").
// // RUNPATH("main.ks").

// printDivider("END OF BOOT SCRIPT").
// printTimestamped("Boot script completed.").
// Print "Boot Took:" + (startTime - time:seconds) + "Seconds".

SET terminal:height TO 22.
SET terminal:width TO 49.