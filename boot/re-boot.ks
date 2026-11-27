// Spaceplane Auto-Recovery Boot Script
// This script automatically restarts the launch sequence if the craft crashes or reverts

CLEARSCREEN.
PRINT "Spaceplane Auto-Recovery System".
PRINT "================================".
PRINT "".

// Wait for game to fully load
WAIT 2.

// Check if we're in a valid flight state
IF SHIP:STATUS = "LANDED" OR SHIP:STATUS = "SPLASHED" OR SHIP:STATUS = "PRELAUNCH" {
    PRINT "Craft detected on surface - starting launch sequence...".
    WAIT 1.
    
    // Run the main launch script from Airship Operating System folder
    RUNPATH("/aos/launch.ks").
    
} ELSE IF SHIP:STATUS = "FLYING" {
    PRINT "Craft is already flying - monitoring for crashes...".
    
    // Monitor flight and restart if crashed
    WHEN TRUE THEN {
        IF SHIP:STATUS = "LANDED" OR SHIP:STATUS = "SPLASHED" {
            PRINT "".
            PRINT "CRASH DETECTED!".
            PRINT "Reverting to launch in 3 seconds...".
            WAIT 3.
            
            // Revert to launch
            KUNIVERSE:REVERTTOLAUNCH().
            PRESERVE.
        }
        
        // Check every second
        WAIT 1.
        PRESERVE.
    }
    
} ELSE {
    PRINT "Unknown flight status: " + SHIP:STATUS.
    PRINT "Attempting to start launch anyway...".
    WAIT 2.
    RUNPATH("/aos/launch.ks").
}

PRINT "Boot script complete - monitoring active.".