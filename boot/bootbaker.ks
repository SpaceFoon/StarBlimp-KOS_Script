// Baker OS Boot Script for Probe Core
// Set this as the boot file for your probe core processor

CLEARSCREEN.
PRINT "=================================".
PRINT "    BAKER OS BOOT                ".
PRINT "=================================".

// Wait for ship to be fully loaded
WAIT UNTIL SHIP:UNPACKED.

// Open terminal for this processor
CORE:PART:GETMODULE("kOSProcessor"):DOEVENT("Open Terminal").

PRINT "Probe Core: " + CORE:PART:NAME.
PRINT "Processor ID: " + CORE:PART:UID.
PRINT "Current Volume: " + CORE:VOLUME:NAME.
PRINT "Loading Baker OS...".

// Debug: Show current files
PRINT "Current files on volume " + CORE:VOLUME:NAME + ":".
LIST FILES.
PRINT "".

// Copy Baker OS from archive if needed
PRINT "Checking for Baker OS...".
IF EXISTS("1:/baker_os.ks") {
    COPYPATH("1:/baker_os.ks", "0:/baker_os.ks").
    PRINT "Baker OS copied to local storage.".
} ELSE IF EXISTS("baker_os.ks") {
    PRINT "Baker OS found in local storage.".
} ELSE {
    PRINT "WARNING: Baker OS not found!".
    PRINT "Looking for baker_os.ks in:".
    PRINT "- Scripts folder (1:/baker_os.ks)".
    PRINT "- Local storage (0:/baker_os.ks)".
    PRINT "".
    PRINT "Available files:".
    LIST FILES.
    PRINT "".
    PRINT "Waiting for Baker OS to be available...".
    WAIT UNTIL EXISTS("baker_os.ks") OR EXISTS("1:/baker_os.ks").
}

// Switch to archive for consistent file access
PRINT "Switching to archive volume 0...".
SWITCH TO 0.
PRINT "Now on volume: " + CORE:VOLUME:NAME.

PRINT "Files on archive:".
LIST FILES.
PRINT "".

PRINT "Starting Baker OS...".
PRINT "=================================".

// Run Baker OS
PRINT "Final check - does baker_os.ks exist? " + EXISTS("baker_os.ks").
IF EXISTS("baker_os.ks") {
    PRINT "SUCCESS: Running baker_os.ks".
    RUN baker_os.ks.
} ELSE {
    PRINT "ERROR: Could not start Baker OS".
    PRINT "Script not found after copy operation".
    PRINT "Final file listing:".
    LIST FILES.
}
