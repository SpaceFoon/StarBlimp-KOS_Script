// bootbaker.ks
// Robust probe-core boot script for Baker OS
// - prints debug info
// - checks for baker_os.ks on current volume, then on archive 1
// - copies from 1:/ to 0:/ if needed
// - lists files and runs baker_os.ks

CLEARSCREEN.
PRINT "=== bootbaker.ks starting ===".
PRINT "Processor: " + CORE:PART:NAME.
PRINT "Time: " + TIME:CALENDAR.

PRINT "Listing files on current volume:".
LIST FILES.

SET can_run TO TRUE.
SET os_path TO "".

// Preferred Baker OS locations (highest priority first)
SET os_candidates TO LIST().
os_candidates:ADD("Baker Operating System/Baker2.2.0.ks").
os_candidates:ADD("0:/Baker Operating System/Baker2.2.0.ks").
os_candidates:ADD("1:/Baker Operating System/Baker2.2.0.ks").
os_candidates:ADD("Baker2.2.0.ks").
os_candidates:ADD("0:/Baker2.2.0.ks").
os_candidates:ADD("1:/Baker2.2.0.ks").
os_candidates:ADD("baker_os.ks").
os_candidates:ADD("0:/baker_os.ks").
os_candidates:ADD("1:/baker_os.ks").

FOR candidate IN os_candidates {
  IF EXISTS(candidate) {
    SET os_path TO candidate.
    BREAK.
  }
}

IF os_path = "" {
  PRINT "Preferred Baker OS files not present on current volume.".
  IF EXISTS("1:/baker_os.ks") {
    PRINT "Found baker_os.ks on archive 1, copying to local volume 0:/...".
    COPY "1:/baker_os.ks" TO "0:/baker_os.ks".
    PRINT "Copy complete.".
    SET os_path TO "0:/baker_os.ks".
  } ELSE {
    PRINT "No Baker OS file found. Place Baker2.2.0.ks or baker_os.ks on an accessible volume.".
    PRINT "Final file listing:".
    LIST FILES.
    SET can_run TO FALSE.
  }
} ELSE {
  PRINT "Selected Baker OS file: " + os_path.
}

PRINT "Final file listing (post-copy):".
LIST FILES.

IF os_path <> "" {
  IF EXISTS(os_path) {
    PRINT "Final check - does " + os_path + " exist? True".
  } ELSE {
    PRINT "Final check - does " + os_path + " exist? False".
  }
} ELSE {
  PRINT "Final check skipped; no Baker OS file selected.".
}

// Attempt to run Baker OS if allowed
IF can_run {
  PRINT "Attempting to run Baker OS...".
  IF os_path <> "" {
    RUNPATH(os_path).
    // If RUN returns for any reason, print that it ended
    PRINT "bootbaker.ks: " + os_path + " ended or returned.".
  } ELSE {
    PRINT "bootbaker.ks: No Baker OS path available.".
  }
} ELSE {
  PRINT "bootbaker.ks: Baker OS not available; aborting boot.".
}

CLEARSCREEN.
PRINT "=== bootbaker.ks finished ===".
