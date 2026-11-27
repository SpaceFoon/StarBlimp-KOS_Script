// boot_my_ui.ks
// Robust probe-core boot script for My UI
// - prints debug info
// - checks for my_ui.ks on current volume, then on archive 1
// - copies from 1:/ to 0:/ if needed
// - lists files and runs my_ui.ks

CLEARSCREEN.
PRINT "=== boot_my_ui.ks starting ===".
PRINT "Processor: " + CORE:PART:NAME.
PRINT "Time: " + TIME:CALENDAR.

PRINT "Listing files on current volume:".
LIST FILES.

SET can_run TO TRUE.
SET os_path TO "".

// Preferred My UI locations (highest priority first)
SET os_candidates TO LIST().
os_candidates:ADD("my_ui.ks").
os_candidates:ADD("0:/my_ui.ks").
os_candidates:ADD("1:/my_ui.ks").

FOR candidate IN os_candidates {
  IF EXISTS(candidate) {
    SET os_path TO candidate.
    BREAK.
  }
}

IF os_path = "" {
  PRINT "Preferred My UI files not present on current volume.".
  IF EXISTS("1:/my_ui.ks") {
    PRINT "Found my_ui.ks on archive 1, copying to local volume 0:/...".
    COPY "1:/my_ui.ks" TO "0:/my_ui.ks".
    PRINT "Copy complete.".
    SET os_path TO "0:/my_ui.ks".
  } ELSE {
    PRINT "No My UI file found. Place my_ui.ks on an accessible volume.".
    PRINT "Final file listing:".
    LIST FILES.
    SET can_run TO FALSE.
  }
} ELSE {
  PRINT "Selected My UI file: " + os_path.
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
  PRINT "Final check skipped; no My UI file selected.".
}

// Attempt to run My UI if allowed
IF can_run {
  PRINT "Attempting to run My UI...".
  IF os_path <> "" {
    RUNPATH(os_path).
    // If RUN returns for any reason, print that it ended
    PRINT "boot_my_ui.ks: " + os_path + " ended or returned.".
  } ELSE {
    PRINT "boot_my_ui.ks: No My UI path available.".
  }
} ELSE {
  PRINT "boot_my_ui.ks: My UI not available; aborting boot.".
}

CLEARSCREEN.
PRINT "=== boot_my_ui.ks finished ===".