// UI-related functions for launch script

// Global variables for non-blocking intro sequence state
// GLOBAL INTRO_ACTIVE IS FALSE.
// GLOBAL INTRO_SEQUENCE_NUM IS 1.
// GLOBAL INTRO_EVENT_INDEX IS 0.
// GLOBAL INTRO_DECORATION_INDEX IS 0.
// GLOBAL INTRO_NEXT_UPDATE IS 0.
// GLOBAL INTRO_PHASE_DELAY IS 0.

IF NOT EXISTS("LOG_DECORATIONS") {
  GLOBAL LOG_DECORATIONS IS LIST(">>>", "***", "===", "~~~", "^^^", "+++", "|||", ":::").
}.

IF NOT EXISTS("LOG_ANIMATION_INDEX") {
  GLOBAL LOG_ANIMATION_INDEX IS -1.
}.

FUNCTION NEXT_LOG_DECORATION {
  IF LOG_DECORATIONS:LENGTH = 0 {
    RETURN "".
  }.

  SET LOG_ANIMATION_INDEX TO LOG_ANIMATION_INDEX + 1.
  IF LOG_ANIMATION_INDEX >= LOG_DECORATIONS:LENGTH {
    SET LOG_ANIMATION_INDEX TO 0.
  }.
  RETURN LOG_DECORATIONS[LOG_ANIMATION_INDEX].
}

// Legacy wrappers for marquee helpers so older code paths still work.
FUNCTION GET_AG_STATE {
  PARAMETER ag.
  RETURN AG_IS_ACTIVE(ag).
}

FUNCTION SET_AG_STATE {
  PARAMETER ag, desired.
  AG_SET_STATE(ag, desired).
}

FUNCTION TOGGLE_AG {
  PARAMETER ag.
  AG_TOGGLE(ag).
}

FUNCTION LOG_FORMAT_LINE {
  PARAMETER level, message, hiLight IS FALSE.
  IF hiLight {
    TERMINAL:REVERSE.
  }
  PRINT "[" + formatMET + "]".
  PRINT level + " " + message.
  IF hiLight {
    WAIT 0.05.
    TERMINAL:REVERSE.
  }
}

FUNCTION LOG_INFO {
  PARAMETER message.
  LOG_FORMAT_LINE("[INFO]", message).
}

FUNCTION LOG_WARN {
  PARAMETER message.
  LOG_FORMAT_LINE("[WARN]", message, TRUE).
}

FUNCTION LOG_ERROR {
  PARAMETER message.
  LOG_FORMAT_LINE("[ERROR]", message, TRUE).
}

FUNCTION LOG_STATUS {
  PARAMETER label, value, unit IS "".
  LOCAL suffix IS unit.
  IF suffix <> "" {
    SET suffix TO " " + suffix.
  }
  LOG_FORMAT_LINE("[STATUS]", label + ": " + value + suffix).
}

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
    + padZ(ts:DAY - 1,3) + " " // subtracts 1 to get days elapsed, not day of year. 
    + padZ(ts:HOUR) + ":"
    + padZ(ts:MINUTE) + ":"
    + padZ(ROUND(ts:SECOND))+ "]".
}
// PRINT formatMET.

FUNCTION formatUNI
{
  LOCAL uni_ts IS TIME.
  RETURN "[Y" 
    + ROUND(uni_ts:YEAR) + ", D"
    + padZ(uni_ts:DAY) + ", "
    + padZ(uni_ts:HOUR) + ":"
    + padZ(uni_ts:MINUTE) + ":"
    + padZ(ROUND(uni_ts:SECOND))+ "]".
}



FUNCTION START_INTRO_SEQUENCE {
  PARAMETER sequence_num.
  SET INTRO_ACTIVE TO TRUE.
  SET INTRO_SEQUENCE_NUM TO sequence_num.
  SET INTRO_EVENT_INDEX TO 0.
  SET INTRO_DECORATION_INDEX TO 0.
  SET INTRO_NEXT_UPDATE TO TIME:SECONDS + 0.25.
  SET INTRO_PHASE_DELAY TO TIME:SECONDS + 0.35.
}

FUNCTION UPDATE_INTRO_SEQUENCE {
  IF NOT INTRO_ACTIVE { RETURN. }
  IF TIME:SECONDS < INTRO_NEXT_UPDATE { RETURN. }
  
  LOCAL decorations IS LIST().
  LOCAL events IS LIST().
  LOCAL title IS "".
  LOCAL dec_symbol IS "".
  
  IF INTRO_SEQUENCE_NUM = 1 {
    SET decorations TO LIST("").
    SET events TO LIST(
      LIST("Helium manifold cracked open", "Lift cells begin to hiss."),
      LIST("Primary balloons inflating", "Envelope sway within tolerance."),
      LIST("Ballast valves trimming", "Target buoyancy looks great."),
      LIST("Landing legs deploying", "Hydraulics green across the board."),
      LIST("Brakes confirmed on", "Hold position while fans spin up.")
    ).
    SET title TO "[PRE-FLIGHT] Systems Coming Alive".
  } ELSE IF INTRO_SEQUENCE_NUM = 2 {
    SET decorations TO LIST("").
    SET events TO LIST(
      LIST("Hands off the controls", "Autopilot taking authority."),
      LIST("SAS and RCS guarded", "Manual toggle overrides disabled."),
      LIST("Control locks engaged", "Stick, throttle and stage levers frozen."),
      LIST("Flight computer synced", "Guidance tables loaded and verified."),
      LIST("Pilot warning acknowledged", "Proceed when you're ready to trust us.")
    ).
    SET title TO "[WARNING] Do Not Interfere".
  } ELSE IF INTRO_SEQUENCE_NUM = 3 {
    SET decorations TO LIST("").
    SET events TO LIST(
      LIST("SAS loop calibrated", "Holding 90° heading reference."),
      LIST("Wheel steering constrained", "No joystick drift detected."),
      LIST("Control surfaces frozen", "HL-10 fins parked neutral."),
      LIST("Reaction wheels synced", "Torque bias trimmed."),
      LIST("RCS thrusters armed", "Feathering standby jets for later.")
    ).
    SET title TO "[SETUP] Flight Locks Engaging".
  } ELSE IF INTRO_SEQUENCE_NUM = 4 {
    SET decorations TO LIST("").
    SET events TO LIST(
      LIST("Starting fan system", "Propulsion system activation"),
      LIST("Crash monitor awake", "Recovery monitors online"),
      LIST("Air intakes breathing", "Sensors report clean flow"),
      LIST("Compressors ramping", "Boost pressure approaching target"),
      LIST("All systems green", "Ready for launch")
    ).
    SET title TO "[ENGINE] Propulsion Coming Online".
  }
  
  IF INTRO_EVENT_INDEX >= events:LENGTH {
    SET INTRO_ACTIVE TO FALSE.
    RETURN.
  }
  
  SET dec_symbol TO decorations[INTRO_DECORATION_INDEX].
  
  CLEARSCREEN.
  PRINT "============================================".
  PRINT "= " + formatMET.
  PRINT "= " + title + " " + dec_symbol.
  PRINT "============================================".
  
  FOR reveal_idx IN RANGE(INTRO_EVENT_INDEX + 1) {
    LOCAL event IS events[reveal_idx].
    PRINT dec_symbol + " " + event[0] + ".".
    IF event:LENGTH > 1 {
      PRINT "  " + event[1].
    }.
  }.
  
  LOCAL border IS "".
  FOR border_idx IN RANGE(12) {
    SET border TO border + "=" + dec_symbol.
  }
  PRINT border + "=".
  
  SET INTRO_DECORATION_INDEX TO INTRO_DECORATION_INDEX + 1.
  IF INTRO_DECORATION_INDEX >= decorations:LENGTH {
    SET INTRO_DECORATION_INDEX TO 0.
    SET INTRO_EVENT_INDEX TO INTRO_EVENT_INDEX + 1.
    SET INTRO_NEXT_UPDATE TO TIME:SECONDS + 0.35.
  } ELSE {
    SET INTRO_NEXT_UPDATE TO TIME:SECONDS + 0.25.
  }
}

FUNCTION INTRO_SEQUENCE_1 {
  START_INTRO_SEQUENCE(1).
}

FUNCTION INTRO_SEQUENCE_2 {
  START_INTRO_SEQUENCE(2).
}

FUNCTION INTRO_SEQUENCE_3 {
  START_INTRO_SEQUENCE(3).
}

FUNCTION INTRO_SEQUENCE_4 {
  START_INTRO_SEQUENCE(4).
}

FUNCTION LOG_STAGE {
    PARAMETER line1, line2.
    LOCAL decoration IS NEXT_LOG_DECORATION().
    PRINT"================================================================================".
PRINT"".
PRINT"" + SHIP:NAME + " SSTE Config".
PRINT"".
PRINT"================================================================================".
PRINT"".
WAIT 1.
    PRINT "============================================".
    PRINT "= " + formatMET.
    PRINT "= [STAGE] " + decoration + " " + line1.
    PRINT "= " + line2.
    PRINT "============================================".
}

FUNCTION LAUNCH_PRINT_DIVIDER {
    PARAMETER title.
    PRINT "============================================".
    PRINT "= " + formatMET.
    PRINT "= " + title.
    PRINT "============================================".
}

FUNCTION LAUNCH_PRINT_WELCOME {
    SET TERMINAL:HEIGHT TO 40.
    SET TERMINAL:WIDTH TO 80.
    CLEARSCREEN.
    PRINT "============================================================================".
    PRINT "====================              WELCOME TO THE            ================".
    PRINT "====================       AIRSHIP LAUNCH CONTROL SYSTEM    ================".
    PRINT "============================================================================".
    PRINT " ".
    PRINT"                                                               _..--=--..._  ".
    PRINT"                                                            .-'            '-".
    PRINT"                                                           /.'    Blimps    '".
    PRINT"                                                          |=-       in       ".
    PRINT"                                                           \'.   Spaaaaace! .".
    PRINT"                                                            '-.,_____ _____.-".
    PRINT"                                                                 [_____]=8   ".
    PRINT " ".
    WAIT .2.
        CLEARSCREEN.
    PRINT "============================================================================".
    PRINT "====================              WELCOME TO THE            ================".
    PRINT "====================       AIRSHIP LAUNCH CONTROL SYSTEM    ================".
    PRINT "============================================================================".
    PRINT " ".
    PRINT"                                                 _..--=--..._        ".
    PRINT"                                              .-'            '-.  .-.".
    PRINT"                                             /.'    Blimps    '.\/  /".
    PRINT"                                            |=-       in       -=| ( ".
    PRINT"                                             \'.   Spaaaaace! .'/\  \".
    PRINT"                                              '-.,_____ _____.-'  '-'".
    PRINT"                                                   [_____]=8         ".
    PRINT " ".
    WAIT .2.
        CLEARSCREEN.
    PRINT "============================================================================".
    PRINT "====================              WELCOME TO THE            ================".
    PRINT "====================       AIRSHIP LAUNCH CONTROL SYSTEM    ================".
    PRINT "============================================================================".
    PRINT " ".
    PRINT"                                     _..--=--..._        ".
    PRINT"                                  .-'            '-.  .-.".
    PRINT"                                 /.'    Blimps    '.\/  /".
    PRINT"                                |=-       in       -=| ( ".
    PRINT"                                 \'.   Spaaaaace! .'/\  \".
    PRINT"                                  '-.,_____ _____.-'  '-'".
    PRINT"                                       [_____]=8         ".
    PRINT " ".
    WAIT .2.
        CLEARSCREEN.
    PRINT "============================================================================".
    PRINT "====================              WELCOME TO THE            ================".
    PRINT "====================       AIRSHIP LAUNCH CONTROL SYSTEM    ================".
    PRINT "============================================================================".
    PRINT " ".
    PRINT"                         _..--=--..._        ".
    PRINT"                      .-'            '-.  .-.".
    PRINT"                     /.'    Blimps    '.\/  /".
    PRINT"                    |=-       in       -=| ( ".
    PRINT"                     \'.   Spaaaaace! .'/\  \".
    PRINT"                      '-.,_____ _____.-'  '-'".
    PRINT"                           [_____]=8         ".
    PRINT " ".
    WAIT .2.
        CLEARSCREEN.
    PRINT "============================================================================".
    PRINT "====================              WELCOME TO THE            ================".
    PRINT "====================       AIRSHIP LAUNCH CONTROL SYSTEM    ================".
    PRINT "============================================================================".
    PRINT " ".
    PRINT "            _..--=--..._        ".
    PRINT "         .-'            '-.  .-.".
    PRINT "        /.'    Blimps    '.\/  /".
    PRINT "       |=-       in       -=| ( ".
    PRINT "        \'.   Spaaaaace! .'/\  \".
    PRINT "         '-.,_____ _____.-'  '-'".
    PRINT "              [_____]=8         ".
    PRINT " ".
    WAIT .2.
        CLEARSCREEN.
    PRINT "============================================================================".
    PRINT "====================              WELCOME TO THE            ================".
    PRINT "====================       AIRSHIP LAUNCH CONTROL SYSTEM    ================".
    PRINT "============================================================================".
    PRINT " ".
    PRINT"..--=--..._        ".
    PRINT"           '-.  .-.".
    PRINT"  Blimps    '.\/  /".
    PRINT"    in       -=| ( ".
    PRINT" Spaaaaace! .'/\  \".
    PRINT"_____ _____.-'  '-'".
    PRINT" [_____]=8         ".
    PRINT " ".
    WAIT .2.
    PRINT "============================================================================".
    PRINT "====================              WELCOME TO THE            ================".
    PRINT "====================       AIRSHIP LAUNCH CONTROL SYSTEM    ================".
    PRINT "============================================================================".
    PRINT " ".
    PRINT"      ".
    PRINT".  .-.".
    PRINT".\/  /".
    PRINT"-=| ( ".
    PRINT"'/\  \".
    PRINT"'  '-'".
    PRINT"      ".
    PRINT " ".
    WAIT .2.
        CLEARSCREEN.
    PRINT "============================================================================".
    PRINT "====================    PROGRAM 1: SINGLE STAGE TO EVE      ================".
    PRINT "============================================================================".
    PRINT " .                        .       ___---___                    .           .".
    PRINT "                .              .--\        --.     .     .         .        ".
    PRINT "                             ./.;_.\     __/~ \.                            ".
    PRINT "    .                       /;  / `-'  __\    . \                           ".
    PRINT "                   .       / ,--'     / .   .;   \        |                 ".
    PRINT "                          | .|       /       __   |      -O-       .        ".
    PRINT "         .               |__/    __ |  . ;   \ | . |      |                 ".
    PRINT "                         |      /  \\_    . ;| \___|                 |      ".
    PRINT "            .    o       |      \  .~\\___,--'     |                -O-     ".
    PRINT "                          |     | . ; ~~~~\_    __|                  |      ".
    PRINT " .           |             \    \   .  .  ; \  /_/   .                      ".
    PRINT "            -O-        .    \   /         . |  ~/                  .        ".
    PRINT "             |    .          ~\ \   .      /  /~          o                 ".
    PRINT "           .                   ~--___ ; ___--~                             .".
    PRINT "                          .          ---         .                       -JT".
    PRINT "============================================================================".
    PRINT "====== Please sit back and relax while the computer takes you to Eve. ======".
    PRINT "===========  Call 1-800-PHONE-HOME for technical support ===================".
    PRINT "============================================================================".
    PRINT " ".
    // TERMINAL:REVERSE.
    // WAIT .2.
    // TERMINAL:REVERSE.
}

FUNCTION LAUNCH_JUMP_DUKE_BOYS{
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
    // WAIT 1.
}

FUNCTION wemadeit{
  SET TERMINAL:HEIGHT TO 94.
  SET TERMINAL:WIDTH TO 170.
PRINT "".
PRINT "⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣀⡀⠤⠠⠐⠒⠀⠉⣉⣉⠀⠁⠀⠀⠄⠤⠤⠬⠤⠭⢈⡉⠍⢁⠒⡒⠀⠤⠀⣀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀".
PRINT "⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⣀⠤⠐⠒⣋⢁⡀⢀⢲⣔⣂⣀⣒⣊⣴⣚⢁⡤⣍⣻⣛⣒⣺⠶⣬⣁⣤⣍⡋⠝⢒⠴⠭⢦⣀⡒⣤⣉⡐⠂⠤⢀⡀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀".
PRINT "⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⡀⠤⢒⠈⠀⣀⣤⣴⠿⠖⠯⠹⠛⠓⠒⠛⠛⢿⠿⣷⠾⢿⢾⡿⢯⣙⣑⣋⠛⣽⠛⢛⡩⣟⣟⣢⣽⣶⣦⣤⣀⣛⣏⠉⠛⡓⠦⢄⣁⡒⠠⢀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀".
PRINT "⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣀⠄⢂⣁⣴⠞⠒⣊⡝⡋⢁⣠⣠⣤⣴⣴⡴⣿⢲⡖⣮⣍⡛⠙⢺⡿⣷⡤⠶⣏⢿⣻⣿⠾⣞⣓⣛⣛⠻⠟⠛⠛⢿⣻⠛⣗⣩⣭⣷⡟⢻⡉⢛⠲⢤⣀⡈⠐⡠⢀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀".
PRINT "⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⡠⠐⣩⣴⢞⣩⣤⣍⣁⡐⡺⡆⠴⠟⠋⠉⠭⢛⠻⣿⣟⣿⣛⣿⡱⣯⡵⣶⢫⣿⡷⣍⣸⣟⣫⠭⠿⠛⠉⠉⠉⢁⠤⠤⢤⡆⣽⣿⢟⣛⣁⣁⣤⣄⣜⣿⣽⣶⣼⣿⣴⣭⣆⢈⠂⢄⡀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀".
PRINT "⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⡀⠔⣨⣴⣿⣯⠰⢟⣟⠭⡤⣲⣏⡔⠒⢂⣤⡦⣶⣶⢶⡔⠙⣿⣾⣽⣷⣿⣳⣿⣽⢿⡽⠖⠋⢁⣀⣤⣀⣔⣀⠤⢄⣿⣿⡟⣶⣦⣾⣿⠿⣮⣿⣯⣉⣿⣿⣿⣟⠋⠉⠉⠛⠿⣯⣿⣧⣝⣞⠯⢆⢄⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀".
PRINT "⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⠔⠊⠠⣼⣷⣿⣷⢿⣞⣯⢆⡍⣻⡿⣙⣳⣦⠾⣿⣿⣭⣷⣎⣄⣴⣿⣟⡿⣿⣿⡿⢿⣛⣭⡀⣠⣦⣵⣟⡿⣴⣿⠞⠃⠀⠹⣿⣿⣿⣿⣿⣹⣿⣿⣿⣿⣝⢼⢻⡿⣿⣷⣖⣤⣐⣄⢍⠻⣿⣾⣝⢿⣯⣜⡈⠢⡀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀".
PRINT "⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣀⠔⠁⢀⣤⣾⣿⣿⣯⢖⣋⣍⣼⣟⣩⡞⣿⣛⣿⣶⠿⢻⣿⡿⣯⡻⣝⠿⣞⡾⣽⣿⣭⠞⠨⣋⠁⢋⣿⣿⣯⣿⣿⣿⣿⣿⢶⣄⣦⣿⣿⣿⡉⢈⠓⣿⡁⠈⢉⠙⡏⣿⡿⡿⠋⠉⢽⣯⣿⢭⣧⣴⡽⣿⣿⣶⣯⣷⡆⠈⣤⢄⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀".
PRINT "⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⡠⠊⠀⣠⣴⣿⣿⣿⣿⡿⣱⣭⡿⣳⣿⣋⣩⣿⣟⣳⡳⡏⣁⡚⣩⣷⣳⢳⡯⣟⠺⠹⣞⣳⢶⣿⣶⣤⡾⣿⣿⢻⡗⣯⣿⣿⣿⡿⢾⣿⣿⣿⣿⣿⣿⣆⣽⣸⣄⣠⣿⣡⣷⣽⡅⠀⢠⠀⢀⣈⢻⣿⣷⣿⣽⣿⣿⣿⣿⣶⣼⣿⣹⣷⢕⢄⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀".
PRINT "⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠞⠀⠐⣿⣿⣿⣿⣿⣻⣟⣴⣿⣿⣳⣟⣳⣿⣿⡟⡒⠙⣛⡽⣿⠋⣽⢧⢯⣳⢿⣭⣛⣟⡼⡵⠹⣖⣫⠷⣾⣿⣳⢟⣶⣿⣿⡿⠟⣻⣾⣿⣷⣯⣿⣿⣿⣿⣿⣷⣽⣿⣿⣿⣿⣿⣿⣳⡌⠦⣄⠋⣮⡿⣿⣿⣿⣿⣻⣿⣯⣿⣿⣿⣷⣿⣮⣷⢕⢄⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀".
PRINT "⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⠔⠁⠐⣠⣼⣿⣿⣿⣿⡧⣿⣧⣦⣿⣯⢵⣻⣹⣿⣾⠛⣡⣩⡲⠧⣷⣾⡹⣞⡧⢟⡽⣲⡽⢾⣽⢟⡯⠔⠒⣼⣟⣰⣟⡿⣿⠋⠁⣠⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣮⣶⣄⡀⢘⣿⣽⠿⣿⣿⣿⣿⣿⡿⣿⣿⣿⣿⡽⡻⣿⣱⡤⡀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀".
PRINT "⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⡠⢃⢤⣿⣻⣿⣿⣿⣿⡿⣿⣾⣯⢙⠿⢿⣷⣞⣩⣿⣭⡴⣓⣼⠯⢙⣪⠵⡼⣋⠿⡽⢊⡴⢧⣟⣳⢮⢣⢌⢦⡊⠴⣨⣿⣿⣿⣧⣶⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣎⣿⣿⣿⣟⣿⣿⣿⣿⣿⣿⣿⣭⡺⣳⣶⣮⠢⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀".
PRINT "⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⠌⢠⣷⣏⣿⣿⣿⣿⢻⣿⠐⠋⣹⡏⠈⣦⣾⢻⢋⣵⣾⣷⢿⠟⠩⢈⣟⢌⠲⡱⣯⡟⣶⢫⡞⡽⡲⢧⠏⣞⡸⣆⢳⣿⣿⣿⣿⣾⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡷⡑⡄⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀".
PRINT "⠀⠀⠀⠀⠀⠀⠀⠀⠀⡠⢂⣼⣽⣿⡿⠻⠋⣿⣾⣽⡧⢒⣽⡟⣼⡇⣿⣿⣿⢿⣾⢿⢫⣧⣀⡿⢏⢫⠚⣩⣵⣶⣬⣵⣧⣼⣿⣿⣿⣿⣿⣷⢲⢹⣿⠟⣿⣿⣿⣿⣾⣿⣿⣿⣿⣿⣿⣿⣿⣿⣻⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣟⣱⣾⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣷⠽⣎⢆⠀⠀⠀⠀⠀⠀⠀⠀⠀".
PRINT "⠀⠀⠀⠀⠀⠀⠀⠀⡔⢀⣾⣟⡾⣿⣦⡔⡚⢿⣾⣉⣴⣿⣿⠟⣻⣷⢿⣿⡿⣼⣻⢆⣿⣿⡋⢤⣲⣼⣿⣿⣿⣿⣿⣿⣿⣿⣿⠿⠿⢟⡿⣟⣈⣽⠿⠉⣰⣾⣿⣿⣿⣿⣿⢿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣷⢻⣧⠢⠀⠀⠀⠀⠀⠀⠀⠀".
PRINT "⠀⠀⠀⠀⠀⠀⠀⠜⣠⣾⣿⣽⣿⣿⣿⣿⣄⣩⠟⣡⣿⣿⣿⣷⣭⣿⣏⢿⣯⣿⣷⢎⣽⢷⣿⣐⢤⣿⣿⣿⣿⣿⣿⣿⢟⣿⣥⣶⢹⢽⣿⣿⡿⣍⣶⡟⢫⣿⣻⣿⣿⣿⣿⡦⣿⣍⡿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡿⣿⣿⣿⣿⡻⣾⣷⡡⠀⠀⠀⠀⠀⠀⠀".
PRINT "⠀⠀⠀⠀⠀⠀⠜⣰⣿⣿⡿⣿⣿⣿⣿⣭⣿⣷⣿⣿⣿⣿⠟⡣⠺⠟⠻⣿⣿⣼⣿⡗⢻⣽⣿⣽⣿⣿⣿⣿⣾⣿⣟⡿⢻⣏⣠⣾⣽⣿⣿⢿⡿⠿⣯⣷⣄⣻⣝⣿⡿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡿⡟⣿⣿⣿⣧⣿⢯⣷⣡⠀⠀⠀⠀⠀⠀".
PRINT "⠀⠀⠀⠀⠀⠌⣀⣿⣛⣽⠃⠻⠿⠿⢿⠋⢘⣿⣻⡿⢳⣵⠞⠓⠫⣕⣦⣌⢻⣿⣿⢲⡽⣷⣟⡼⣿⣿⣿⣾⣿⣿⣿⣿⣛⣽⢻⣿⣿⣷⣿⣽⣿⣿⣷⣤⢮⡻⠿⣿⣿⣿⣿⣹⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡿⣿⣿⣿⡿⠹⣿⣏⠉⡟⣯⢿⣻⣷⡡⠀⠀⠀⠀⠀".
PRINT "⠀⠀⠀⠀⡘⡰⣿⡿⣿⣿⣧⣄⠴⣀⣜⣠⣿⣿⠛⢀⡟⢡⡤⠶⡛⠪⣯⢻⣈⢿⣫⣗⣻⣼⣿⢶⣿⣾⣿⣿⣿⣿⢯⢾⠽⢻⣿⣿⠟⠟⣿⣿⣿⣿⣿⣿⣿⣷⣿⣷⣽⣿⣻⡿⣿⡿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣽⣿⣿⣿⣽⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣼⣿⣿⣿⣿⣿⣿⣿⠀⢷⡜⢿⣿⣿⣵⢡⠀⠀⠀⠀".
PRINT "⠀⠀⠀⢠⢡⣷⣿⣷⣨⣿⣿⣿⣿⣿⣿⣿⣿⣿⣟⠘⠏⣿⠐⡀⡇⣄⣿⣞⣯⠸⡿⠳⣿⣿⢿⣾⡷⡽⠛⣿⣝⣮⣟⢣⡶⣿⣿⡇⢰⣾⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣾⣷⣶⣿⣯⣹⣟⣿⣟⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣶⣗⣳⡺⣿⣿⣻⣧⠃⠀⠀⠀".
PRINT "⠀⠀⢀⠃⣾⣿⣿⣿⢿⣮⡙⠻⣿⣽⣿⣿⣿⢿⣿⡐⠀⠉⠊⡜⣰⣿⡿⣿⡯⣷⣷⡄⣻⣹⣯⣯⣻⣿⣿⡻⣾⡷⠮⣵⣶⡿⣿⣿⣿⠶⢿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣷⣿⣿⣿⣿⣿⣿⡿⠟⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣇⡟⢿⣹⣿⣯⡟⡀⠀⠀".
PRINT "⠀⠀⡜⣸⡟⢿⡿⠃⣾⣿⣿⣷⣾⣿⣿⣿⣿⣿⣿⣷⣖⣤⣾⣼⣷⣗⠃⢠⣿⣻⣿⣿⣿⣭⣿⣿⣿⣿⣿⣿⣦⣹⡻⣿⣛⣧⣊⣡⣯⣤⣼⣯⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣾⣿⣏⢙⣿⣿⣿⣿⣿⣷⣶⣾⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣷⣿⣇⣿⣿⣿⣿⡽⣱⠀⠀".
PRINT "⠀⢠⢡⣿⣷⡾⣧⠀⣿⣿⣿⣿⣿⣿⡟⠿⣿⣿⣿⣿⣵⣷⣿⣿⣿⣧⠖⣿⣿⣷⡿⢻⣿⣿⣿⣿⣿⣿⣿⢿⣿⣽⣿⣿⣿⣾⣟⣿⠿⢻⣍⣻⣿⣿⣿⣿⡿⣿⣿⣿⣿⣿⣿⠿⣿⣿⣿⣿⣿⣿⣿⣿⣽⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡽⣯⣽⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣾⣿⣿⣿⣿⡃⡄⠀".
PRINT "⠀⡌⣸⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⠛⢯⡖⠘⠉⢿⣽⣿⣿⣿⢿⠟⢃⣴⣿⣿⣿⢿⢵⣿⣿⣿⣿⣿⣯⢞⣞⣹⡟⠧⠉⠛⠿⣿⣿⣮⡤⡹⠟⢿⣿⣿⣿⣿⣿⣿⣟⣛⣯⡷⣟⢫⡵⣯⣿⣿⣿⣿⣿⣿⣿⣿⣟⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣷⠠⠀".
PRINT "⢀⠁⣿⣿⣿⣿⣿⣿⡿⣿⣿⣿⣿⣿⡎⢀⣴⠀⣿⣿⣿⣿⣰⣿⣶⣿⢝⣿⡿⢁⡾⠐⣿⣿⣿⣿⣿⣿⣟⡿⣿⣿⣷⣶⣶⣶⣩⣽⣷⣽⣧⣟⣁⢸⣯⣛⢿⡿⣛⣎⣝⣾⣟⡾⢛⣼⣯⢹⡻⢯⣿⣩⠻⡝⢯⣭⠿⣿⣿⣿⣿⣿⣷⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣻⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣯⣿⣿⣿⡀⡄".
PRINT "⡞⢘⣿⣿⣿⣿⣿⣿⣾⣿⣿⣿⣿⣿⣿⣿⡣⢊⣽⣿⣿⣿⣿⣿⣿⣧⣞⣟⡃⣦⢇⣬⣿⡟⣿⣿⣏⣿⣻⣷⣦⠿⣿⣿⣿⠿⡿⠿⢿⠻⢿⣿⣧⣿⣿⡛⢿⢿⣿⣴⣾⣻⣿⢻⣿⣙⡶⠯⢭⡿⣤⣾⣧⣹⣿⣿⠿⣼⢿⢿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣯⣿⣿⣿⣿⣿⣿⣿⣿⣿⡟⣿⢯⣿⢩⣿⣿⡇⢁".
PRINT "⡇⣸⣿⣿⣿⠏⣯⣿⣾⣿⣿⣿⡿⣿⣫⣟⣾⢛⣡⣾⣿⣿⣿⣿⣿⣿⣿⣧⣮⢾⡫⡽⠿⠓⢈⣿⡿⢻⣿⣿⣿⢻⣉⣽⣿⣶⣶⣶⣿⡷⠀⠙⠛⣿⠿⠃⢹⣯⣹⣿⣿⡿⣻⣽⢏⣬⢗⢻⡹⣷⣿⣿⣺⣹⣻⣿⣿⣿⣳⣎⡽⣟⠯⣝⡻⢿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣄⡿⣻⣿⢾⣿⣿⣧⠀".
PRINT "⡇⠉⣿⠋⢻⡘⣿⣿⣿⣿⣿⣷⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣫⣹⢿⣿⣿⣏⢼⣡⢎⠏⡀⠰⢢⠊⠳⠟⣿⣿⣿⣿⣟⣿⣿⡟⣿⣿⣿⣿⠇⣤⣄⣀⣀⠀⠀⣹⣾⣿⡏⠐⡿⣡⣽⣾⣿⣯⣷⣾⣼⣿⣿⣿⣽⡷⠿⠾⣿⣳⢽⢾⣻⢽⣳⣧⡏⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣷⣾⣿⣿⣷⣿⣿⠀".
PRINT "⡇⣰⡷⣠⣿⣷⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣽⣿⣯⣿⣿⣿⣿⣿⣿⡿⢜⠜⡤⣾⢸⣧⣷⣷⣿⣇⢈⣯⡄⣻⣿⣴⣿⠟⣀⡹⣾⠟⠟⣿⣻⣀⣾⣿⣯⣽⣵⣿⣿⣿⣬⣗⣝⣶⣯⡖⣯⣿⣽⣿⣻⢿⣴⣋⡷⢯⣯⠿⣯⡿⣧⣟⣾⡝⢯⠿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣾⣿⣿⣿⠀".
PRINT "⡅⣟⣡⣾⣽⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣷⣿⣿⣿⣿⣿⣿⣿⣿⣿⣷⣾⣿⣿⣿⣿⣿⣿⣏⣈⣽⣯⣶⣿⡿⢻⢋⣽⣿⣿⣿⣖⣨⣾⣩⣯⢯⣿⣫⡗⣝⣿⣷⢏⡾⢓⡇⡧⣟⣟⣯⢏⣷⣻⣽⣷⢯⡿⢼⡇⢻⠻⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⠄".
PRINT "⡇⣻⢃⣿⣿⢿⣿⣿⣮⣿⡿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡿⣿⣿⣿⣿⣿⡟⣿⣹⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡟⣿⣿⣿⣿⣿⣾⣷⣿⣿⣿⣿⣟⣃⣼⣼⢋⣿⡟⢻⣤⡿⣫⣎⡁⢴⡾⣵⢺⣭⠽⣬⢚⣶⣓⠾⣵⣛⢾⣭⡿⣧⣿⣟⣿⣞⡟⣾⡼⣽⢧⠧⣿⢿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡇⢩⣿⣿⣿⠂".
PRINT "⡇⢳⣿⣿⣿⣾⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣷⢿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣯⣤⡾⡧⠿⢩⠎⠁⢲⣶⣎⡷⣦⣞⢷⡳⣏⣞⣻⡘⣯⢳⣭⡻⡵⢫⡟⢞⣓⡵⣬⣬⢷⡾⣝⣳⢏⡟⡎⣿⣍⡽⣭⣛⣿⢿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣥⣰⣿⣿⣿⠀".
PRINT "⡇⠈⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣷⣿⣿⣿⠿⣿⣿⣿⣿⣇⣿⣿⣻⡷⢿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡫⢾⡏⠀⡟⢩⣾⠷⡀⠋⠿⣗⣿⣎⢷⡓⢮⠼⠓⣷⢶⣱⢲⣕⣞⣹⣜⣳⢏⣞⡵⣯⣻⡼⣯⣝⣯⣿⣽⣿⣷⢶⢯⡿⣬⡟⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⠀".
PRINT "⡇⠀⣛⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣷⣽⣿⣿⣿⣿⣿⣹⣯⣿⣧⡩⡿⣿⣿⣿⣿⣿⣿⣿⣿⢷⡈⣷⣬⣒⣌⢻⣦⡴⣷⣬⠍⢻⣮⣓⣹⢋⡿⣛⡥⡿⢭⡻⣶⣝⠶⣍⡞⣯⢿⣹⢯⣽⣷⣯⣾⢿⣹⣿⠽⣭⣿⣿⣿⢅⣿⣿⣿⢏⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡟⠀".
PRINT "⢣⢠⣿⣾⣿⣿⣿⣿⣿⣿⣿⣿⣷⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣦⣿⣻⣿⣿⣟⡿⡟⣿⣿⣿⣿⡟⣽⢝⣽⣿⠏⠓⠉⠉⣿⡻⣿⣿⣿⣜⠾⢦⢿⢷⣯⣷⢿⣛⠾⣯⣛⣶⢧⣟⣞⡳⡝⣽⢹⢯⡷⣯⣟⣷⣚⢧⣟⣼⢻⣶⢿⣳⣿⣿⢸⣧⢽⡾⣿⣿⣯⠟⣿⣿⢿⣾⣿⣿⣻⣿⣿⣿⣟⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡇⠀".
PRINT "⠈⡄.⢻⣿⣯⡿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣷⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣯⣟⣿⣿⣿⣾⣬⣞⣻⣿⣶⣶⣴⣦⣸⡁⠙⢻⣿⣿⡞⡿⣷⣿⣿⡿⣏⠟⡿⣴⣛⢮⠻⢼⡝⣳⢽⢪⡽⣶⡽⣟⡖⣷⣻⣧⣟⠾⣿⣻⡿⣷⣿⢣⣿⣟⣍⣿⣿⢽⣧⣿⣿⣿⣿⣽⣿⣿⣿⢿⣿⣿⣿⣷⣽⣿⣿⣿⣿⣿⣿⣿⣿".
PRINT "⠀⢃⢸⣿⣿⣷⣟⣿⢿⣿⣿⣿⣿⣿⣿⣻⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣻⣿⣿⡻⣿⣟⣻⣻⣿⣟⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣧⣠⣬⣿⣛⣿⣏⢆⠻⣿⢶⣽⣞⣻⢖⣊⢷⣹⠾⣝⣫⠾⣧⣳⣈⠿⣝⣞⣲⣟⢧⣾⣻⣿⣯⢷⣻⢷⢻⣟⣧⣾⣿⣯⣿⣷⣷⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣏⢼⣿⣿⣿⡟⠀⠀".
PRINT "⠀⠘⡈⣿⣿⣿⣽⣿⣿⣻⣿⣽⣿⡿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣷⣿⣿⣯⣿⣿⣿⣾⣿⣿⣿⣷⣿⣿⣿⡿⣿⣿⣿⣿⣿⣿⢿⡿⣿⠛⡾⢷⡹⣿⣏⠻⣿⣧⡯⣝⡼⢚⠝⣿⠿⣧⣻⣗⡛⣾⣟⣿⢽⣻⠳⣿⣿⣵⢿⣿⣻⡿⣻⣿⣿⣦⣟⣲⣿⢛⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣷⣼⣶⣿⣿⣿⢇⠃⠀".
PRINT "⠀⠀⢣⠹⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣽⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣯⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡿⣯⣇⠈⢻⣿⣼⣷⣍⡟⢻⣷⣧⣯⢷⣿⣭⣲⣸⣴⡟⣳⡿⣏⣷⣽⣬⣾⣿⡾⣏⣟⣴⣿⡏⣿⣵⣽⣣⣿⣾⣿⣿⣿⠍⠙⢿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡘⠀⠀".
PRINT "⠀⠀⠈⡄⠽⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣾⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣻⣿⣿⣗⣿⣿⡛⢟⢿⠉⢛⣳⣯⠻⣿⣿⣦⡽⢿⣟⣿⣿⣟⣷⣽⣱⣟⣿⣱⢾⣽⣽⣓⡽⣿⣾⢿⣻⣟⣿⣷⣯⣿⣟⣽⣿⣿⠄⠠⡔⣤⣭⣛⠿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⢣⠁⠀⠀".
PRINT "⠀⠀⠀⠐⢀⠹⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⢽⣿⣿⣿⣯⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣷⣿⣄⢐⣟⣿⣤⣤⣟⣿⣿⣎⢫⢿⡳⣿⣿⣾⠣⣜⠓⣿⣷⣻⢿⣍⣷⡾⢹⣿⣿⣏⣿⣿⡿⣿⣿⣿⡟⠁⠀⢠⡾⠛⠋⠛⠻⠶⢌⣹⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⠏⠂⠀⠀⠀".
PRINT "⠀⠀⠀⠀⢡⠀⠙⢿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣻⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣟⣻⡟⠛⠙⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣷⣻⣷⡿⡗⡘⠿⣤⠈⣛⣿⡼⢪⣷⣹⣿⣿⡽⣏⢿⡼⢻⣿⣷⢞⣟⣿⣻⣽⢟⣿⢽⢾⣿⣿⣿⡏⠀⠞⡏⢧⣷⣿⣏⣐⣷⣶⡳⣍⡙⢿⢿⣿⣿⣿⣿⣿⣿⡟⡘⠀⠀⠀⠀".
PRINT "⠀⠀⠀⠀⠀⢃⠱⣆⠣⡈⠻⢿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣷⣿⣿⣾⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣶⣶⣿⣿⣿⣿⣿⣿⣿⣿⣿⢿⣻⣿⣿⣾⣍⡳⣟⣿⡿⣤⠝⣧⡵⡿⣿⣪⣿⣿⣿⣿⠻⣜⠯⢞⣽⣳⡟⣾⣻⢻⢿⠛⣹⣾⣋⡾⣿⣿⣿⣀⢀⣾⣴⣿⣿⣿⣿⣿⣿⣿⡓⠼⣿⣪⣝⣿⣿⣿⣿⣿⡿⡑⠀⠀⠀⠀⠀".
PRINT "⠀⠀⠀⠀⠀⠀⢣⠈⣧⡈⠢⠀⢨⠛⢯⢻⣟⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡿⣽⡻⢿⣿⣿⣿⣿⣿⣽⣗⣄⡛⣿⣿⢿⣶⣿⣿⢿⣇⣘⣯⣟⣿⣿⣻⣿⣿⢣⡽⢞⣽⣫⢞⣻⣿⣿⡿⣏⣧⣿⣿⣲⣷⣾⣽⣫⣾⣴⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣳⣯⣽⣿⣿⣿⣿⣿⡿⡑⠀⠀⠀⠀⠀⠀".
PRINT "⠀⠀⠀⠀⠀⠀⠀⢢⠹⣿⣦⣄⠀⠁⠀⠡⠈⠛⢿⣿⣿⣿⣿⣿⣿⣿⣿⣿⢿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣧⣈⡁⢶⣾⣿⣿⣿⣿⣿⣿⣿⣿⣷⣿⣿⣿⣿⣷⣿⣿⣿⣿⣼⢏⣿⣿⣿⣿⣏⢷⡹⣟⣾⣵⢯⣷⣿⣿⣷⣿⣿⡿⣯⣿⣾⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡿⡑⠀⠀⠀⠀⠀⠀⠀".
PRINT "⠀⠀⠀⠀⠀⠀⠀⠀⠢⡘⣿⣿⣷⣤⡀⠀⠀⠐⡀⢉⠻⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣻⡿⣿⣿⣿⣿⣿⣿⣿⣯⣿⣽⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⢓⡇⢀⣷⣿⡿⣿⣘⢧⡟⣵⢿⠞⠳⣾⠿⣿⣿⣿⣿⢿⡿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⠟⠔⠀⠀⠀⠀⠀⠀⠀⠀".
PRINT "⠀⠀⠀⠀⠀⠀⠀⠀⠀⠁⠌⠻⣿⣿⣿⣦⡀⠀⢤⡀⠀⠈⢿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣇⡾⣿⣿⣿⢾⢿⣳⣟⣾⡹⣭⢿⣅⣤⣁⣊⣉⠍⠉⢀⡾⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⢋⠌⠀⠀⠀⠀⠀⠀⠀⠀⠀".
PRINT "⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠈⢃⠘⢿⣿⣿⣷⣦⡀⠐⡄⠀⠈⠱⡿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣽⠃⣿⠿⣿⡟⣶⣿⡕⣮⢷⣭⣯⣽⣯⣎⣁⡀⢁⣤⠴⣿⣻⢿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡿⡱⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀".
PRINT "⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠑⡌⠙⣿⣿⣿⣿⣦⣈⡀⠀⡀⠀⠙⢿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡻⣿⣿⣿⣿⣮⣿⠿⣽⣟⣿⣿⣿⣿⣽⡿⣿⣿⣿⣿⣿⣿⣿⣿⣿⠆⣿⣗⣿⣿⣿⢯⢿⣹⣞⡷⣿⣻⣿⣽⣿⡓⠶⣽⠹⠛⣵⣯⠿⣿⣽⡿⢩⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⢟⠔⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀".
PRINT "⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠈⠢⡀⠛⢿⡻⣿⣿⣿⣤⣈⢄⡀⠀⠘⣿⣿⣿⣿⣿⣿⣿⣿⣿⡟⢡⣾⣿⣿⣿⣿⣿⣿⣿⣿⣿⣻⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⠿⠇⣸⣽⣿⣏⢞⡷⣯⣛⠷⣮⢻⣽⣾⣿⣿⣿⣿⣷⡆⠐⢠⣿⣭⡾⣿⣿⣥⣾⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⢟⠑⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀".
PRINT "⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢦⠉⠻⣾⣿⣿⣿⣿⣷⣭⣦⡀⠈⠺⠻⣿⣿⣿⡿⣿⣿⡆⣤⣿⣿⡿⣭⠉⠳⣻⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡿⡘⣸⣿⡿⡏⠞⣽⢾⠝⣮⣿⣾⣿⣟⢻⠿⠿⢿⠿⣿⣧⡴⢯⣍⡵⠴⢿⣿⣿⣿⣿⡿⢿⣿⣿⣿⣿⣿⣿⣿⡟⡣⠂⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀".
PRINT "⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠙⢦⠈⠻⣟⣿⣿⣿⣿⡿⣿⣦⣀⠀⠑⢽⡻⣿⣽⡙⢿⣮⣿⡉⠻⠿⢷⣦⣄⣈⡑⠻⠽⣛⣿⠿⢿⣿⣿⣷⣿⣿⣿⡿⡟⢲⣿⠛⠻⡟⢾⣭⣯⣿⣿⣏⣋⠙⢿⣶⡄⠀⠂⣀⣩⡝⣳⣶⢋⢤⣶⣿⢽⡿⢛⣏⡉⣾⣿⣿⣿⣿⣿⠟⡩⠊⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀".
PRINT "⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠢⡈⠛⢽⢿⣿⣿⣮⣽⣿⣷⣦⠀⠻⣿⣿⣧⡀⠉⠉⠉⠀⠀⠓⠒⠲⢮⣝⣻⣻⢶⣮⡭⣓⡲⠿⣿⣫⡯⢟⣻⢏⣼⣿⣷⣿⠷⣿⣿⣿⣿⣿⣿⡿⢿⣶⡢⠤⣇⣁⣀⣩⡶⣫⣿⣟⣾⠿⣾⢿⣾⣛⣶⣾⣿⣿⣿⡿⢟⠁⠊⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀".
PRINT "⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠈⠢⢄⠑⠳⣾⣿⣿⠻⣻⡷⡑⡀⠙⢿⣿⣿⣿⣶⣦⣄⡤⠀⠀⢀⣤⣾⡫⠉⠡⢶⢬⠝⠻⠫⠶⢌⠙⢿⠼⣵⠿⢿⣿⣷⣿⣿⣿⣿⣿⣷⣉⡛⢻⡿⢿⣭⣿⣯⣽⣽⣿⣿⣷⣜⣶⡶⣾⣯⣝⣫⣽⣿⣿⠟⣋⠔⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀".
PRINT "⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠁⠢⣌⠙⠛⠿⣯⣿⠷⣱⣄⡀⠈⠹⠿⣿⡛⢿⠿⣷⣀⠘⢿⣟⠁⠀⢩⢃⣭⣻⣟⣳⣄⣀⠀⠘⠀⣶⣆⣛⠛⣷⣿⣿⠿⣿⡿⣷⣟⣿⣿⣟⣿⣶⣾⣿⣿⣿⣿⡿⠿⠻⣟⡿⣿⣿⣿⣿⠿⢋⠕⠊⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀".
PRINT "⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠈⠑⠢⣀⠈⠻⣆⣫⣽⡿⣗⢦⡐⣤⣉⣈⡽⠚⢻⡿⠟⠚⡩⠔⣣⣾⣿⣋⣩⡍⠿⢿⣿⣦⡄⠿⣾⣧⣶⣍⣉⣛⣳⣥⣴⣮⡽⠿⠟⢻⠿⣛⣟⡏⣡⣴⣶⣿⣟⡵⣿⣽⣿⠿⢋⠅⠊⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀".
PRINT "⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠈⠂⠤⡉⠉⠛⢋⣝⢪⠬⣑⣂⣠⣙⣩⣄⣶⣿⣶⣿⠿⠷⠟⠋⠩⣭⣿⣷⣿⣿⣿⣷⣄⠈⠉⠉⠉⢉⣀⣤⣴⣶⣾⣿⣿⢛⣛⣯⠳⠞⣛⣻⡿⣿⠻⢛⣋⠫⠐⠉⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀".
PRINT "⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠁⠒⠄⢀⡀⠙⠥⣘⠛⠛⡛⢋⣉⢩⣫⣭⣵⣶⣶⡿⠿⠛⢏⠾⠾⠥⢒⠛⣛⠻⠟⠛⠛⠛⢚⡹⠋⡭⠒⠉⣀⠈⣀⣄⣳⠶⠟⢉⣉⠦⠒⠈⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀".
PRINT "⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠉⠒⠀⠤⣀⠈⠈⠉⠉⠋⠉⠹⠩⠄⠀⠀⠀⠁⠀⠀⠀⠀⠉⠀⠀⠀⠀⠉⠈⠁⠀⢀⠀⣤⠤⡴⢛⣉⠱⠔⠚⠉⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀".
PRINT "⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠉⠁⠒⠂⠤⠤⠄⣀⢀⣠⣀⣀⣀⣀⣀⣀⣀⣀⣈⣠⣤⣴⣂⣰⡮⠦⠔⠓⠈⠉⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀".
PRINT "".
PRINT "".
PRINT "".
PRINT "".
PRINT "".
4.
    SET TERMINAL:HEIGHT TO 40.
  SET TERMINAL:WIDTH TO 80.
}

//------------------ Marquee chase lights code -------------------//

// Marquee chase using AGX groups with a trailing tail.
// Edit the ORDER list to match your AGX mapping (head chases in list order).

// ---------- STOP CONDITION ----------
FUNCTION STOP_NOW {
  RETURN GET_AG_STATE(112).
}.

// ---------- ORDER (lower/upper, interleaved, left->right) ----------
FUNCTION BUILD_ORDER {
  // Lower row in your exact physical order (note 102,111,136,103)
  LOCAL LOWER IS LIST(101, 102, 111, 136, 103, 104, 105, 106, 107, 108, 109).
  // Upper row left->right
  LOCAL UPPER IS LIST(126, 127, 128, 129, 130, 131, 132, 133, 134).

  LOCAL L IS LIST().
  LOCAL i IS 0.
  LOCAL j IS 0.

  // Interleave lower and upper: L,U,L,U,...
  UNTIL i >= LOWER:LENGTH AND j >= UPPER:LENGTH {
    IF i < LOWER:LENGTH { L:ADD(LOWER[i]). SET i TO i + 1. }.
    IF j < UPPER:LENGTH { L:ADD(UPPER[j]). SET j TO j + 1. }.
  }

  RETURN L.
}.

SET marquee_active TO FALSE.
SET marquee_order TO LIST().
SET marquee_onstate TO LIST().
SET marquee_life TO LIST().
SET marquee_original_state TO LIST().
SET marquee_pos TO -1.
SET marquee_reverse TO FALSE.
SET marquee_tail_steps TO 4.
SET marquee_dt TO 0.2.
SET marquee_nextTick TO 0.
SET marquee_generation TO 0.

FUNCTION CLEAR_MARQUEE_LIGHTS {
  LOCAL n IS marquee_order:LENGTH.
  LOCAL onN IS marquee_onstate:LENGTH.
  LOCAL lifeN IS marquee_life:LENGTH.

  FOR IDX IN RANGE(0, n) {
    LOCAL ag IS marquee_order[IDX].
    // Only toggle if the light is currently on
    IF GET_AG_STATE(ag) {
      TOGGLE_AG(ag).
    }.
    IF IDX < onN { SET marquee_onstate[IDX] TO FALSE. }.
    IF IDX < lifeN { SET marquee_life[IDX] TO 0. }.
  }.
}.

FUNCTION STOP_MARQUEE {
  IF marquee_active { CLEAR_MARQUEE_LIGHTS(). }.
  SET marquee_active TO FALSE.
  // Park the trigger: it will stop firing because the time condition won't be met:
  SET marquee_nextTick TO TIME:SECONDS + 999999.
  // (If your kOS supports trigger removal and you stored it, you can still do:)
  // IF marquee_trigger <> FALSE { marquee_trigger:REMOVE(). SET marquee_trigger TO FALSE. }.
}.


FUNCTION MARQUEE_STEP {
  IF NOT marquee_active { RETURN. }.

  IF STOP_NOW() {
    STOP_MARQUEE().
    RETURN.
  }.

  SET marquee_pos TO MOD(marquee_pos + 1, marquee_order:LENGTH).
  LOCAL idx IS marquee_pos.
  IF marquee_reverse {
    SET idx TO marquee_order:LENGTH - 1 - marquee_pos.
  }.
  LOCAL ag IS marquee_order[idx].

  IF NOT marquee_onstate[idx] {
    TOGGLE_AG(ag).
    SET marquee_onstate[idx] TO TRUE.
  }.
  SET marquee_life[idx] TO marquee_tail_steps.

  FOR j IN RANGE(0, marquee_order:LENGTH) {
    IF marquee_life[j] > 0 {
      SET marquee_life[j] TO marquee_life[j] - 1.
    } ELSE {
      IF marquee_onstate[j] {
        TOGGLE_AG(marquee_order[j]).
        SET marquee_onstate[j] TO FALSE.
      }.
    }.
  }.
}.


// ---------- CORE ENGINE ----------
FUNCTION RUN_MARQUEE {
  PARAMETER orderList, dt IS 0.12, tailSteps IS 4, reverse IS FALSE.

  LOCAL N IS orderList:LENGTH.
  IF N = 0 OR tailSteps < 1 { RETURN. }.

  STOP_MARQUEE().
  LOCAL myGen IS marquee_generation.

  SET marquee_order TO LIST().
  FOR ag IN orderList {
    marquee_order:ADD(ag).
  }.

  // Save original states before changing anything
  SET marquee_original_state TO LIST().
  FOR ag IN marquee_order {
    marquee_original_state:ADD(GET_AG_STATE(ag)).
  }.

  // Turn off all lights to start clean
  FOR ag IN marquee_order {
    SET_AG_STATE(ag, FALSE).
  }.

  SET marquee_onstate TO LIST().
  SET marquee_life TO LIST().
  FOR idx IN RANGE(0, N) {
    marquee_onstate:ADD(FALSE).
    marquee_life:ADD(0).
    SET marquee_onstate[idx] TO FALSE.
    SET marquee_life[idx] TO 0.
  }.

  SET marquee_pos TO -1.
  SET marquee_reverse TO reverse.
  SET marquee_tail_steps TO tailSteps.

  LOCAL stepDt IS dt.
  IF stepDt <= 0 { SET stepDt TO 0.01. }.
  SET marquee_dt TO stepDt.
  SET marquee_nextTick TO TIME:SECONDS.

  SET marquee_active TO TRUE.

  WHEN TRUE THEN {
    IF marquee_generation <> myGen { RETURN. }.
    IF NOT marquee_active { RETURN. }.
    IF TIME:SECONDS < marquee_nextTick { PRESERVE. }.
    SET marquee_nextTick TO TIME:SECONDS + marquee_dt.
    MARQUEE_STEP().
    PRESERVE.
  }.
}.

// ---------- ENTRY POINTS ----------
FUNCTION start_marquee_lr {
  PARAMETER dt IS 0.12, tailSteps IS 4.
  RUN_MARQUEE(BUILD_ORDER(), dt, tailSteps, FALSE).
}.

FUNCTION start_marquee_rl {
  PARAMETER dt IS 0.12, tailSteps IS 4.
  RUN_MARQUEE(BUILD_ORDER(), dt, tailSteps, TRUE).
}.

FUNCTION DISPLAY_STAGE_LABEL {
  PARAMETER stage_label.
  // Display the stage label in the UI
  CLEARSCREEN.
  PRINT "=================================================".
  PRINT "= " + stage_label.
  PRINT "=================================================".

}