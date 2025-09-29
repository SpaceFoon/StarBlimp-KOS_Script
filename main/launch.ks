


//--------------- TIME FORMATS---------------------------------------------------
//https://www.reddit.com/r/Kos/comments/4bh15w/program_simple_code_TO_convert_mission_time_inTO/
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
// PRINT formatMET.

FUNCTION formatUNI
{
  LOCAL ts IS TIME.
  RETURN "[Y" 
    + ROUND(ts:YEAR) + ", D"
    + padZ(ts:DAY) + ", "
    + padZ(ts:HOUR) + ":"
    + padZ(ts:MINUTE) + ":"
    + padZ(ROUND(ts:SECOND))+ "]".
}
// PRINT formatUNI.

//---------------------------Steering stuff-------------------------------------

//The KSC (fully upgraded) runway's (Latitude, Longitude) starts from about (-0.0485997, -74.724375) and ends at about (-0.0502119, -74.489998).
//https://www.reddit.com/r/KerbalSpaceProgram/comments/5wl42h/kos_runway_centerline_geoposition_analysis/

//--------steers with only wheels while on runway. Holds center line. Somehow works even when you are not on the runway.

SET CENTERLINE_EQ_A TO 142.13236295.    // latitude coefficient for the linear equation
SET CENTERLINE_EQ_B TO 1.               // longitude coefficient
SET CENTERLINE_EQ_C TO 81.62849024.     // constant term
SET RUNWAY_EAST_THRESHOLD_LAT TO -0.0502118560109606. //start of runway
SET RUNWAY_EAST_THRESHOLD_LNG TO -74.4899977802028. //start of runway
SET CENTERLINE_PID_OUTPUT_LIMIT TO 2.

//steering pid
SET centerline_pid TO pidloop(1, .2, 2, -CENTERLINE_PID_OUTPUT_LIMIT, CENTERLINE_PID_OUTPUT_LIMIT).
SET centerline_pid:SETpoint TO 0.

LOCK current_heading_target TO centerline_pid:output + 90.

// SET current_pitch_target TO SHIP:FACING:PITCH.
LOCK centerline_angular_deviation TO (CENTERLINE_EQ_A * ship:geoposition:lat + CENTERLINE_EQ_B * ship:geoposition:lng + CENTERLINE_EQ_C) / sqrt(CENTERLINE_EQ_A^2 + CENTERLINE_EQ_B^2).
LOCK centerline_linear_deviation TO -2 * constant:pi * KERBIN:RADIUS * centerline_angular_deviation / 360.

//flying pid
FUNCTION SETPID {
    PARAMETER axis, Kp, Ki, Kd.

    if axis = "pitch" {
        SET steeringmanager:pitchpid:Kp TO Kp.
        SET steeringmanager:pitchpid:Ki TO Ki.
        SET steeringmanager:pitchpid:Kd TO Kd.
    }
    ELSE IF axis = "roll" {
        SET steeringmanager:rollpid:Kp TO Kp.
        SET steeringmanager:rollpid:Ki TO Ki.
        SET steeringmanager:rollpid:Kd TO Kd.
    }
    ELSE IF axis = "yaw" {
        SET steeringmanager:yawpid:Kp TO Kp.
        SET steeringmanager:yawpid:Ki TO Ki.
        SET steeringmanager:yawpid:Kd TO Kd.
    }
  LOG_INFO(axis + " PID settings updated").
  LOG_STATUS(axis + " Kp", Kp).
  LOG_STATUS(axis + " Ki", Ki).
  LOG_STATUS(axis + " Kd", Kd).
}
// Set PID values for pitch, roll, and yaw
// SETPID("pitch", 5, 0.5, .5).
// SETPID("roll", 3, 0.1, 0.5).
// SETPID("yaw", 3, 0.3, 0.8).

//put steering wheels in list for later use.
SET wheels TO LIST().
FOR PART in SHIP:PARTS {
    if PART:HASMODULE("ModuleWheelSteering") {
        wheels:ADD(PART).
    }
}

// Find and turn off all hl10rudder parts which is all of the crafts aero control surface. They will slow us down while driving.
FUNCTION ControlSurfacesOff{
    FOR  PART in SHIP:PARTS {
        if PART:NAME = "HL10Rudder" {
            // PRINT "Turning off part: " + PART:NAME.
            // Assuming we need TO SET the control surface  authority TO zero
            PART:GETMODULE("SyncModuleControlSurface"):SETFIELD ("authority limiter", 0).
            // PRINT PART:modules.
        }
    }
}

//Function to shut down engines and intakes properly when they are out of juice.
FUNCTION monitorEngines {

    list engines in engList.  //list of all engines
    FOR  eng in engList {
        if eng:NAME = "WBILargeElectricPart"{
            if eng:ignition = true and PROPSDONE = FALSE{
                LOG_STATUS("Engine " + eng:NAME + " thrust", ROUND(eng:THRUST), "kN").
                if eng:THRUST < 15 {
                    LOG_WARN("Engine thrust too low, shutting down: " + eng:NAME).
                    eng:SHUTDOWN().
                    SET PROPSDONE TO TRUE.
                }
            }
            
        }
        if eng:NAME = "turboFanEngine" {
            LOG_STATUS("Engine " + eng:NAME + " thrust", ROUND(eng:THRUST), "kN").
            if eng:THRUST < 10 {
                LOG_WARN("Engine thrust too low, shutting down: " + eng:NAME).
                eng:SHUTDOWN().
                INTAKES OFF.
                SET JETSDONE TO TRUE.
            }
        }
    }
}

//-----------------------------Pretty Stuff------------------------------------//
// Function to print a formatted message with a timestamp
FUNCTION PrintTimeStamped {
    PARAMETER msg.
    PRINT " " + formatMET + " " + msg.
}.

FUNCTION LOG_PREFIXED {
    PARAMETER tag, msg.
    PrintTimeStamped("[" + tag + "] " + msg).
}.

FUNCTION LOG_INFO {
    PARAMETER msg.
    LOG_PREFIXED("INFO", msg).
}.

FUNCTION LOG_WARN {
    PARAMETER msg.
    LOG_PREFIXED("WARN", msg).
}.

FUNCTION LOG_ERROR {
    PARAMETER msg.
    LOG_PREFIXED("ERROR", msg).
}.

FUNCTION LOG_STATUS {
    PARAMETER label, value, units IS "".
    LOCAL message IS label + ": " + value.
    IF units <> "" {
        SET message TO message + " " + units.
    }.
    LOG_PREFIXED("STATUS", message).
}.

FUNCTION LOG_STAGE {
    PARAMETER title.
    PRINTDivider("[STAGE] " + title).
}.
FUNCTION PRINTDivider {
    PARAMETER title.
    PRINT "============================================".
    PRINT "= " + formatMET.
    PRINT "= " +title.
    PRINT "============================================".
}

FUNCTION PRINTWelcome {
    SET TERMINAL:HEIGHT TO 46.
    SET TERMINAL:WIDTH TO 80.
    CLEARSCREEN.
    PRINT "============================================================================".
    PRINT "====================              WELCOME TO THE            ================".
    PRINT "====================       AIRSHIP LAUNCH CONTROL SYSTEM    ================".
    PRINT "============================================================================".
    PRINT " ".
    PRINT"                                 _..--=--..._        ".
    PRINT"                              .-'            '-.  .-.".
    PRINT"                             /.'    Blimps    '.\/  /".
    PRINT"                            |=-       in       -=| ( ".
    PRINT"                             \'.   Spaaaaace! .'/\  \".
    PRINT"                              '-.,_____ _____.-'  '-'".
    PRINT"                                   [_____]=8         ".
    PRINT " ".
    WAIT 1.
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
    TERMINAL:REVERSE.
    WAIT .2.
    TERMINAL:REVERSE.
}
//----------- Steering and Runway Loop-----------------//

//brake till sTOpped then go foward
SET FLAPSLVL TO 0.
FUNCTION lstage1 {
        RUN_MARQUEE(BUILD_ORDER(), 0.25, 4).
        //reverse fan
        TOGGLE AG4.
        WAIT.1.
        //jets
        TOGGLE AG3.
        WAIT.1.
        //just TO make sure
        INTAKES ON.
        SET BRAKES TO true.
        WAIT until SHIP:groundspeed < 1.
        SET BRAKES TO false.
        WAIT.1.
        STOP_MARQUEE().
    }
//jump
FUNCTION lstage2 {
        TOGGLE ag5.
        SET TERMINAL:HEIGHT TO 44.
        SET TERMINAL:WIDTH TO 74.
        WAIT.1.
        RUN_MARQUEE(BUILD_ORDER(), 0.50, 6).
        
  LOG_STAGE("Dem Duke boys...").
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
                    WAIT 1.
        //TOGGLE ag50. Fireworks are broken. They doin't go anywhere and break themselvs
        SET TERMINAL:HEIGHT TO 14..
        SET TERMINAL:WIDTH TO 44.
        SET FLAPSLVL TO 1.
        STOP_MARQUEE().
        RETURN FLAPSLVL.

    }


//------------------ Marquee chase lights code -------------------//

// Marquee chase using AGX groups with a trailing tail.
// Edit the ORDER list to match your AGX mapping (head chases in list order).

// ---------- STOP CONDITION ----------
FUNCTION STOP_NOW {
  RETURN AG112.
}.

// ---------- AG TOGGLER (map AG number -> toggle) ----------
FUNCTION TOGGLE_AG {
  PARAMETER ag.
  IF ag = 101 { TOGGLE AG101. } ELSE IF ag = 102 { TOGGLE AG102. }
  ELSE IF ag = 103 { TOGGLE AG103. } ELSE IF ag = 104 { TOGGLE AG104. }
  ELSE IF ag = 105 { TOGGLE AG105. } ELSE IF ag = 106 { TOGGLE AG106. }
  ELSE IF ag = 107 { TOGGLE AG107. } ELSE IF ag = 108 { TOGGLE AG108. }
  ELSE IF ag = 109 { TOGGLE AG109. } ELSE IF ag = 111 { TOGGLE AG111. }
  ELSE IF ag = 126 { TOGGLE AG126. } ELSE IF ag = 127 { TOGGLE AG127. }
  ELSE IF ag = 128 { TOGGLE AG128. } ELSE IF ag = 129 { TOGGLE AG129. }
  ELSE IF ag = 130 { TOGGLE AG130. } ELSE IF ag = 131 { TOGGLE AG131. }
  ELSE IF ag = 132 { TOGGLE AG132. } ELSE IF ag = 133 { TOGGLE AG133. }
  ELSE IF ag = 134 { TOGGLE AG134. } ELSE IF ag = 136 { TOGGLE AG136. }.
}.

FUNCTION GET_AG_STATE {
  PARAMETER ag.
  IF ag = 101 { RETURN AG101. } ELSE IF ag = 102 { RETURN AG102. }
  ELSE IF ag = 103 { RETURN AG103. } ELSE IF ag = 104 { RETURN AG104. }
  ELSE IF ag = 105 { RETURN AG105. } ELSE IF ag = 106 { RETURN AG106. }
  ELSE IF ag = 107 { RETURN AG107. } ELSE IF ag = 108 { RETURN AG108. }
  ELSE IF ag = 109 { RETURN AG109. } ELSE IF ag = 111 { RETURN AG111. }
  ELSE IF ag = 126 { RETURN AG126. } ELSE IF ag = 127 { RETURN AG127. }
  ELSE IF ag = 128 { RETURN AG128. } ELSE IF ag = 129 { RETURN AG129. }
  ELSE IF ag = 130 { RETURN AG130. } ELSE IF ag = 131 { RETURN AG131. }
  ELSE IF ag = 132 { RETURN AG132. } ELSE IF ag = 133 { RETURN AG133. }
  ELSE IF ag = 134 { RETURN AG134. } ELSE IF ag = 136 { RETURN AG136. }.
  RETURN FALSE.
}.

FUNCTION SET_AG_STATE {
  PARAMETER ag, desired.
  LOCAL current IS GET_AG_STATE(ag).
  IF desired AND NOT current {
    TOGGLE_AG(ag).
  } ELSE IF NOT desired AND current {
    TOGGLE_AG(ag).
  }.
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
SET marquee_pos TO -1.
SET marquee_reverse TO FALSE.
SET marquee_tail_steps TO 4.
SET marquee_dt TO 0.12.
SET marquee_nextTick TO 0.
SET marquee_generation TO 0.

FUNCTION CLEAR_MARQUEE_LIGHTS {
  LOCAL n IS marquee_order:LENGTH.
  LOCAL onN IS marquee_onstate:LENGTH.
  LOCAL lifeN IS marquee_life:LENGTH.

  FOR IDX IN RANGE(0, n) {
    SET_AG_STATE(marquee_order[IDX], FALSE).
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







//Re-enable Rudder control.
FUNCTION enableCS {
    FOR  PART in SHIP:PARTS {
       if PART:NAME = "HL10Rudder" {
            // PRINT "Turning on part: " + PART:NAME.
          PART:GETMODULE("SyncModuleControlSurface"):SETFIELD("authority limiter", 32).
        }
    }
}
//With Helium!
FUNCTION fillBaloon {
    FOR  PART in SHIP:PARTS {
     if PART:HASMODULE("HLEnvelopePartModule") {
        // PRINT "part: " + part:name.
        PART:GETMODULE("HLEnvelopePartModule"):DOACTION("buoyancy max", true).
    }
}
}

//Dont fill this one as much for balance.
FUNCTION lvlBaloon {
    FOR  PART in SHIP:PARTS {
        if PART:NAME = "hl10NoseCone" {
            // Loop TO press the "buoyancy --" butTOn 39 times
            SET butTOnPresses TO 0.
            until butTOnPresses >= 39 {
                PART:GETMODULE("HLEnvelopePartModule"):DOACTION("buoyancy --", true).
                CLEARSCREEN.
                PRINT "lEVELING BALOONS: "+ ROUND(butTOnPresses * 2.631) + " / 100".
                SET butTOnPresses TO butTOnPresses + 1.
                WAIT 0.02.
            }
        }
    }
}

FUNCTION mediumCS {
    FOR  PART in SHIP:PARTS {
       if PART:NAME = "hl10rudder" {
            // PRINT "Turning on part: " + PART:NAME.
          PART:GETMODULE("SyncModuleControlSurface"):SETFIELD("authority limiter", 20).
        }
    }
}

//https://forum.kerbalspaceprogram.com/topic/181894-orbital-mechanics-of-circularization/
// circularizes at the next apsis
FUNCTION CIRCLE {
SET burn_at_periapsis TO TRUE.
if APOAPSIS > 0 { // if apoapsis is negative, time TO apoapsis is infinite
	if ETA:APOAPSIS < ETA:PERIAPSIS { SET burn_at_periapsis TO FALSE. }
}
if burn_at_periapsis { SET which_apsis_text TO "Periapsis". } ELSE { SET which_apsis_text TO "Apoapsis". }

if burn_at_periapsis {
	SET node_time TO TIME:SECONDS + ETA:PERIAPSIS.
	SET otherapsis TO APOAPSIS.
	SET burnapsis TO PERIAPSIS.
} ELSE {
	SET node_time TO TIME:SECONDS + ETA:APOAPSIS.
	SET otherapsis TO PERIAPSIS.
	SET burnapsis TO APOAPSIS.
}

SET v_old TO sqrt(BODY:MU * (2/(burnapsis+BODY:RADIUS) -
                             1/SHIP:OBT:SEMIMAJORAXIS)).
SET v_new TO sqrt(BODY:MU * (2/(burnapsis+BODY:RADIUS) -
			     1/(BODY:RADIUS+burnapsis))).
SET dv TO v_new - v_old.

SET MyNode TO NODE(node_time, 0,0,dv).
ADD MyNode.

LOG_INFO("Setting up node at " + which_apsis_text).
LOG_INFO("ETA=" + ROUND(node_time - TIME:SECONDS,1) + ", dV=" + ROUND(dv,1)).
LOG_INFO("Old eccentricity was " + ROUND(SHIP:ORBIT:ECCENTRICITY,4) + ", new eccentricity is " + ROUND(MyNode:ORBIT:ECCENTRICITY,4)).
PRINT MyNode:ORBIT:APOAPSIS.  // apoapsis after maneuver
PRINT MyNode:ORBIT:PERIAPSIS. // periapsis after maneuver
}

// Base formulas:
// Δv = ∫ F / (m0 - consumptionRate * t) dt
// consumptionRate = F / (Isp * g)
// ∴ Δv = ∫ F / (m0 - (F * t / g * Isp)) dt

// Integrate:
// ∫ F / (m0 - (F * t / g * Isp)) dt = -g * Isp * log(g * m0 * Isp - F * t)
// F(t) - F(0) = known Δv
// Expand, simplify, and solve for t

FUNCTION calculateBurnTime {
  SET totalDeltaV TO nextnode:DELTAV:MAG.
  SET totalThrust TO 0.
  SET totalISP TO 0.
    LIST ENGINES IN engList.
    for eng in engList {
        if eng:IGNITION {
            set totalThrust to totalThrust + eng:AVAILABLETHRUST.
            set totalISP to totalISP + (eng:ISP * eng:AVAILABLETHRUST).
             print eng:ISP.
             print eng:AVAILABLETHRUST.
        }
    }
  LOCAL f IS totalThrust * 1000.  // Engine Thrust (kg * m/s²)
  LOCAL m IS SHIP:MASS * 1000.        // Starting mass (kg)
  LOCAL e IS CONSTANT:E.            // Base of natural log
  LOCAL p IS totalISP.               // Engine ISP (s)
  LOCAL g IS CONSTANT:g0.                 // Gravitational acceleration constant (m/s²)
  RETURN g * m * p * (1 - e^(-totalDeltaV/(g*p))) / f.

}

// Function to execute a burn at a maneuver node
// Not using now. Was not accurate.
FUNCTION executeManeuver {
  LOG_INFO("Burn time is: " + burnTime).
    wait.2.
    if mynode:typename = "node"{
        LOCK STEERING TO mynode:burnvector.
    }else{
    LOG_ERROR("LOCK STEERING TO mynode:burnvector failed.").
    }
    
  LOG_INFO("Steering locked to maneuver node.").

    // WAIT until the maneuver node's time minus half the burn time
    WAIT until eta:nextnode < (burnTime / 2).

    LOCK throttle TO 1.
  LOG_INFO("Executing burn...").

    // WAIT for the burn duration
    WAIT burnTime.
    // I have no idea why I need a -1 but I do. A real magic number.

    LOCK throttle TO 0.
    unlock throttle.
    unlock steering.
  LOG_INFO("Burn complete.").
    wait 2.
    // Remove this maneuver node
    remove nextnode.
}

// Function TO SET a Kerbal Alarm CLOCK alarm for the next maneuver node

// SET alarmTitle TO "Forgot TO SET".
// SET alarmNotes TO "".
FUNCTION SETKACAlarmForNextNode{
    PARAMETER KacAction.
    PARAMETER kacLeadTime.
    PARAMETER alarmTitle.
    PARAMETER alarmNotes.
    addAlarm("Manuver", NEXTNODE:TIME + kacleadtime + eta:nextnode < (burnTime / 2) - 60, alarmTitle, alarmNotes).
    // ADDONS:KAC:ALARMS:  KacAction
    }



// For long term travel radiation management. Needs to point away from the sun for protection.

FUNCTION antisun {
    // Calculate vector towards the Sun
    local sunVector is body("Sun"):position - ship:position.
    local awayFromSun is -sunVector:normalized.

    // Lock steering away from Sun
    return awayFromSun.

}
// https://www.reddit.com/r/Kos/comments/4kk0gd/coming_out_of_time_warp/
FUNCTION warpHelper {
    SET warp TO 1.
    WAIT until not ship:unpacked. // warp is actually engaged
    // WAIT until warp time
    SET warp TO 0.
    WAIT until ship:unpacked. // warp is actually disengaged
// vessel should imediately be responsive
}


FUNCTION AreoBrakeAssist{
    SET periapsisAlt TO SHIP:OBT:PERIAPSIS - SHIP:BODY:ATM:HEIGHT. // Altitude of periapsis above atmosphere
    SET atmoEntryAlt TO 90000. // Altitude at which the atmosphere starts for Eve
    SET velocityAtPeri TO VANG(SHIP:VELOCITY:SURFACE:NORTH, SHIP:VELOCITY:SURFACE:EAST).

    SET descentDistance TO periapsisAlt - atmoEntryAlt.

    SET descentTime TO descentDistance / velocityAtPeri.

    SET safetyMargin TO 0.1. // 10% safety margin
    SET adjustedDescentTime TO descentTime * (1 + safetyMargin).

}
set logging to true.
if(Logging) {
LOG "TIME" + "," +
 "SHIP:ALTITUDE" + "," +
 "TargetAltitude" + "," +
 "AltitudePitch" + "," +
 "GROUNDSPEED" + "," +
 "VERTICALSPEED" + "," +
 "AIRSPEED" + "," +
 "SpeedPitch" + "," +
 "SHIP:Q" + "," +
 "SHIP:SENSORS:PRES" + "," +
 "AirResistPitch" + "," +
 "SHIP:APOAPSIS" + "," +
 "AVAILABLETHRUST" + "," +
 "SHIP:MASS" + "," +
 "SHIP:WETMASS" + "," +
 "SHIP:DRYMASS" + "," +
 "PitchingSteer"
  to "0:/LaunchProfile.csv". 

  }.
  SET TargetAltitude TO 85000.
SET GravCst TO KERBIN:MU / KERBIN:RADIUS^2. 
SET TargetOrbitalSpeed TO 600000 * SQRT(GravCst/(600000+TargetAltitude)).
SET Staggincount TO 3.
Set PitchingSteer to 90.
SET AirResistPitch TO 90.
SET AdjustedThrottle TO 1.
SET SpeedPitch TO 90.
SET AltitudePitch TO 90.
SET GravityLoss TO 1.
SET DragCoef TO 1.
SET CrossSection TO 1.25.
SET Area TO constant:pi * (CrossSection/2)*(CrossSection/2).
SET DragCST TO DragCoef * Area.
SET RemainingSecs TO 400.
SET oneSecondsLater to TIME:SECONDS + 1.
LOCK RemainingSecs TO ((70000-SHIP:ALTITUDE)/MAX(0.0000001,SIN(PitchingSteer)))  /  MAX(0.0000001,SHIP:AIRSPEED).
LOCK AirEstimatedLoss TO (SHIP:Q * DragCST)/2 * RemainingSecs.
LOCK AirResistPitch TO MIN((AirEstimatedLoss*10),90).
LOCK SpeedPitch TO (100-(GROUNDSPEED/12)).
LOCK AltitudePitch TO (90 - ((SHIP:Altitude /60000 )*90)).
LOCK GravityLoss TO SIN(PitchingSteer)*AVAILABLETHRUST - SHIP:MASS * GravCst.
LOCK GravityEstimatedLoss TO GravityLoss*SHIP:MASS*(TargetOrbitalSpeed-GROUNDSPEED)/(AVAILABLETHRUST*MAX(0.0001,COS(PitchingSteer))).
LOCK LNPitch TO 90 - (90 *  LN(SHIP:ALTITUDE/TargetAltitude +1)).
//Loggin loop
WHEN TIME:SECONDS > oneSecondsLater THEN {
if(Logging) {
SET oneSecondsLater to TIME:SECONDS + 1.
LOG TIME:SECONDS + "," +
 SHIP:ALTITUDE + "," +
//  TargetAltitude + "," +
//  AltitudePitch + "," +
 GROUNDSPEED + "," +
 VERTICALSPEED + "," +
 AIRSPEED + "," +
 SpeedPitch + "," +
 SHIP:Q + "," +
//  SHIP:SENSORS:PRES + "," +
 AirResistPitch + "," +
 SHIP:APOAPSIS + "," +
 AVAILABLETHRUST + "," +
 SHIP:MASS + "," +
 SHIP:WETMASS + "," +
 SHIP:DRYMASS + "," +
 PitchingSteer
  to "0:/Launch.csv". 
  }.
RETURN TRUE.
}

//---------------------------------------------------------//
// ---------------------- STARTS HERE ---------------------//-------------------------
//---------------------------------------------------------//


WAIT until ship:unpacked.
SET realMissionTime to KUniverse:REALTIME.

//starts fillballons now becuase it takes a while.
fillBaloon().
RUN_MARQUEE(BUILD_ORDER(), 0.05, 6, FALSE).
PRINT"===============================".
PRINT"".
PRINT"" + SHIP:NAME + " SSTE Config".
PRINT"".
PRINT"===============================".
PRINT"".

LOG_STAGE("FILLING BALOONS").

LOG_INFO("Applying brakes and raising legs!").
brakes on.
TOGGLE AG1.

PRINTWelcome().

LOG_STAGE("WARNING: DON'T TOUCH SAS, RCS, OR STAGING.").
PRINT " ".
PRINT "Press any key to confirm you will die if you do that...".
TERMINAL:REVERSE.
UNTIL TERMINAL:input:haschar {
    TERMINAL:REVERSE.
    SET tcolor TO false.
    WAIT .3.
    TERMINAL:REVERSE.
    SET tcolor TO true.
}
if tcolor = true {
    TERMINAL:REVERSE.
}
STOP_MARQUEE().
//------------- no turning back--------
SAS OFF.
LOG_INFO("SAS off").
WAIT .2.
LOG_STAGE("LOCKING CONTROLS").
WAIT .2.
LOCK STEERING TO HEADING(90, .4, 0).
LOG_INFO("Yaw, pitch and roll locked").
WAIT .2.
LOG_INFO("Steering wheels locked").
WAIT .2.
LOG_INFO("Control surfaces locked").

// Find and turn off all hl10rudder parts which is all of the crafts aero control surface. They will slow us down while driving.
ControlSurfacesOff().
WAIT .2.

LOG_INFO("RCS on").
RCS ON.
WAIT .4.
LOG_STAGE("STARTING FAN SYSTEM!").
WAIT .2.
LOG_INFO("Opening intakes...").
WAIT .2.
LOG_INFO("Starting compressors...").
WAIT .2.
LOG_INFO("Starting nuclear reactor...").
WAIT .2.
TOGGLE ag2.
LOG_INFO("Powering up fans...").
// Reverse fans
TOGGLE ag4.
WAIT 1.5.
lvlBaloon().
// Throttle up and release the brakes
LOG_STAGE("BACK UP SEQUENCE").

LOG_INFO("Throttling up reverse fans!").
LOCK throttle TO 1.
WAIT .8.
LOG_INFO("Fan thrust is nominal").
wait .5.
BRAKES off.
LOG_INFO("Brakes released!").
wait .2.
LOG_INFO("And away we go!").
WAIT 1.
RUN_MARQUEE(BUILD_ORDER(), 0.05, 4, true).


// Driving part
SET TERMINAL:HEIGHT TO 20.
SET TERMINAL:WIDTH TO 49.
SET lstage TO 0.
LOCAL runway_hud_interval IS 0.2.
LOCAL runway_next_hud IS 0.
LOCAL runway_last_stage_logged IS -999.
until lstage >= 5 {
    SET myGroundSpeed TO VDOT(FACING:VECTOR, VELOCITY:SURFACE).
    SET radarAltitude TO alt:radar.//old way, use vessel.
    IF myGroundSpeed > 10 {
        STOP_MARQUEE().
    }
    if myGroundSpeed <= -0.05 {
        if current_heading_target >= 90{
            SET equals TO current_heading_target - 90.
            SET reverseHeading TO (90 -equals).
        }
        if current_heading_target < 90{
            SET equals TO ABS(current_heading_target - 90).
            SET reverseHeading TO (90 + equals).
        }
        LOCK WHEELSTEERING TO reverseHeading.
        LOCK STEERING TO heading(reverseHeading, .4, 0).
    } ELSE {
        LOCK WHEELSTEERING TO current_heading_target.
        LOCK STEERING TO HEADING(current_heading_target, .4, 0).
    }
    //Reverse thrusters (TO foward), hit brakes and fire jets. When sTOpped release brakes and go.
    if myGroundSpeed < -33.5 {
        SET lstage TO 1.
           if myGroundSpeed < -33.5 {
               SET lstage TO 2.
               lstage1().
            }
    }
 
    //flaps lvl 1 and fireworks
    //This means we hit the jump
    if radarAltitude > 4 and lstage = 2 {
        lstage2().
        rcs off.
        SET lstage TO 3.
        // Need buoyancy over balance during liftoff.
        fillBaloon().
    }.
    
    //flaps lvl 2 (takeoff)
    if myGroundSpeed > 100{
        
        IF FLAPSLVL < 2{
        TOGGLE ag5.
        SET FLAPSLVL TO 2.
        }
        
        SET lstage TO 4.
        //reenable control surfaces
        if myGroundSpeed > 108 {
            IF myGroundSpeed > 115{
                SET lstage TO 5.
            }
            enableCS().
             
        }
    }
    
  //--------------------   Steering Limiter for driving --------------------
    // Instead of tyring TO make pid adjustments at different speeds, I just limit
    // the maximum steering power as you go faster. Easy way TO make steering work
    // at all speeds with no pid.
    if ABS(myGroundSpeed) < 14 {
    SET AngSet TO 30.
    }ELSE {
        SET AngSet TO ABS(40 / ABS(myGroundSpeed)).
        FOR WHEEL IN WHEELS
        {
            WHEEL:GETMODULE("ModuleWheelSteering"):SETFIELD("steering angle limiter", AngSet).
        }
    }

  LOCAL stage_label IS "Launch stage: " + LSTAGE.
  IF lstage <= 0 {
    SET stage_label TO stage_label + ", Back it up!".
  } ELSE IF lstage <= 1 {
    SET stage_label TO stage_label + ", Fire Jets".
  } ELSE IF lstage <= 2 {
    SET stage_label TO stage_label + ", Finally Forward!".
  } ELSE IF lstage <= 3 {
    SET stage_label TO stage_label + ", Dem Duke Boys!".
  } ELSE IF lstage <= 4 {
    SET stage_label TO stage_label + ", DO OR DIE!".
  }.

  IF lstage <> runway_last_stage_logged {
    LOG_STAGE(stage_label).
    SET runway_last_stage_logged TO lstage.
  }.

  IF TIME:SECONDS >= runway_next_hud {
    SET runway_next_hud TO TIME:SECONDS + runway_hud_interval.
    CLEARSCREEN.
    PRINT stage_label.
    IF myGroundSpeed >= 0 {
      PRINT "Moving Forward >>>>>>>".
      PRINT "Target heading: " + ROUND(current_heading_target, 2).
    } ELSE IF myGroundSpeed < -.05 {
      PRINT "Moving Backwards <<<<<<.".
      PRINT "Target heading: " + ROUND(reverseHeading, 2).
    }.
    PRINT "Centerline deviation: " + ROUND(centerline_linear_deviation, 4).
    PRINT "Ground Speed: " + ROUND(myGroundSpeed, 2) + " m/s".
    PRINT "Wheel turn limit: " + ROUND(AngSet, 2).
    PRINT "Radar altitude: " + ROUND(radarAltitude, 2).
  }.

// Check if the vessel is off the ground
    // IF radarAltitude > 3 {
    //     PRINT "The vessel is off the ground.".
    // } ELSE {
    //     PRINT "The vessel is on the ground.".
    // }

    centerline_pid:update(time:seconds, centerline_linear_deviation).

    // if ship:geoposition:lng > RUNWAY_EAST_THRESHOLD_LNG {
    //     SET lstage TO 3.
    // }
}
CLEARSCREEN.
rcs on.
LOG_STAGE("Launch stage: 4, DO OR DIE!").
RUN_MARQUEE(BUILD_ORDER(), 0.2, 4, FALSE).
WAIT.1.
UNLOCK ALL.  // Releases ALL LOCKs on steering and other vars not using anymore.
LOCK THROTTLE TO 1.
SETPID("pitch", 5, 0.1, 4.5).
SETPID("roll", 3, 0.2, 2.5).
SETPID("yaw", 3, 0.3, 3).
// this number is very important. At low speeds it gets tippy and this will bad things.
SET STEERINGMANAGER:MAXSTOPPINGTIME TO 3.

LOG_INFO("yaw, 3, 0.3, 3").
// LOCK STEERING TO HEADING(90.2, 0, 0).

WAIT UNTIL alt:radar > 5.
SETPID("pitch", 1, 0.1, 3).
LOCK STEERING TO HEADING(90.2, 4, 0).
LOG_INFO("RADAR > 5").
wait 0.5.
CLEARSCREEN.
STOP_MARQUEE().
LOG_STAGE("Launch stage: 5, LIFTOFF, WE HAVE LIFTOFF!!!").
UNLOCK WHEELSTEERING.
LOG_INFO("Unlock wheel steering").

// PrintTimeStamped("LOCK TO HEADING(90.2, -1, 0).").



// Hardest part: Take off, Lose alt, tailstrike bounce, lose alt again, tail strike the water.
// Then finally gain alt or die in the water.


// Wait till we clear the first hill after runway.
WAIT UNTIL alt:radar > 17.
LOG_INFO("radar > 17: PULL UP HARD").
//Pull up very hard
SETPID("pitch", 5, 0.1, 5).
//90.2 because the runway is .4 and we want go 0 so I split the difference to avoid too much yaw at takeoff
LOCK STEERING TO HEADING(90.2, 10, 0).
// PrintTimeStamped("LOCK TO HEADING(90.2, 14, 0).").
WAIT .8.
LOG_INFO("Gear up!").
GEAR OFF.
WAIT 5.
// Clear water first.
WAIT UNTIL SHIP:altitude > 20.
// Level baloon now so we can stop pulling up later...
lvlBaloon().

// Finaly gaining alt
WAIT UNTIL SHIP:verticalspeed > 5.
LOG_INFO("vertical speed > 5").
LOG_INFO("Lock to heading (90, 7.25, 0)").
LOCK STEERING TO HEADING(90, 7.25, 0).
WAIT UNTIL SHIP:AIRSPEED > 150.
// make damn sure the flaps are up.
// had problems, probably don't need this anymore.
LOG_INFO("Flaps up").
TOGGLE AG6.
WAIT.1.
TOGGLE AG6.
WAIT.1.
TOGGLE AG6.
WAIT UNTIL SHIP:AIRSPEED > 160.
// toggle ag3.//
TOGGLE AG6.
LOG_INFO("Flaps up").
list engines in engList.
    //Saves gas?
    FOR  eng in engList {
        if eng:NAME = "turboFanEngine" {
        set eng:thrustlimit to 0.
        }
    }
LOG_INFO("Cutting jet thrust to save gas").
WAIT UNTIL SHIP:AIRSPEED > 170.
LOG_INFO("Flaps up").
TOGGLE AG6.
WAIT UNTIL SHIP:AIRSPEED > 180.

TOGGLE AG6.
LOG_INFO("Flaps up").
WAIT UNTIL SHIP:AIRSPEED > 190.
TOGGLE AG6.
LOG_INFO("Flaps up, damn it!").

//225 m/s is enough to start climbing.
//this will keep you climbing without going too fast to save on fuel.

WAIT UNTIL SHIP:AIRSPEED > 200.
LOG_INFO("AIRSPEED > 200: HEADING(90, 10, 0)").
LOCK STEERING TO HEADING(90, 10, 0).
WAIT UNTIL alt:radar > 500.
SET STEERINGMANAGER:MAXSTOPPINGTIME TO 1.5.

SET CLIMB TO 0.
UNTIL CLIMB > 0{
    SET TOTALTHRUST TO SHIP:THRUST.
    WAIT .3.
    CLEARSCREEN.
  LOG_STAGE("Launch stage: 6, Slow Climb.").
    PRINT "Airspeed: " + ROUND(AIRSPEED, 2) + " m/s".
    PRINT "Vertical Speed: " + ROUND(SHIP:verticalspeed, 2) + " m/s".
    list engines in engList.
    //Saves gas?
    FOR  eng in engList {
        if eng:NAME = "turboFanEngine" {
        set eng:thrustlimit to (ship:altitude - 1000) / 45.
        }
    }
    PRINT "Total Thrust: " + ROUND(TOTALTHRUST) + " kN".
    IF SHIP:AIRSPEED > 275{
        LOCK STEERING TO HEADING(90, 18, 0).
    }
    IF SHIP:AIRSPEED > 250{
     LOG_INFO("Lock to heading (90, 13, 0)").
       // LOCK STEERING TO HEADING(90, 25, 0).
       LOCK STEERING TO HEADING(90, 16, 0).
    }
    IF SHIP:AIRSPEED > 240 AND SHIP:AIRSPEED < 250{
     LOG_INFO("Lock to heading (90, 11, 0)").
       LOCK STEERING TO HEADING(90, 15, 0).
       // LOCK STEERING TO HEADING(90, 11, 0).
    }
     IF SHIP:AIRSPEED < 240 AND SHIP:AIRSPEED > 230 {
     LOG_INFO("Lock to heading (90, 9, 0)").
       LOCK STEERING TO HEADING(90, 14, 0).
       // LOCK STEERING TO HEADING(90, 9, 0).
    }
     IF SHIP:AIRSPEED < 230 AND SHIP:AIRSPEED > 220{
     LOG_INFO("Lock to heading (90, 7, 0)").
       LOCK STEERING TO HEADING(90, 11, 0).
       // LOCK STEERING TO HEADING(90, 7, 0).
    }
    IF SHIP:AIRSPEED < 220{
     LOG_INFO("Lock to heading (90, 6, 0)").
       LOCK STEERING TO HEADING(90, 9, 0).
       // LOCK STEERING TO HEADING(90, 6, 0).
    }

 IF ship:altitude > 4500 {
    //save rcs
    rcs off.
   LOG_INFO("RCS off").
    SET CLIMB TO 1.
 }
}

// We have some alt, now we need speed. A lot of speed. This will take a long time...
CLEARSCREEN.
list engines in engList.
    FOR  eng in engList {
        if eng:NAME = "turboFanEngine" {
        set eng:thrustlimit to 100.
        }
    }
LOG_STAGE("Launch stage: 7, Gain Speed!").
SETPID("pitch", 2, 0.1, 3).
LOCK STEERING TO HEADING(90, 7.5, 0).
LOG_INFO("Heading (90, 7.5, 0)").

// Turn fans off and close intakes when they become useless.
WAIT UNTIL altitude > 6000.
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
//90747 liquid fuel needed for rockets. <<< not no more

//https://github.com/lordcirth/kOS-Public/blob/master/maxq.ks
until SHIP:LIQUIDFUEL <= 91300 {
    WAIT .5.
    CLEARSCREEN.
  LOG_STAGE("Launch stage: 7, Gain Speed!").
    PRINT "Airspeed: " + ROUND(AIRSPEED, 2) + " m/s".
    PRINT "Vertical Speed: " + ROUND(SHIP:verticalspeed, 2) + " m/s".
    PRINT("Liquid Fuel Left: " + ROUND(SHIP:LIQUIDFUEL)).
    SET TOTALTHRUST TO 0.
    FOR eng in engList {
        SET TOTALTHRUST TO TOTALTHRUST + ENG:THRUST.
    }
    PRINT "Total Thrust: " + ROUND(TOTALTHRUST) + " kN".
    ship:altitude.
    if SHIP:LIQUIDFUEL <= 93500{
        PRINT "|=== GET READY TO BURN! In: " + ROUND((SHIP:LIQUIDFUEL - 90747)).
        TERMINAL:REVERSE.
    }
}
RUN_MARQUEE(BUILD_ORDER(), 0.06, 6, TRUE).
// TERMINAL:REVERSE.
// WAIT .2.
// TERMINAL:REVERSE.
LOG_INFO("Time to burn").
rcs on.
LOG_INFO("RCS on").
SETPID("pitch", 5, 0.1, 5.5).
// PrintTimeStamped("PID: 5, 0.1, 6").
LOCK STEERING TO HEADING(90, 29, 0).
LOG_INFO("Heading (90, 29, 0)").
// Start pitching up then fire rockets.
// We start the burn at 91300 because the jets will still be working a bit longer.
WAIT until SHIP:LIQUIDFUEL <= 90747.
STOP_MARQUEE().
WAIT .5.
RUN_MARQUEE(BUILD_ORDER(), 0.35, 2).
LOG_STAGE("Launch stage: 8, Pitch and BURN!").
stage.
WAIT 3.
rcs off.

//---------------------------- Burn heading to space! -------------------------//
SET CLIMB2 TO 0.
SET JETSDONE TO false.
STOP_MARQUEE().
UNTIL CLIMB2 > 0{
    SET TOTALTHRUST TO 0.
    WAIT .3.
    CLEARSCREEN.
  LOG_STAGE("Launch stage: 9, BURN TO Space!").
    PRINT "Alt: " + ROUND(ship:altitude).
    PRINT "Airspeed: " + ROUND(AIRSPEED, 2) + " m/s".
    PRINT "Vertical Speed: " + ROUND(SHIP:verticalspeed, 2) + " m/s".

    PRINT "Jet Shutdown: " + JETSDONE.
    list engines in engList.
    FOR eng in engList {
        SET TOTALTHRUST TO TOTALTHRUST + ENG:THRUST.
    }
    PRINT "Thrust: " + ROUND(TOTALTHRUST) + " kN".
    IF SHIP:OBT:ETA:APOAPSIS > 40{
        LOCK STEERING TO HEADING(90, 25, 0).
    }
    IF SHIP:OBT:ETA:APOAPSIS > 47{
        LOCK STEERING TO HEADING(90, 20, 0).
    }
    IF JETSDONE = FALSE {
    monitorEngines().
    WAIT .3.
    }
    IF SHIP:OBT:ETA:APOAPSIS > 51.5{
        LOCK STEERING TO PROGRADE.
    }
    IF SHIP:apoapsis >= 74950{
        LOCK THROTTLE TO 0.
        SET CLIMB2 TO 1.
    }
}
CLEARSCREEN.

//------------------------------- MAKE ORBIT------------------------------//

LOG_STAGE("Launch stage: 10, Coast TO Space!").
WAIT until ship:altitude >= 70100.

LOG_STAGE("Launch stage: 11, Blimps in SPAAAAAACE!").
ControlSurfacesOff().
LOG_INFO("Aero control surfaces off").
WAIT.5.

// mediumCS().
LOG_INFO("Set SteeringManager max stopping time to 3").
SETPID("pitch", 2, 0.2, 3).
SETPID("roll", 2, 0.2, 3).
SETPID("yaw", 2, 0.2, 3).
SET STEERINGMANAGER:MAXSTOPPINGTIME TO 3.
WAIT .5.
//Calculate when it is time TO burn, SET an alarm and do it.
LOG_INFO("Creating circularization burn").
CIRCLE().
WAIT 1.
LOG_INFO("Calculating burn time").
SET burnTime TO calculateBurnTime().
WAIT 1.
// SETKACAlarmForNextNode("KillWarp",15, "Burn TO Orbit!", "This burn was brought to you by Snacky Smores. Ride the Walrus!").
// PrintTimeStamped("KAC alarm SET for 15 seconds out").
WAIT .5.
EXECUTE_MANEUVER().
WAIT 2.
SET timeToOrbit to KUniverse:REALTIME - realMissionTime.
PRINT "It took "+ round(timeToOrbit) / 60 + "real life mins to orbit.".
// WAIT UNTIL SHIP:OBT:ETA:APOAPSIS < 10 OR SHIP:OBT:ETA:PERIAPSIS < 20.

SET ROLL_ANGLE TO 180.
SET STEERINGMANAGER:MAXSTOPPINGTIME TO 4.
WAIT .5.
local awayFromSun is antisun().
lock steering to lookdirup(-awayFromSun, ship:up:vector).
WAIT 3.


//------------------- IN ORBIT ------------------------------------//
wemadeit().
WAIT 5.

LOG_STAGE("Opening up ship!").
//open hanger
WAIT 2.
LOG_INFO("Opening lower hangar bay door").
TOGGLE AG43.
WAIT 3.
//lights on
LOG_INFO("Turning on flood lights").
WAIT .2.
TOGGLE AG14.
LOG_INFO("Turning on accent lights").
WAIT .2.
TOGGLE AG15.
LOG_INFO("Turning on gondola cab lights").
WAIT .2.
TOGGLE AG17.
LOG_INFO("Turning on habitat lights").
WAIT .2.
TOGGLE AG13.
WAIT 4.
LOG_INFO("Unlocking solar panel motors and powering on").
//unlock solar motors
TOGGLE AG39.
WAIT 1.
//deply side solar.
LOG_INFO("Deploying side solar panels").
TOGGLE AG30.
WAIT 5.
//unlock boom motors
LOG_INFO("Unlocking and powering on boom motors").

TOGGLE AG19.
TOGGLE AG20.
WAIT 1.
//depoly boom
LOG_INFO("Deploying boom").
TOGGLE AG22.
WAIT 9.
LOG_INFO("Deploying boom solar panels").
TOGGLE AG31.
wait 6.
//LOCK motors
LOG_INFO("Locking motors").

TOGGLE AG19.
TOGGLE AG20.
WAIT 2.
//depoly comms
LOG_INFO("Deploying communications equipment").

TOGGLE AG24.
//deploy boom solar

WAIT 2.
// depoly science
// Cooling System
LOG_INFO("Deploying cooling system").
WAIT 2.
TOGGLE AG36.

//---------------------Set Course for Eve!-------------------------//
WAIT 1.
addons:astrogator:create(eve).
addons:astrogator:makenode("Eve").
WAIT 1.
SET burnTime TO calculateBurnTime().
WAIT 1.
SETKACAlarmForNextNode("KillWarp",600, "Burn to Eve!!!", "This burn was brought to you by Snacky Smores. Ride the Walrus!").
WAIT 1.
EXECUTE_MANEUVER().
PLAN_AND_EXECUTE_EVE_CORRECTION().
WAIT 1.
local awayFromSun is antisun().
lock steering to lookdirup(-awayFromSun, ship:up:vector).



// ===============================================
// Eve MCC + Precise Node Burn + Entry Reserve Burn + Multi-Pass Aerobrake
// kOS syntax correct (DECLARE FUNCTION / PARAMETER / periods).
// ===============================================

// -------- Config --------
SET DESIRED_PE_ALT TO 82000.      // target Eve periapsis (m)
SET TUNE_INCLINATION TO TRUE.     // lightly penalize Eve patch inclination in tuning
SET EVE_ATM_TOP TO 90000.         // Eve atmosphere top (m)
SET DV_RESERVE TO 50.             // leave ~this much Δv after entry burn (m/s)
SET RETRO_SURFACE TO FALSE.       // TRUE: surface retrograde, FALSE: orbital retrograde

// -------- Constants --------
SET G0 TO 9.80665.
SET DENS_LF TO 0.005.
SET DENS_OX TO 0.005.
SET DENS_MP TO 0.004.
SET DENS_XE TO 0.0001.
SET DENS_SF TO 0.0075.

// ===============================================
// Warp helpers
// ===============================================
DECLARE FUNCTION WARP_TO_NODE {
  PARAMETER nd.
  PARAMETER lead IS 10.

  IF NOT HASNODE { PRINT "No node for warp_to_node.". RETURN. }

  IF nd:ETA <= lead { SET WARP TO 0. RETURN. }

  UNTIL nd:ETA <= lead {
    LOCAL e IS nd:ETA - lead.
    LOCAL w IS 0.
    IF e > 6*HOUR       { SET w TO 6. }
    ELSE IF e > 1*HOUR  { SET w TO 5. }
    ELSE IF e > 20*MINUTE { SET w TO 4. }
    ELSE IF e > 3*MINUTE  { SET w TO 3. }
    ELSE IF e > 40        { SET w TO 2. }
    ELSE IF e > 10        { SET w TO 1. }
    ELSE { SET w TO 0. }
    IF WARP <> w { SET WARP TO w. }
    WAIT 0.2.
  }
  SET WARP TO 0.
}.

DECLARE FUNCTION WARP_TO_ALT {
  PARAMETER targetAlt.
  PARAMETER leadAlt IS 5000.

  IF SHIP:ALTITUDE <= targetAlt + leadAlt { SET WARP TO 0. RETURN. }

  UNTIL SHIP:ALTITUDE <= targetAlt + leadAlt {
    LOCAL a IS SHIP:ALTITUDE - (targetAlt + leadAlt).
    LOCAL w IS 0.
    IF a > 2*SHIP:BODY:RADIUS { SET w TO 6. }
    ELSE IF a > 500000        { SET w TO 5. }
    ELSE IF a > 150000        { SET w TO 4. }
    ELSE IF a > 40000         { SET w TO 3. }
    ELSE IF a > 5000          { SET w TO 2. }
    ELSE                      { SET w TO 1. }
    IF WARP <> w { SET WARP TO w. }
    WAIT 0.2.
  }
  SET WARP TO 0.
}.

// ===============================================
// DV / thrust utilities
// ===============================================
DECLARE FUNCTION EFFECTIVE_ISP {
  LOCAL thrustSum IS 0.
  LOCAL ispWeighted IS 0.
  LIST ENGINES IN engs.
  FOR e IN engs {
    // Use engines that can make thrust now (in vac near Eve edge this is fine)
    IF e:MAXTHRUST <= 0 { CONTINUE. }
    SET thrustSum TO thrustSum + e:MAXTHRUST.
    SET ispWeighted TO ispWeighted + (e:ISP * e:MAXTHRUST).
  }
  IF thrustSum <= 0 { RETURN 0. }
  RETURN ispWeighted / thrustSum.
}.

DECLARE FUNCTION PROP_MASS {
  LOCAL m IS 0.
  SET m TO m + SHIP:RESOURCES["LiquidFuel"]:AMOUNT * DENS_LF.
  SET m TO m + SHIP:RESOURCES["Oxidizer"]:AMOUNT * DENS_OX.
  SET m TO m + SHIP:RESOURCES["MonoPropellant"]:AMOUNT * DENS_MP.
  SET m TO m + SHIP:RESOURCES["XenonGas"]:AMOUNT * DENS_XE.
  SET m TO m + SHIP:RESOURCES["SolidFuel"]:AMOUNT * DENS_SF.
  RETURN m.
}.

DECLARE FUNCTION DV_REMAINING {
  LOCAL isp IS EFFECTIVE_ISP().
  IF isp <= 0 { RETURN 0. }
  LOCAL m0 IS SHIP:MASS.
  LOCAL mp IS PROP_MASS().
  LOCAL mdry IS m0 - mp.
  IF mdry <= 0 { RETURN 0. }
  RETURN isp * G0 * LN(m0 / mdry).
}.

DECLARE FUNCTION CALC_BURN_TIME {
// Most-accurate analytic burn time for a node at (assumed) steady throttle.
// Uses per-engine AVAILABLETHRUST and ISP at current conditions.
// t = (m0 * ve / F) * (1 - exp(-Δv / ve)),  where ve = g0 * Isp_eq
// Isp_eq = (Σ Fi) / (Σ Fi / Ispi)

  PARAMETER nd IS NEXTNODE.
  PARAMETER myThrottle IS 1.0.      // 0.0..1.0 (1.0 = full)

  IF nd = 0 { RETURN 0. }.
  LOCAL dv IS nd:DELTAV:MAG.
  IF dv <= 0 { RETURN 0. }.

  // Collect active engines and compute thrust-weighted Isp
  LIST ENGINES IN es.
  LOCAL FkN IS 0.                  // Σ Fi (kN)
  LOCAL sumFoverIsp IS 0.          // Σ (Fi / Ispi) (kN/s)

  FOR e IN es {
    IF e:IGNITION AND NOT e:FLAMEOUT {
      LOCAL Fi IS e:AVAILABLETHRUST * myThrottle.   // kN
      LOCAL Ispi IS e:ISP.                         // s (current atm)
      IF Fi > 0 AND Ispi > 0 {
        SET FkN TO FkN + Fi.
        SET sumFoverIsp TO sumFoverIsp + Fi / Ispi.
      }
    }
  }

  IF FkN <= 0 OR sumFoverIsp <= 0 { RETURN 0. }.

  // Effective exhaust velocity
  LOCAL g0 IS 9.82.                         // m/s^2
  LOCAL IspEq IS FkN / sumFoverIsp.         // s
  LOCAL ve IS IspEq * g0.                   // m/s

  // Mass & force in consistent units
  LOCAL F   IS FkN * 1000.                  // N
  LOCAL m0  IS SHIP:MASS * 1000.            // kg

  // Analytic burn time under constant F and Isp
  // t = (m0 * ve / F) * (1 - exp(-dv / ve))
  LOCAL eConst IS CONSTANT:E.
  RETURN (m0 * ve / F) * (1 - eConst ^ (-dv / ve)).
}.



// ===============================================
// Precise node executor (auto-warp + Δv feedback)
// ===============================================
DECLARE FUNCTION EXECUTE_MANEUVER {
  PARAMETER execNode IS NEXTNODE.
  PARAMETER autoWarp IS FALSE.
  PARAMETER SETTLE IS 15.
  PARAMETER USE_RCS_TRIM IS TRUE.

  IF NOT HASNODE {
    PRINT "ERROR: no maneuver node.".
    RETURN.
  }.
  
  IF execNode:DELTAV:MAG <= 0 {
    PRINT "Node Δv is zero; removing.".
    REMOVE execNode.
    RETURN.
  }
  IF execNode:ETA < -2 {
    PRINT "Node time passed.".
    RETURN.
  }

  LOCAL tburn IS CALC_BURN_TIME(execNode).
  IF tburn <= 0 { PRINT "ERROR: zero thrust or Δv.". RETURN. }

  IF autoWarp = TRUE { WARP_TO_NODE(execNode, tburn/2 + SETTLE). }

  // Recompute immediately before settle
  SET tburn TO CALC_BURN_TIME(execNode).
  IF tburn <= 0 { PRINT "ERROR: cannot compute burn time.". RETURN. }

  LOCK STEERING TO execNode:BURNVECTOR.
  SET STEERINGMANAGER:MAXSTOPPINGTIME TO 0.5.
  SET STEERINGMANAGER:ROLLCONTROL TO 0.
  SAS OFF.
  WAIT UNTIL execNode:ETA <= (tburn/2).

  // Burn with Δv feedback
  LOCK THROTTLE TO 1.
  UNTIL execNode:DELTAV:MAG < 10 {
    LOCK STEERING TO execNode:BURNVECTOR.
    WAIT 0.05.
  }
  LOCK THROTTLE TO 0.2.
  UNTIL execNode:DELTAV:MAG < 0.5 {
    LOCK STEERING TO execNode:BURNVECTOR.
    WAIT 0.05.
  }
  LOCK THROTTLE TO 0.05.
  UNTIL execNode:DELTAV:MAG < 0.05 {
    LOCK STEERING TO execNode:BURNVECTOR.
    WAIT 0.05.
  }
  LOCK THROTTLE TO 0.

  // Optional micro-trim with RCS
  IF USE_RCS_TRIM AND execNode:DELTAV:MAG > 0.02 {
    LOCAL savedRcs IS RCS.
    RCS ON.
    UNTIL execNode:DELTAV:MAG < 0.02 {
      SET SHIP:CONTROL:FORE TO 0.1. WAIT 0.1.
      SET SHIP:CONTROL:FORE TO 0.0. WAIT 0.1.
    }
    IF NOT savedRcs { RCS OFF. }
  }

  UNLOCK THROTTLE.
  UNLOCK STEERING.

  LOCAL rem IS execNode:DELTAV:MAG.
  REMOVE execNode.
  PRINT "Node burn complete. Remaining Δv on node: " + ROUND(rem,3) + " m/s".
}.

// ===============================================
// MCC planner/tuner
// ===============================================
DECLARE FUNCTION FIND_EVE_PATCH {
  LOCAL res IS 0.
  FOR p IN SHIP:ORBIT:PATCHES {
    IF p:BODY:NAME = "Eve" {
      SET res TO p.
      BREAK.
    }
  }
  RETURN res.
}.

DECLARE FUNCTION NODE_COST {
  PARAMETER nd.
  LOCAL ep IS FIND_EVE_PATCH().
  IF ep = 0 { RETURN 1.0E12. }
  LOCAL peAlt IS ep:PERIAPSIS - ep:BODY:RADIUS.
  LOCAL peErr IS ABS(peAlt - DESIRED_PE_ALT).
  IF NOT TUNE_INCLINATION { RETURN peErr. }
  LOCAL incDeg IS ep:INCLINATION.
  RETURN peErr + (incDeg * 1500).
}.

DECLARE FUNCTION TUNE_NODE {
  PARAMETER nd.
  PARAMETER step IS 4.0.
  PARAMETER minStep IS 0.05.

  LOCAL bestDV IS nd:DELTAV.
  LOCAL bestCost IS NODE_COST(nd).

  UNTIL step < minStep {
    LOCAL improved IS FALSE.

    LOCAL dirs IS LIST().
    dirs:ADD(V(step,0,0)).   // +prograde
    dirs:ADD(V(-step,0,0)).  // -prograde
    dirs:ADD(V(0,step,0)).   // +radial
    dirs:ADD(V(0,-step,0)).  // -radial
    dirs:ADD(V(0,0,step)).   // +normal
    dirs:ADD(V(0,0,-step)).  // -normal

    FOR d IN dirs {
      SET nd:DELTAV TO nd:DELTAV + d.
      WAIT 0.
      LOCAL c IS NODE_COST(nd).
      IF c < bestCost {
        SET bestCost TO c.
        SET bestDV TO nd:DELTAV.
        SET improved TO TRUE.
      }
      SET nd:DELTAV TO nd:DELTAV - d.
      WAIT 0.
    }

    IF improved {
      SET nd:DELTAV TO bestDV.
      WAIT 0.
    } ELSE {
      SET step TO step / 2.
    }
  }
}.

DECLARE FUNCTION PLAN_AND_EXECUTE_EVE_CORRECTION {
  // Ensure we're in Sun SOI
  WAIT UNTIL SHIP:BODY:NAME = "Sun".

  // Node at mid-transfer if Eve is next patch; else fallback to +30 min
  LOCAL etaMid IS 1800.
  // This access assumes patched conics present; if not, fallback holds.
  IF SHIP:ORBIT:NEXTPATCH:BODY:NAME = "Eve" {
    SET etaMid TO MAX(SHIP:ORBIT:NEXTPATCH:ETA * 0.5, 1800).
  }

  LOCAL tNode IS TIME:SECONDS + etaMid.
  LOCAL n IS NODE(tNode, 0, 0, 0).
  ADD n.

  TUNE_NODE(n).

  // If still no Eve encounter, seed small nudge then re-tune
  IF FIND_EVE_PATCH() = 0 {
    LOCAL seeds IS LIST(V(20,0,0), V(-20,0,0), V(0,0,20), V(0,0,-20)).
    FOR s IN seeds {
      SET n:DELTAV TO n:DELTAV + s.
      WAIT 0.
      IF FIND_EVE_PATCH() <> 0 { BREAK. }
      SET n:DELTAV TO n:DELTAV - s.
      WAIT 0.
    }
    TUNE_NODE(n).
  }

  LOCAL ep IS FIND_EVE_PATCH().
  IF ep = 0 {
    PRINT "Warning: no Eve encounter predicted after tuning.".
  } ELSE {
    LOCAL peAlt IS ep:PERIAPSIS - ep:BODY:RADIUS.
    PRINT "Planned Eve Pe ≈ " + ROUND(peAlt/1000,1) + " km, Inc ≈ " + ROUND(ep:INCLINATION,2) + "°.".
  }

  EXECUTE_MANEUVER(n, 5, TRUE).
}.

// ===============================================
// Arrival: burn to reserve at edge of atmo, then aerobrake repeatedly
// ===============================================
DECLARE FUNCTION BURN_TO_RESERVE_AT_ENTRY {
  PARAMETER reserveDV IS DV_RESERVE.
  PARAMETER atmTop IS EVE_ATM_TOP.

  WARP_TO_ALT(atmTop).

  IF RETRO_SURFACE {
    LOCK STEERING TO -SHIP:VELOCITY:SURFACE.
  } ELSE {
    LOCK STEERING TO -SHIP:VELOCITY:ORBIT.
  }
  SET STEERINGMANAGER:MAXSTOPPINGTIME TO 0.5.
  SET STEERINGMANAGER:ROLLCONTROL TO 0.
  SAS OFF.
  WAIT 2.

  IF EFFECTIVE_ISP() <= 0 OR SHIP:AVAILABLETHRUST <= 0 {
    PRINT "No usable engines for entry burn.".
    UNLOCK STEERING.
    RETURN.
  }

  LOCK THROTTLE TO 1.
  UNTIL DV_REMAINING() <= reserveDV OR SHIP:ALTITUDE < (atmTop - 1000) {
    IF RETRO_SURFACE {
      LOCK STEERING TO -SHIP:VELOCITY:SURFACE.
    } ELSE {
      LOCK STEERING TO -SHIP:VELOCITY:ORBIT.
    }
    WAIT 0.05.
  }

  LOCK THROTTLE TO 0.2.
  UNTIL DV_REMAINING() <= reserveDV {
    WAIT 0.05.
    IF SHIP:ALTITUDE < (atmTop - 200) { BREAK. }
  }

  LOCK THROTTLE TO 0.
  UNLOCK THROTTLE.
  UNLOCK STEERING.

  PRINT "Entry cut at Δv ≈ " + ROUND(DV_REMAINING(),1) + " m/s (target " + reserveDV + ").".
}.

DECLARE FUNCTION MULTIPASS_AEROBRAKE_TO_SUBORBITAL {
  PARAMETER atmTop IS EVE_ATM_TOP.
  PARAMETER maxPasses IS 20.

  LOCAL pass IS 0.

  UNTIL (SHIP:APOAPSIS - SHIP:BODY:RADIUS) < atmTop OR pass >= maxPasses {
    SET pass TO pass + 1.
    PRINT "Aerobrake pass " + pass + " ...".

    IF SHIP:ALTITUDE > atmTop + 2000 {
      WARP_TO_ALT(atmTop).
    }

    IF RETRO_SURFACE {
      LOCK STEERING TO -SHIP:VELOCITY:SURFACE.
    } ELSE {
      LOCK STEERING TO -SHIP:VELOCITY:ORBIT.
    }
    SET STEERINGMANAGER:MAXSTOPPINGTIME TO 0.5.
    SET STEERINGMANAGER:ROLLCONTROL TO 0.
    SAS OFF.

    WAIT UNTIL SHIP:ALTITUDE < atmTop.
    WAIT UNTIL SHIP:ALTITUDE > atmTop AND SHIP:ORBIT:RADIALVELOCITY > 0.

    UNLOCK STEERING.

    LOCAL apAlt IS (SHIP:APOAPSIS - SHIP:BODY:RADIUS).
    PRINT "After pass " + pass + ": Ap = " + ROUND(apAlt/1000,1) + " km".
    IF apAlt > atmTop {
      WARP_TO_ALT(atmTop + 20000).
    }
  }

  IF (SHIP:APOAPSIS - SHIP:BODY:RADIUS) < atmTop {
    PRINT "Sub-orbital achieved.".
  } ELSE {
    PRINT "Max passes reached.".
  }
}.

DECLARE FUNCTION EVE_ENTRY_AND_AEROBRAKE {
  BURN_TO_RESERVE_AT_ENTRY(DV_RESERVE, EVE_ATM_TOP).
  MULTIPASS_AEROBRAKE_TO_SUBORBITAL(EVE_ATM_TOP, 20).
}.

// ===============================================
// Usage (examples):
// RUN PLAN_AND_EXECUTE_EVE_CORRECTION.
// Later, on arrival:
// RUN EVE_ENTRY_AND_AEROBRAKE.
// ===============================================



//------------------Prepare to capture.---------------------------//
// put everything away and turn on nukes. get a little rcs.
//--------------------Capture and areobrake-----------------------//
// probably come in retrograde and burn then turn around real quick
// to hit the atmo leaving 50 dv left for adjustments on the final pass
//---------------------Entry--------------------------------------//
// ag8 for pitch with airbrake. use airbrakes as brakes until pitching too much
// flip over, start fans, fly up to stall, eject heat shield. 
//---------------------Find a place to land-----------------------//
// Just find a flat place or water.
//----------------------Land--------------------------------------//
// 
//-------------------Open up the craft for long journey-----------//
  // at least make a script for this part

// //open hanger
// TOGGLE AG43.
// //lights on
// TOGGLE AG14.
// TOGGLE AG15.
// TOGGLE AG17.
// TOGGLE AG13.
// //unLOCK solor moTOrs
// TOGGLE AG39.
// //deply side solar.
// TOGGLE AG30.
// //unLOCK moTOrs
// TOGGLE AG19.
// TOGGLE AG20.
// //depoly boom
// TOGGLE AG22.
// //LOCK moTOrs
// // TOGGLE AG.
// //depoly comms
// TOGGLE AG24.
// // point away from sun

// // Cooling System
// TOGGLE AG36.


LOG_STAGE("Opening up ship!").
//open hanger
WAIT 2.
LOG_INFO("Opening lower hangar bay door").
TOGGLE AG43.
WAIT 3.
//lights on
LOG_INFO("Turning on flood lights").
WAIT .2.
TOGGLE AG14.
LOG_INFO("Turning on accent lights").
WAIT .2.
TOGGLE AG15.
LOG_INFO("Turning on gondola cab lights").
WAIT .2.
TOGGLE AG17.
LOG_INFO("Turning on habitat lights").
WAIT .2.
TOGGLE AG13.
WAIT 4.
LOG_INFO("Unlocking solar panel motors and powering on").
//unLOCK solor moTOrs
TOGGLE AG39.
WAIT 1.
//deply side solar.
LOG_INFO("Deploying side solar panels").
TOGGLE AG30.
WAIT 5.
//unLOCK boom moTOrs
LOG_INFO("Unlocking and powering on boom motors").

TOGGLE AG19.
TOGGLE AG20.
WAIT 1.
//depoly boom
LOG_INFO("Deploying boom").
TOGGLE AG22.
WAIT 9.
LOG_INFO("Deploying boom solar panels").
TOGGLE AG31.
wait 6.
//LOCK moTOrs
LOG_INFO("Locking motors").

TOGGLE AG19.
TOGGLE AG20.
WAIT 2.
//depoly comms
LOG_INFO("Deploying communications equipment").

TOGGLE AG24.
//deploy boom solar

WAIT 2.
// depoly science
// Cooling System
LOG_INFO("Deploying cooling system").
WAIT 2.
TOGGLE AG36.


WAIT until JETSDONE = false.















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
}
