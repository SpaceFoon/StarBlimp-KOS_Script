


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
    + padZ(ts:YEAR - 1) + "-" // subtracts 1 TO get years elapsed, not game year
    + padZ(ts:DAY - 1,3) + " " // subtracts 1 TO get days elapsed, not day of year. What is the 3 for?
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
    PrintTimeStamped(axis + " PID settings updated:").
    PRINT "  Kp: " + Kp.
    PRINT "  Ki: " + Ki.
    PRINT "  Kd: " + Kd.
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
        if PART:NAME = "hl10rudder" {
            // PRINT "Turning off part: " + PART:NAME.
            // Assuming we need TO SET the control surface  authority TO zero
            PART:GETMODULE("ModuleControlSurface"):SETFIELD ("authority limiter", 0).
        }
    }
}

//Function TO shut down engines and intakes properly when they are out of juice.
FUNCTION monitorEngines {

    list engines in engList.  //list of all engines
    FOR  eng in engList {
        if eng:NAME = "WBILargeElectricPart"{
            if eng:ignition = true and PROPSDONE = FALSE{
                PRINT "Engine: " + eng:NAME + " Thrust: " + ROUND(eng:THRUST) + " kN".
                if eng:THRUST < 7 {
                    PRINT "Engine thrust TOo low, shutting down: " + eng:NAME.
                    eng:SHUTDOWN().
                    SET PROPSDONE TO TRUE.
                }
            }
            
        }
        if eng:NAME = "turboFanEngine" {
            PRINT "Engine: " + eng:NAME + " Thrust: " + ROUND(eng:THRUST)  + " kN".
            if eng:THRUST < 10 {
                PRINT "Engine thrust TOo low, shutting down: " + eng:NAME.
                eng:SHUTDOWN().
                INTAKES OFF.
                SET JETSDONE TO TRUE.
            }
        }
    }
}

//-----------------------------Pretty Stuff------------------------------------//
// Function TO PRINT a formatted message with a timestamp
FUNCTION PrintTimeStamped {
    PARAMETER msg.
    PRINT " " + formatMET + " " + msg.
}
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
    }
//jump
FUNCTION lstage2 {
        TOGGLE ag5.
        SET TERMINAL:HEIGHT TO 44.
        SET TERMINAL:WIDTH TO 74.
        WAIT.1.
        
        // fill baloon
        PRINTDivider("Dem Duke boys...").
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
                    WAIT 1.5.
        //TOGGLE ag50. Fireworks are broken. They doin't go anywhere and break themselvs
        WAIT.2.
        SET TERMINAL:HEIGHT TO 14..
        SET TERMINAL:WIDTH TO 44.
        SET FLAPSLVL TO 1.
        RETURN FLAPSLVL.
    }

//Re-enable Rudder control.
FUNCTION enableCS {
    FOR  PART in SHIP:PARTS {
       if PART:NAME = "hl10rudder" {
            // PRINT "Turning on part: " + PART:NAME.
          PART:GETMODULE("ModuleControlSurface"):SETFIELD("authority limiter", 32).
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
          PART:GETMODULE("ModuleControlSurface"):SETFIELD("authority limiter", 20).
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

PrintTimeStamped ("Setting up node at " + which_apsis_text).
PrintTimeStamped ("ETA=" + ROUND(node_time - TIME:SECONDS,1) + ", dV=" + ROUND(dv,1)).
PrintTimeStamped ("Old eccentricity was " + ROUND(SHIP:ORBIT:ECCENTRICITY,4) + ", new eccentricity is " + ROUND(MyNode:ORBIT:ECCENTRICITY,4)).
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

// Function TO execute a burn at a maneuver node
FUNCTION executeManeuver {
    PrintTimeStamped("Burn time is: " + burnTime).
    wait.2.
    if mynode:typename = "node"{
        LOCK STEERING TO mynode:burnvector.
    }else{
        print "ERROR: LOCK STEERING TO mynode:burnvector.".
    }
    
    PrintTimeStamped("Steering Locked TO maneuver node.").

    // WAIT until the maneuver node's time minus half the burn time
    WAIT until eta:nextnode < (burnTime / 2).

    LOCK throttle TO 1.
    PrintTimeStamped("Executing burn...").

    // WAIT for the burn duration
    WAIT burnTime.
    // I have no idea why I need a -1 but I do. A real magic number.

    LOCK throttle TO 0.
    unlock throttle.
    unlock steering.
    PrintTimeStamped("Burn complete.").
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



//for long term travel radiation management. Needs TO point away from the sun for protectino.

FUNCTION antisun {
    // Calculate vector towards the Sun
    local sunVector is body("Sun"):position - ship:position.
    local awayFromSun is -sunVector:normalized.

    // Lock steering away from Sun
    return awayFromSun.

}
//https://www.reddit.com/r/Kos/comments/4kk0gd/coming_out_of_time_warp/
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
PRINT"===============================".
PRINT"".
PRINT"" + SHIP:NAME + " SSTE Config".
PRINT"".
PRINT"===============================".
PRINT"".

PRINTDivider("FILLING BALOONS").

PrintTimeStamped("Applying brakes and raising legs!").
brakes on.
TOGGLE AG1.

PRINTWelcome().

PRINTDivider("WARNING: DON'T TOUCH SAS, RCS, OR STAGING.").
PRINT " ".
PRINT "Press any key to confirm you will die if you do that...".
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

//------------- no turning back--------
SAS OFF.
PrintTimeStamped("SAS off").
WAIT .2.
PRINTDivider("LOCKING CONTROLS").
WAIT .2.
LOCK STEERING TO HEADING(90, .4, 0).
PrintTimeStamped("Yaw, Pitch and Roll Locked").
WAIT .2.
PrintTimeStamped("Steering wheels Locked").
WAIT .2.
PrintTimeStamped("Control surfaces Locked").

// Find and turn off all hl10rudder parts which is all of the crafts aero control surface. They will slow us down while driving.
ControlSurfacesOff().
WAIT .2.
RCS ON.
WAIT .2.
PrintTimeStamped("RCS ON").
WAIT .4.
PRINTDivider("STARTING FAN SYSTEM!").
WAIT .2.
PrintTimeStamped("Opening Intakes...").
WAIT .2.
PrintTimeStamped("Starting Compressors...").
WAIT .2.
PrintTimeStamped("Starting Nuclear Reactor...").
WAIT .2.
TOGGLE ag2.
PrintTimeStamped("Powering up Fans...").
// reversefans
TOGGLE ag4.
WAIT 1.5.

// Throttle up and release the brakes
PRINTDivider("BACK UP SEQUENCE").

PrintTimeStamped("Throttling up Reverse Fans!").
LOCK throttle TO 1.
WAIT .8.
PrintTimeStamped("Fan thrust is nominal").
wait .5.
BRAKES off.
PrintTimeStamped("Brakes RELEASED!").
wait .2.
PrintTimeStamped("Brakes RELEASED!").
wait .2.
PrintTimeStamped("Brakes RELEASED!").
WAIT 2.

lvlBaloon().

// Driving part
SET TERMINAL:HEIGHT TO 20.
SET TERMINAL:WIDTH TO 49.
SET lstage TO 0.
until lstage >= 5 {
    SET myGroundSpeed TO VDOT(FACING:VECTOR, VELOCITY:SURFACE).
    SET radarAltitude TO alt:radar.//old way, use vessel.
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

       CLEARSCREEN.
    // PRINTDivider("Launch stage: " + lstage).
    if lstage <= 0{
        PRINTDivider("Launch stage: "+LSTAGE+", Back it up!").
    } ELSE IF lstage <= 1{
        PRINTDivider("Launch stage: "+LSTAGE+", Fire Jets").
    } ELSE IF lstage <= 2{
        PRINTDivider("Launch stage: "+LSTAGE+", Finally Forward!").
    } ELSE IF lstage <= 3{
        PRINTDivider("Launch stage: "+LSTAGE+", Dem Duke Boys!").
    } ELSE IF lstage <= 4{
        PRINTDivider("Launch stage: "+LSTAGE+", DO OR DIE!").
    }

    if myGroundSpeed >= 0 {
        PRINT "Moving Forward >>>>>>>".
        PRINT "Target heading: " + ROUND(current_heading_target, 2).
        }ELSE IF myGroundSpeed < -.05{
            PRINT "Moving Backwards <<<<<<.".
            PRINT "Target heading: " + ROUND(reverseHeading, 2).
    }
    // SET pitch TO SHIP:FACING:PITCH.
    // PRINT "Pitch angle: " + ROUND(pitch, 2).
    PRINT "Centerline deviation: " + ROUND(centerline_linear_deviation, 4).
    PRINT "Ground Speed: "+ ROUND(myGroundSpeed, 2) + " m/s".
    PRINT "Wheel turn limit:" + ROUND(AngSet, 2).
    PRINT "Radar altitude: " + ROUND(radarAltitude, 2).

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
PRINTDivider("Launch stage: 4, DO OR DIE!").
WAIT.1.
UNLOCK ALL.  // Releases ALL LOCKs on steering and other vars not using anymore.
LOCK THROTTLE TO 1.
SETPID("pitch", 5, 0.1, 4.5).
SETPID("roll", 3, 0.2, 2.5).
SETPID("yaw", 3, 0.3, 3).
// this number is very important. At low speeds it gets tippy and this will bad things.
SET STEERINGMANAGER:MAXSTOPPINGTIME TO 3.

PrintTimeStamped("yaw, 3, 0.3, 3").
// LOCK STEERING TO HEADING(90.2, 0, 0).

WAIT UNTIL alt:radar > 5.
SETPID("pitch", 1, 0.1, 3).
LOCK STEERING TO HEADING(90.2, 3.25, 0).
PrintTimeStamped("RADAR > 5").
PRINTDivider("Launch stage: 5, LIFTOFF, WE HAVE LIFTOFF!!!").
UNLOCK WHEELSTEERING.
PrintTimeStamped("UNLOCK WHEELSTEERING.").

// PrintTimeStamped("LOCK TO HEADING(90.2, -1, 0).").//90.2 because the runway is .4 and we want go 0 so I split the difference TO avoid TO much yaw at takeoff

// Hardest part: Lost alt, bounce off ground, lost alt, take a dip inthe water.
// Then finally gain alt or die in the water.
// This should happen after tail gear strike. 
WAIT UNTIL alt:radar > 15.
PrintTimeStamped("radar > 15. WAIT 3").
//Pull up very hard
SETPID("pitch", 5, 0.1, 5).
LOCK STEERING TO HEADING(90.2, 10, 0).
// PrintTimeStamped("LOCK TO HEADING(90.2, 14, 0).").
WAIT .8.
PrintTimeStamped("Gear Up").
GEAR OFF.
WAIT 3.5.
// so we can sTOp pulling up...
lvlBaloon().

// Finaly gaining alt
WAIT UNTIL SHIP:verticalspeed > 5.
PrintTimeStamped("verticalspeed > 5").
PrintTimeStamped("LOCK TO HEADING(90, 7.25, 0).").
LOCK STEERING TO HEADING(90, 7.25, 0).
WAIT UNTIL SHIP:AIRSPEED > 150.
// make damn sure the flaps are up.
// had problems, probably don't need this anymore.
PrintTimeStamped("Flaps up.").
TOGGLE AG6.
WAIT.1.
TOGGLE AG6.
WAIT.1.
TOGGLE AG6.
WAIT UNTIL SHIP:AIRSPEED > 160.
// toggle ag3.//
TOGGLE AG6.
PrintTimeStamped("Flaps up.").
list engines in engList.
    //Saves gas?
    FOR  eng in engList {
        if eng:NAME = "turboFanEngine" {
        set eng:thrustlimit to 10.
        }
    }
WAIT UNTIL SHIP:AIRSPEED > 170.
PrintTimeStamped("Flaps up.").
TOGGLE AG6.
WAIT UNTIL SHIP:AIRSPEED > 180.

TOGGLE AG6.
PrintTimeStamped("Flaps up.").
WAIT UNTIL SHIP:AIRSPEED > 190.
TOGGLE AG6.
PrintTimeStamped("Flaps up damnt!").

//225 m/s is enough TO start climbing.
//this will keep you climbing without going TO fast TO save on fuel.
WAIT UNTIL SHIP:AIRSPEED > 200.
PrintTimeStamped("AIRSPEED > 200 HEADING(90, 10, 0)").
LOCK STEERING TO HEADING(90, 10, 0).
WAIT UNTIL alt:radar > 500.
SET STEERINGMANAGER:MAXSTOPPINGTIME TO 1.5.
PrintTimeStamped("radar > 500").
SET CLIMB TO 0.
UNTIL CLIMB > 0{
    SET TOTALTHRUST TO 0.
    WAIT .3.
    CLEARSCREEN.
    PRINTDivider("Launch stage: 6, Slow Climb.").
    PRINT "Airspeed: " + ROUND(AIRSPEED, 2) + " m/s".
    PRINT "Vertical Speed: " + ROUND(SHIP:verticalspeed, 2) + " m/s".
    list engines in engList.
    //Saves gas?
    FOR  eng in engList {
        if eng:NAME = "turboFanEngine" {
        set eng:thrustlimit to ship:altitude / 45.
        }
    }
    PRINT "TOtal Thrust: " + ROUND(TOTALTHRUST) + " kN".
    IF SHIP:AIRSPEED > 275{
        LOCK STEERING TO HEADING(90, 18, 0).
    }
    IF SHIP:AIRSPEED > 250{
       PrintTimeStamped("LOCK TO HEADING(90, 13, 0).").
       // LOCK STEERING TO HEADING(90, 25, 0).
       LOCK STEERING TO HEADING(90, 16, 0).
    }
    IF SHIP:AIRSPEED > 240 AND SHIP:AIRSPEED < 250{
       PrintTimeStamped("LOCK TO HEADING(90, 11, 0).").
       LOCK STEERING TO HEADING(90, 15, 0).
       // LOCK STEERING TO HEADING(90, 11, 0).
    }
     IF SHIP:AIRSPEED < 240 AND SHIP:AIRSPEED > 230 {
       PrintTimeStamped("LOCK TO HEADING(90, 9, 0).").
       LOCK STEERING TO HEADING(90, 14, 0).
       // LOCK STEERING TO HEADING(90, 9, 0).
    }
     IF SHIP:AIRSPEED < 230 AND SHIP:AIRSPEED > 220{
       PrintTimeStamped("LOCK TO HEADING(90, 7, 0).").
       LOCK STEERING TO HEADING(90, 11, 0).
       // LOCK STEERING TO HEADING(90, 7, 0).
    }
    IF SHIP:AIRSPEED < 220{
       PrintTimeStamped("LOCK TO HEADING(90, 6, 0).").
       LOCK STEERING TO HEADING(90, 9, 0).
       // LOCK STEERING TO HEADING(90, 6, 0).
    }

 IF ship:altitude > 5150 {
    //save rcs
    rcs off.
    PrintTimeStamped("RCS OFF").
    SET CLIMB TO 1.
 }
}

// We have some alt, now we need speed. A lot of speed. This will take a long time...
CLEARSCREEN.
// toggle ag3.
PRINTDivider("Launch stage: 7, Gain Speed!").
SETPID("pitch", 2, 0.1, 3).
LOCK STEERING TO HEADING(90, 7.5, 0).
PrintTimeStamped("HEADING(90, 7.5, 0).").

// Turn fans off and close intakes when they become useless.
WAIT UNTIL altitude > 6000.
SET PROPSDONE TO FALSE.
UNTIL PROPSDONE = TRUE {
    monitorEngines().
    WAIT .5.
    IF PROPSDONE = TRUE {
        TOGGLE AG2.
    }
    moniTOrEngines().
    WAIT .5.
}
//90747 liquid fuel needed for rockets.

//https://github.com/lordcirth/kOS-Public/blob/master/maxq.ks
until SHIP:LIQUIDFUEL <= 91427 {
    WAIT .5.
    CLEARSCREEN.
    PRINTDivider("Launch stage: 7, Gain Speed!").
    PRINT "Airspeed: " + ROUND(AIRSPEED, 2) + " m/s".
    PRINT "Vertical Speed: " + ROUND(SHIP:verticalspeed, 2) + " m/s".
    PRINT("Liquid Fuel Left: " + ROUND(SHIP:LIQUIDFUEL)).
    SET TOTALTHRUST TO 0.
    FOR eng in engList {
        SET TOTALTHRUST TO TOTALTHRUST + ENG:THRUST.
    }
    PRINT "Total Thrust: " + ROUND(TOTALTHRUST) + " kN".
    ship:altitude.
    if SHIP:LIQUIDFUEL <= 92500{
        PRINT "|=== GET READY TO BURN! In: " + ROUND((SHIP:LIQUIDFUEL - 91245)).
        TERMINAL:REVERSE.
    }
}
// TERMINAL:REVERSE.
// WAIT .2.
// TERMINAL:REVERSE.
PrintTimeStamped("TIME TO BURN").
rcs on.
PrintTimeStamped("RCS ON").
SETPID("pitch", 5, 0.1, 5.5).
// PrintTimeStamped("PID: 5, 0.1, 6").
LOCK STEERING TO HEADING(90, 29, 0).
PrintTimeStamped("HEADING(90, 29, 0)").
// Start pitching up then fire rockets.
// We start the burn at 91300 because the jets will still be working a bit longer.
WAIT until SHIP:LIQUIDFUEL <= 91392.
PRINTDivider("Launch stage: 8, Pitch and BURN!").
stage.
WAIT 3.
rcs off.

//Start heading TO space!
SET CLIMB2 TO 0.
SET JETSDONE TO false.
UNTIL CLIMB2 > 0{
    SET TOTALTHRUST TO 0.
    WAIT .3.
    CLEARSCREEN.
    PRINTDivider("Launch stage: 9, BURN TO Space!").
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
    moniTOrEngines().
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

//------------------------------- MAKE ORBIT------------------------------

PRINTDivider("Launch stage: 10, Coast TO Space!").
WAIT until ship:altitude >= 70100.

PRINTDivider("Launch stage: 10, Blimps in SPAAAAAACE!").
ControlSurfacesOff().
PrintTimeStamped("Aero Control Surfaces OFF.").
WAIT.5.

// mediumCS().
PrintTimeStamped("SET STEERINGMANAGER:MAXSTOPPINGTIME TO 3.").
SETPID("pitch", 2, 0.2, 3).
SETPID("roll", 2, 0.2, 3).
SETPID("yaw", 2, 0.2, 3).
SET STEERINGMANAGER:MAXSTOPPINGTIME TO 3.
WAIT .5.
//Calculate when it is time TO burn, SET an alarm and do it.
PrintTimeStamped("Creating circularization burn").
CIRCLE().
WAIT 1.
PrintTimeStamped("Calculating burn time.").
SET burnTime TO calculateBurnTime().
WAIT 1.
// SETKACAlarmForNextNode("KillWarp",15, "Burn TO Orbit!", "This burn was brought to you by Snacky Smores. Ride the Walrus!").
// PrintTimeStamped("KAC alarm SET for 15 seconds out").
WAIT .5.
executeManeuver().
wait 2.
SET timeToOrbit to KUniverse:REALTIME - realMissionTime.
print "It took "+ round(timeToOrbit) / 60 + "real life mins to orbit.".
WAIT UNTIL SHIP:OBT:ETA:APOAPSIS < 10 OR SHIP:OBT:ETA:PERIAPSIS < 20.

SET ROLL_ANGLE TO 180.
SET STEERINGMANAGER:MAXSTOPPINGTIME TO 4.
WAIT.5.
local awayFromSun is antisun().
lock steering to lookdirup(-awayFromSun, ship:up:vector).
WAIT 3.


//------------------- IN ORBIT ------------------------------------//
wemadeit().
WAIT 5.
PRINTDivider("Opening up ship!").
//open hanger
WAIT 2.
PrintTimeStamped("Opening Lower Hander Bay Door!").
TOGGLE AG43.
WAIT 3.
//lights on
PrintTimeStamped("Turing on flood lights...").
WAIT .2.
TOGGLE AG14.
PrintTimeStamped("Turing on accent lights...").
WAIT .2.
TOGGLE AG15.
PrintTimeStamped("Turing on gondo cab lights...").
WAIT .2.
TOGGLE AG17.
PrintTimeStamped("Turing on habitat lights").
WAIT .2.
TOGGLE AG13.
WAIT 4.
PrintTimeStamped("Unlock solar panel motors and power on").
//unLOCK solor moTOrs
TOGGLE AG39.
WAIT 1.
//deply side solar.
PrintTimeStamped("Deploy side solar panels.").
TOGGLE AG30.
WAIT 5.
//unLOCK boom moTOrs
PrintTimeStamped("Unlock and power on boom motors.").

TOGGLE AG19.
TOGGLE AG20.
WAIT 1.
//depoly boom
PrintTimeStamped("Deploying boom...").
TOGGLE AG22.
WAIT 9.
PrintTimeStamped("Deploy boom solar").
TOGGLE AG31.
wait 6.
//LOCK moTOrs
PrintTimeStamped("Locking motors").

TOGGLE AG19.
TOGGLE AG20.
WAIT 2.
//depoly comms
PrintTimeStamped("Deploy comms.").

TOGGLE AG24.
//deploy boom solar

WAIT 2.
// depoly science
// Cooling System
PrintTimeStamped("Deploy cooling system").
WAIT 2.
TOGGLE AG36.

//---------------------Set Course for Eve!-------------------------//
WAIT 1.
addons:astrogaTOr:create(eve).
WAIT 1.
SET burnTime TO calculateBurnTime().
WAIT 1.
SETKACAlarmForNextNode("KillWarp",600, "Burn to Eve!!!", "This burn was brought to you by Snacky Smores. Ride the Walrus!").
WAIT 1.
// executeManeuver().
WAIT 1.
local awayFromSun is antisun().
lock steering to lookdirup(-awayFromSun, ship:up:vector).

//------------------Prepare to capture.---------------------------//
// put everything away and turn on nukes. get a little rcs.
//--------------------Capture and areobrake-----------------------//
// probably come in retrograde and burn then turn around real quick
// to hit the atmo leaving 50 dv left for adjustments on the final part
//---------------------Entry--------------------------------------//
// ag8 for pitch with airbrake. use airbrakes as brakes until pitching too much
// flip over, start fans, fly up to stall, eject heat shield. 
//---------------------Find a place to land-----------------------//

//----------------------Land--------------------------------------//

//-------------------Open up the craft for long journey-----------//

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









// PRINT SHIP:PARTSINGROUP("AG5").


// FUNCTION getFlapStatus {
//     for part in ship:parts {
//         // Check if the part has the FAR module
//         if part:hasmodule("FARControllableSurface") {
//             // Get the FAR module
//             SET farModule TO part:getmodule("FARControllableSurface").
            
//             // Check if the part has a flap deflection field
//             if farModule:hasfield("Current flap SETting") {
//                 SET flapSetting TO farModule:getfield("flap").
//                 PRINT "Part: " + part:name + ", Flap Setting: " + flapSetting.
//             } ELSE {
//                 PRINT "Part: " + part:name + " does not have a flap SETting field.".
//             }
//         }
//     }
// }
