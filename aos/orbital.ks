// Orbital mechanics and maneuver functions

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
LOG_INFO("Old eccentricity: " + ROUND(SHIP:ORBIT:ECCENTRICITY,3)).
LOG_INFO("New eccentricity: " + ROUND(MyNode:ORBIT:ECCENTRICITY,3)).
PRINT "Planned Apoapsis: " + ROUND(MyNode:ORBIT:APOAPSIS/1000, 1) + " km".
PRINT "Planned Periapsis: " + ROUND(MyNode:ORBIT:PERIAPSIS/1000, 1) + " km".
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