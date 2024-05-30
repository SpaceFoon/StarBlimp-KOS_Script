// // Target altitude for circular orbit
// set targetAltitude to 75000. // 75 km

// // Current apoapsis
// set MYapoapsis to ship:orbit:apoapsis.

// // Check if apoapsis is at the desired target altitude
// // if abs(apoapsis - targetAltitude) > 1000 {
// //     print "Apoapsis is not close to the target altitude. Adjust your initial burn.". 
// //     return. // Exit the script if apoapsis is not close to target altitude
// // }

// // Orbital parameters
// set mu to body:mu.
// set r_apo to MYapoapsis + body:radius.

// // Calculate velocity for circular orbit at the target altitude
// set v_circular to sqrt(mu / r_apo).

// // Current velocity at apoapsis
// set v_apo to sqrt(mu * (2 / r_apo - 1 / ship:orbit:semiMajorAxis)).

// // Delta-v needed for circularization
// set delta_v to v_circular - v_apo.

// // Add maneuver node at apoapsis
// set MYnode to node(TIME + ship:OBT:ETA:APOAPSIS, delta_v, 0, 0).
// add MYnode.

// // Print information
// print "Maneuver node created at apoapsis for circularization.".
// print "Delta-v: " + delta_v.

// // Optionally, point the ship towards the maneuver node
// // lock steering to MYnode:prograde.

// // Optionally, wait until near the node to execute
// // This block waits until 30 seconds before the node execution time to start the burn
// wait until MYnode:eta < 2.

// // Execute the node
// lock throttle to 1.
// wait until MYnode:burnVector:mag < 0.1. // Burn until the delta-v is nearly exhausted
// lock throttle to 0.
// print "Burn complete, circular orbit achieved.".
// ..


//https://forum.kerbalspaceprogram.com/topic/181894-orbital-mechanics-of-circularization/
// circularizes at the next apsis

function CIRCLE {
set burn_at_periapsis to TRUE.
if APOAPSIS > 0 { // if apoapsis is negative, time to apoapsis is infinite
	if ETA:APOAPSIS < ETA:PERIAPSIS { SET burn_at_periapsis to FALSE. }
}
if burn_at_periapsis { set which_apsis_text to "Periapsis". } else { set which_apsis_text to "Apoapsis". }

if burn_at_periapsis {
	set node_time to TIME:SECONDS + ETA:PERIAPSIS.
	set otherapsis to APOAPSIS.
	set burnapsis to PERIAPSIS.
} ELSE {
	set node_time to TIME:SECONDS + ETA:APOAPSIS.
	set otherapsis to PERIAPSIS.
	set burnapsis to APOAPSIS.
}

set v_old to sqrt(BODY:MU * (2/(burnapsis+BODY:RADIUS) -
                             1/SHIP:OBT:SEMIMAJORAXIS)).
set v_new to sqrt(BODY:MU * (2/(burnapsis+BODY:RADIUS) -
			     1/(BODY:RADIUS+burnapsis))).
set dv to v_new - v_old.

set MyNode to NODE(node_time, 0,0,dv).
ADD MyNode.

PRINT ("Setting up node at " + which_apsis_text).
PRINT ("ETA=" + ROUND(node_time - TIME:SECONDS,1) + ", dV=" + ROUND(dv,1)).
PRINT ("Old eccentricity was " + ROUND(SHIP:ORBIT:ECCENTRICITY,4) + ", new eccentricity is " + ROUND(MyNode:ORBIT:ECCENTRICITY,4)).
// PRINT MyNode:ORBIT:APOAPSIS.  // apoapsis after maneuver
// PRINT MyNode:ORBIT:PERIAPSIS. // periapsis after maneuver
}
CIRCLE().

