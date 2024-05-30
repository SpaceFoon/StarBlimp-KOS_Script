// Target altitude for circular orbit
set targetAltitude to 75000. // 75 km

// Current apoapsis
set apoapsis to ship:orbit:apoapsis.

// Check if apoapsis is at the desired target altitude
if abs(apoapsis - targetAltitude) > 1000 {
    print "Apoapsis is not close to the target altitude. Adjust your initial burn.". 
    return. // Exit the script if apoapsis is not close to target altitude
}

// Orbital parameters
set mu to body:mu.
set r_apo to apoapsis + body:radius.

// Calculate velocity for circular orbit at the target altitude
set v_circular to sqrt(mu / r_apo).

// Current velocity at apoapsis
set v_apo to sqrt(mu * (2 / r_apo - 1 / ship:orbit:semiMajorAxis)).

// Delta-v needed for circularization
set delta_v to v_circular - v_apo.

// Add maneuver node at apoapsis
set node to node(ship:orbit:timeToApoapsis, delta_v, 0, 0).
add node.

// Print information
print "Maneuver node created at apoapsis for circularization.".
print "Delta-v: " + delta_v.

// Optionally, point the ship towards the maneuver node
lock steering to node:prograde.

// Optionally, wait until near the node to execute
// This block waits until 30 seconds before the node execution time to start the burn
wait until node:eta < 30.

// Execute the node
lock throttle to 1.
wait until node:burnVector:mag < 0.1. // Burn until the delta-v is nearly exhausted
lock throttle to 0.
print "Burn complete, circular orbit achieved.".
..