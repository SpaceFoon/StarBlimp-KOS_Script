// Loop continuously to check the rover's movement direction
set state to 0.
set RUNWAY_EAST_THRESHOLD_LAT to -0.0502118560109606.
set RUNWAY_EAST_THRESHOLD_LNG to -74.4899977802028.
when state = 0 then {
    // Calculate the dot product of the facing vector and the surface velocity vector
    SET forward_speed to VDOT(FACING:VECTOR, VELOCITY:SURFACE).
    
    // Print the forward speed
    PRINT "Forward Speed: " + forward_speed.
    
    // Check if the rover is moving forward or backward
    IF forward_speed > 0 {
        // Rover is moving forward
        PRINT "Rover is moving forward.".
    } ELSE {
        // Rover is moving backward
        PRINT "Rover is moving backward.".
    }
    
    // Wait for a short duration before checking again
    WAIT 0.1.  // Adjust the duration as needed
    preserve.
}

wait until ship:geoposition:lat < RUNWAY_EAST_THRESHOLD_LAT and ship:geoposition:lng > RUNWAY_EAST_THRESHOLD_LNG.
set state to 1.