
// Define PID constants
set kp to 0.1.
set kd to 0.05.

// Function to check pitch and apply brakes if deviation is too large
function checkPitch {
    set currentPitch to ship:facing:pitch.
    set pitchDeviation to abs(currentPitch - desiredPitch).

    if pitchDeviation > pitchThreshold {
        print "Pitch deviation too large! Applying brakes.".
        brakes on.
        return true.
    }
    return false.
}

// Function to correct the steering to keep the craft in the middle of the runway
function maintainCenterline {
    set latError to ship:geoposition:longitude - runwayLongitude.
    set latErrorRate to (latError - latErrorPrev) / (time:seconds - timePrev).
    
    set correction to kp * latError + kd * latErrorRate.
    
    lock steering to heading(desiredHeading, desiredPitch) + R(0, 0, correction).
    
    set latErrorPrev to latError.
    set timePrev to time:seconds.
}

// Initialize variables for PD control
set latErrorPrev to 0.
set timePrev to time:seconds.
set runwayLongitude to ship:geoposition:lng. // Set this to the longitude of the center of the runway

// Wait until the speed reaches 30 m/s, while checking pitch and maintaining centerline
until ship:velocity:surface:mag >= 30 {
    if checkPitch() {
        lock throttle to 0.
        break.
    }
    
    maintainCenterline().
    wait 0.1. // Check every 0.1 seconds
}

PRINT "There is " + STAGE:LIQUIDFUEL + " liquid fuel in this stage.".


