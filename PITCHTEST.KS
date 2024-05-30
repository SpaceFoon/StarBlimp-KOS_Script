// Function to adjust pitch over time
function adjustPitch {
    parameter targetPitch.  // The target pitch angle in degrees
    parameter duration.  // The time duration over which to adjust the pitch

    // Get the initial pitch
    set initialPitch to ship:facing:vector:pitch.
    set pitchDifference to targetPitch - initialPitch.
    set steps to duration / timeStep.

    // Loop to adjust the pitch gradually
    for i in range(steps) {

        set currentPitch to initialPitch + (pitchDifference * (i / steps)).
        lock steering to heading(currentPitch, ship:facing:vector:heading).
        wait 0.5.
    }

    // Ensure final target pitch is set accurately
    lock steering to heading(targetPitch, ship:facing:vector:heading).
}

// Example usage
set targetPitch to 45.  // Target pitch angle in degrees
set duration to 10.  // Time duration in seconds

// Call the function to adjust pitch
adjustPitch(targetPitch, duration).