//https://www.reddit.com/r/KerbalSpaceProgram/comments/5wl42h/kos_runway_centerline_geoposition_analysis/
//steers with wheels while on runway
set CENTERLINE_EQ_A to 142.13236295.    // latitude coefficient for the linear equation
set CENTERLINE_EQ_B to 1.               // longitude coefficient
set CENTERLINE_EQ_C to 81.62849024.     // constant term
set RUNWAY_EAST_THRESHOLD_LAT to -0.0502118560109606.
set RUNWAY_EAST_THRESHOLD_LNG to -74.4899977802028.
set CENTERLINE_PID_OUTPUT_LIMIT to 2.
 
print "Takeoff roll.".
set state to 0. // takeoff roll.
set centerline_pid to pidloop(1, 0.2, 3, -CENTERLINE_PID_OUTPUT_LIMIT, CENTERLINE_PID_OUTPUT_LIMIT).
set centerline_pid:setpoint to 0.
lock current_heading_target to centerline_pid:output + 90.
 
// set current_pitch_target to SHIP:FACING:PITCH.
lock centerline_angular_deviation to (CENTERLINE_EQ_A * ship:geoposition:lat + CENTERLINE_EQ_B * ship:geoposition:lng + CENTERLINE_EQ_C) / sqrt(CENTERLINE_EQ_A^2 + CENTERLINE_EQ_B^2).
lock centerline_linear_deviation to -2 * constant:pi * KERBIN:RADIUS * centerline_angular_deviation / 360.

// lock steering to heading(current_heading_target, 0).
// lock wheelThrottle to 1.
set desiredRoll  to 0.0.

set wheels to LIST().
for PART in SHIP:PARTS {
    if PART:HASMODULE("ModuleWheelSteering") {
        wheels:ADD(PART).
    }
}

when state = 0 then {
    set myGroundSpeed to VDOT(FACING:VECTOR, VELOCITY:SURFACE).
    if myGroundSpeed < 0 {
        if current_heading_target > 90{
            set equals to current_heading_target - 90.

            set reverseHeading to (90 -equals).
        }
        if current_heading_target < 90{
            set equals to ABS(current_heading_target - 90).

            set reverseHeading to (90 + equals).
        }
        // if current_heading_target - 180 < 0 {
        //     set reverseHeading to current_heading_target - 180 + 360.
        // }else{
        //     set reverseHeading to current_heading_target - 180.
        // }
        // lock WHEELSTEERING to reverseHeading.
        lock WHEELSTEERING to reverseHeading.
        // FOR WHEEL IN WHEELS 
        // {
        //     WHEEL:GETMODULE("ModuleWheelSteering"):SETFIELD("Steering: Direction", "Inverted").
        //     set direc to WHEEL:GETMODULE("ModuleWheelSteering"):GETFIELD("Steering: Direction").
        // }.
        // set current_heading_target to current_heading_target - 180.
        
    }else {
        // FOR WHEEL IN WHEELS 
        // {
        //     WHEEL:GETMODULE("ModuleWheelSteering"):SETFIELD("Steering: Direction", false).
        //     set direc to WHEEL:GETMODULE("ModuleWheelSteering"):GETFIELD("Steering: Direction").
        // }.
        lock WHEELSTEERING to current_heading_target.
            
    }

    if ROUND(SHIP:groundspeed) < 14 {
    set AngSet to 30.
    }else {
        set speed to ROUND(SHIP:groundspeed).
        set AngSet to ABS(40 / ABS(speed)).

        FOR WHEEL IN WHEELS 
        {
            WHEEL:GETMODULE("ModuleWheelSteering"):SETFIELD("steering angle limiter", AngSet).
            
        }.
    }.

   
    clearscreen.
    print "Centerline deviation: " + centerline_linear_deviation.
    if myGroundSpeed > 0 {
        print "Moving Forward >>>>>>>".
        print "Target heading: " + current_heading_target.
        }else {
            print "Moving Backwards <<<<<<.".
            print "reverse heading: " + reverseHeading.
    }
    print "Ground Speed: "+ myGroundSpeed.
    print "Wheel turn limit:" + AngSet.
    // print "Steering directin" + direc.
    centerline_pid:update(time:seconds, centerline_linear_deviation).
    preserve.
}
 
wait until ship:geoposition:lat < RUNWAY_EAST_THRESHOLD_LAT and ship:geoposition:lng > RUNWAY_EAST_THRESHOLD_LNG.
set state to 1.