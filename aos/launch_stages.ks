// Launch stage functions

//brake till sTOpped then go foward
SET FLAPSLVL TO 0.
FUNCTION lstage1 {
        RUN_MARQUEE(BUILD_ORDER(), 0.25, 4).
        //reverse fan
    AG_TOGGLE(AGX_LEGEND["reverse_fan"]).
        WAIT.1.
        //jets
    AG_TOGGLE(AGX_LEGEND["jets"]).
        WAIT.1.
        //just to make sure
        INTAKES ON.
        SET BRAKES TO true.
        WAIT until SHIP:groundspeed < 1.
        SET BRAKES TO false.
        WAIT.1.
        STOP_MARQUEE().
    }
//jump
FUNCTION lstage2 {
        // SET TERMINAL:HEIGHT TO 44.
        // SET TERMINAL:WIDTH TO 74.
        WAIT.1.
        RUN_MARQUEE(BUILD_ORDER(), 0.50, 6).
        
  LOG_STAGE("Special Event: Dem Duke boys...", "Airship Jump Sequence").
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
        // SET TERMINAL:HEIGHT TO 20.
        // SET TERMINAL:WIDTH TO 40.
        SET FLAPSLVL TO 1.
        STOP_MARQUEE().
        RETURN FLAPSLVL.

    }

//taxi to takeoff
FUNCTION lstage3 {
    //Taxi to takeoff sequence
    LOCAL runway_hud_interval IS 0.5.
    LOCAL runway_next_hud IS 0.
    LOCAL runway_last_stage_logged IS -999.
    until lstage >= 5 {
        UPDATE_CRASH_MONITOR().
        SET myGroundSpeed TO VDOT(FACING:VECTOR, VELOCITY:SURFACE).
        SET radarAltitude TO SHIP:ALTITUDE - SHIP:GEOPOSITION:TERRAINHEIGHT.
        // IF myGroundSpeed > 10 {
        //   lvlBalloon().
        //     STOP_MARQUEE().
        // }
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
     
        //This means we hit the jump
        if radarAltitude > 4 and lstage = 2 {
            AG_TOGGLE("flaps_down").
            lstage2().
            rcs off.
            SET lstage TO 3.
            // Need buoyancy over balance during liftoff.
            fillBalloons().
        }.
        
        //flaps lvl 2 (takeoff)
        if myGroundSpeed > 115{
            
            IF FLAPSLVL < 2{
                AG_TOGGLE("flaps_down").
                SET FLAPSLVL TO 2.
            }
            
            SET lstage TO 4.
            //reenable control surfaces
            if myGroundSpeed > 108 {
                IF myGroundSpeed > 115{
                    SET lstage TO 5.
                }
                CONTROL_AUTHORITY(25).
                 
            }
        }
        
      //--------------------   Steering Limiter for driving --------------------
        // Instead of trying to make pid adjustments at different speeds, I just limit
        // the maximum steering power as you go faster. Easy way to make steering work
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
        // Use LOG_STAGE for consistent animated decorations
        LOCAL stage_desc IS "Ground Operations - Backing Up".
        IF lstage <= 0 {
          SET stage_desc TO "Ground Operations - Backing Up".
        } ELSE IF lstage <= 1 {
          SET stage_desc TO "Engine Startup - Jets Firing".
        } ELSE IF lstage <= 2 {
          SET stage_desc TO "Taxi Operations - Moving Forward".
        } ELSE IF lstage <= 3 {
          SET stage_desc TO "Jump Sequence - Airborne".
        } ELSE IF lstage <= 4 {
          SET stage_desc TO "Critical Phase - No Turning Back".
        }.
        LOG_STAGE(stage_label, stage_desc).
        SET runway_last_stage_logged TO lstage.
      }.

      IF TIME:SECONDS >= runway_next_hud {
        SET runway_next_hud TO TIME:SECONDS + runway_hud_interval.

        DISPLAY_STAGE_LABEL(stage_label).
        IF myGroundSpeed > 60{
          IF myGroundSpeed >= 0 {
          // PRINT "Moving Forward >>>>>>>".
          PRINT "Target heading: " + ROUND(current_heading_target, 1) + "°".
        } ELSE IF myGroundSpeed < -.05 {
          // PRINT "Moving Backwards <<<<<<.".
          PRINT "Target heading: " + ROUND(reverseHeading, 1) + "°".
        }.
        }else{
           IF myGroundSpeed >= 0 {
          PRINT "Moving Forward >>>>>>>".
          PRINT "Target heading: " + ROUND(current_heading_target, 1) + "°".
        } ELSE IF myGroundSpeed < -.05 {
          PRINT "Moving Backwards <<<<<<.".
          PRINT "Target heading: " + ROUND(reverseHeading, 1) + "°".
        }.
        }
       
        PRINT "Centerline deviation: " + ROUND(centerline_linear_deviation, 2) + " m".
        PRINT "Ground Speed: " + ROUND(myGroundSpeed, 2) + " m/s".
        PRINT "Wheel turn limit: " + ROUND(AngSet, 1) + "°".
        PRINT "Radar altitude: " + ROUND(radarAltitude, 1) + " m".
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
}

//DO OR DIE braking
FUNCTION lstage4 {
    CLEARSCREEN.
    rcs on.
    LOG_STAGE("Launch stage: 3,", "DO OR DIE! - Too late to brake!").
    RUN_MARQUEE(BUILD_ORDER(), 0.2, 4, FALSE).
    WAIT.1.
    UNLOCK ALL.  // Releases ALL LOCKs on steering and other vars not using anymore.
    LOCK THROTTLE TO 1.
    SETPID("pitch", 5, 0.1, 4.5).
    SETPID("roll", 3, 0.2, 2.5).
    SETPID("yaw", 3, 0.3, 3).
    // This number is very important. At low speeds it gets tippy and this will do bad things.

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
}

//takeoff
FUNCTION lstage5 {
    CLEARSCREEN.
    LOG_STAGE("Launch stage: 4,", "LIFTOFF, WE HAVE LIFTOFF!!!").
    UNLOCK WHEELSTEERING.
    LOG_INFO("Unlock wheel steering").

    // PrintTimeStamped("LOCK TO HEADING(90.2, -1, 0).").

    // Hardest part: Take off, Lose alt, tailstrike bounce, lose alt again, tail strike the water.
    // Then finally gain alt or die in the water.

    // Wait till we clear the first hill after runway.
    WAIT UNTIL alt:radar > 3.
    LOG_INFO("radar > 3: PULL UP HARD").
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
    lvlBalloon().

    // Finaly gaining alt
    WAIT UNTIL SHIP:verticalspeed > 5.
    LOG_INFO("vertical speed > 5").
    LOG_INFO("Lock to heading (90, 7.25, 0)").
    LOCK STEERING TO HEADING(90, 7.25, 0).
    WAIT UNTIL SHIP:AIRSPEED > 150.
    // make damn sure the flaps are up.
    // had problems, probably don't need this anymore.
    LOG_INFO("Flaps up").
    AG_TOGGLE(AGX_LEGEND["flaps_up"]).
    WAIT.1.
    AG_TOGGLE(AGX_LEGEND["flaps_up"]).
    WAIT.1.
    AG_TOGGLE(AGX_LEGEND["flaps_up"]).
    WAIT UNTIL SHIP:AIRSPEED > 160.
    // toggle ag3.//
    AG_TOGGLE(AGX_LEGEND["flaps_up"]).
    LOG_INFO("Flaps up").
    list engines in engList.
        //Saves gas?
        FOR  eng in engList {
            if eng:NAME = "turboFanEngine" {
            set eng:thrustlimit to 0.
            }
        }
    LOG_INFO("Jets off, save fuel").
    WAIT UNTIL SHIP:AIRSPEED > 170.
    LOG_INFO("Flaps up").
    AG_TOGGLE(AGX_LEGEND["flaps_up"]).
    WAIT UNTIL SHIP:AIRSPEED > 180.

    AG_TOGGLE(AGX_LEGEND["flaps_up"]).
    LOG_INFO("Flaps up").
    WAIT UNTIL SHIP:AIRSPEED > 190.
    AG_TOGGLE(AGX_LEGEND["flaps_up"]).
    LOG_INFO("Flaps up, damn it!").

    //225 m/s is enough to start climbing.
    //this will keep you climbing without going too fast to save on fuel.

    WAIT UNTIL SHIP:AIRSPEED > 200.
    LOG_INFO("200 m/s: pitch 10°").
    LOCK STEERING TO HEADING(90, 10, 0).
    WAIT UNTIL alt:radar > 500.
    SET STEERINGMANAGER:MAXSTOPPINGTIME TO 1.5.
}

//slow climb saving gas
FUNCTION lstage6 {
    SET CLIMB TO 0.
    UNTIL CLIMB > 0{
        SET TOTALTHRUST TO SHIP:THRUST.
        WAIT .3.
        CLEARSCREEN.
      LOG_STAGE("Launch stage: 5, ", "Slow Climb, Saving Gas").
        PRINT "Airspeed: " + ROUND(AIRSPEED, 2) + " m/s".
        PRINT "Vertical Speed: " + ROUND(SHIP:verticalspeed, 2) + " m/s".
        list engines in engList.
        //Saves gas?
        FOR  eng in engList {
            if eng:NAME = "turboFanEngine" {
            set eng:thrustlimit to (ship:altitude - 1000) / 45.
            }
        }
        PRINT "Total Thrust: " + ROUND(TOTALTHRUST, 1) + " kN".
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
}

//gain speed maximum jet thrust
FUNCTION lstage7 {
    CLEARSCREEN.
    list engines in engList.
        FOR  eng in engList {
            if eng:NAME = "turboFanEngine" {
            set eng:thrustlimit to 100.
            }
        }
    LOG_STAGE("Launch stage: 6, ", "Gain Speed! Maximum Jet Thrust").
    SETPID("pitch", 2, 0.1, 3).
    LOCK STEERING TO HEADING(90, 7.5, 0).
    LOG_INFO("Heading (90, 7.5, 0)").

    // Turn fans off and close intakes when they become useless.
    WAIT UNTIL altitude > 6000.
    LOG_INFO("Monitoring propeller engines for shutdown...").
    UNTIL FALSE {  // Loop until we break
        LOCAL shutdown_type IS monitorEngines().
        IF shutdown_type = "props" {
            LOG_WARN("Prop flameout protection enabled: Engines shutting down.").
            BREAK.
        }
        WAIT 0.5.
    }
    //90747 liquid fuel needed for rockets. <<< not no more

    //https://github.com/lordcirth/kOS-Public/blob/master/maxq.ks
    until SHIP:LIQUIDFUEL <= 90747 {
        WAIT .5.
        CLEARSCREEN.
      LOG_STAGE("Launch stage: 7, ", "Burn to space!").
        PRINT "Airspeed: " + ROUND(AIRSPEED, 2) + " m/s".
        PRINT "Vertical Speed: " + ROUND(SHIP:verticalspeed, 2) + " m/s".
        PRINT("Liquid Fuel Left: " + ROUND(SHIP:LIQUIDFUEL) + " units").
        SET TOTALTHRUST TO 0.
        FOR eng in engList {
            SET TOTALTHRUST TO TOTALTHRUST + ENG:THRUST.
        }
        PRINT "Total Thrust: " + ROUND(TOTALTHRUST, 1) + " kN".
        ship:altitude.
        if SHIP:LIQUIDFUEL <= 93500{
            PRINT "|=== GET READY TO BURN!".
            PRINT "Fuel remaining: " + ROUND((SHIP:LIQUIDFUEL - 90747)) + " units".
            TERMINAL:REVERSE.
        }
    }
}

//pitch and burn
FUNCTION lstage8 {
    RUN_MARQUEE(BUILD_ORDER(), 0.2, 6, TRUE).
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
    WAIT 2.
    STOP_MARQUEE().
    WAIT 1.
    RUN_MARQUEE(BUILD_ORDER(), 0.35, 2).
    wait 1.
    LOG_STAGE("Launch stage: 8, Pitch and BURN!", "Rocket Ignition - Main Engine Start").
    stage.
    WAIT 3.
    rcs off.
}

//burn to space
FUNCTION lstage9 {
    //---------------------------- Burn heading to space! -------------------------//
    SET CLIMB2 TO 0.
    SET JETSDONE TO false.
    STOP_MARQUEE().
    UNTIL CLIMB2 > 0{
        SET TOTALTHRUST TO 0.
        WAIT .3.
        CLEARSCREEN.
      LOG_STAGE("Launch stage: 9, BURN TO Space!", "Rocket Powered Ascent - Main Burn").
        PRINT "Alt: " + ROUND(ship:altitude) + " m".
        PRINT "Airspeed: " + ROUND(AIRSPEED, 2) + " m/s".
        PRINT "Vertical Speed: " + ROUND(SHIP:verticalspeed, 2) + " m/s".

        PRINT "Jet Shutdown: " + JETSDONE.
        list engines in engList.
        FOR eng in engList {
            SET TOTALTHRUST TO TOTALTHRUST + ENG:THRUST.
        }
        PRINT "Thrust: " + ROUND(TOTALTHRUST, 1) + " kN".
        IF SHIP:OBT:ETA:APOAPSIS > 40{
            LOCK STEERING TO HEADING(90, 25, 0).
        }
        IF SHIP:OBT:ETA:APOAPSIS > 47{
            LOCK STEERING TO HEADING(90, 20, 0).
        }
        IF JETSDONE = FALSE {
            // Monitor engines to check if jets have failed at high altitude
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
}

//coast to space
FUNCTION lstage10 {
    CLEARSCREEN.

    //------------------------------- MAKE ORBIT------------------------------//

    LOG_STAGE("Launch stage: 10, Coast TO Space!", "Coasting Phase - Ballistic Trajectory").
    WAIT until ship:altitude >= 70100.
}

//blimps in space
FUNCTION lstage11 {
    LOG_STAGE("Launch stage: 11, Blimps in SPACE!", "Space Achievement - We Made It!").
    ControlSurfacesOff().
    LOG_INFO("Aero control surfaces off").
    WAIT.5.

    // mediumCS().
    LOG_INFO("Steering tuned").
    SETPID("pitch", 2, 0.2, 3).
    SETPID("roll", 2, 0.2, 3).
    SETPID("yaw", 2, 0.2, 3).
    SET STEERINGMANAGER:MAXSTOPPINGTIME TO 3.
    WAIT .5.
    //Calculate when it is time TO burn, SET an alarm and do it.
}

//circularize
FUNCTION lstage12 {
    LOG_STAGE("Launch stage: 12, Circularize!", "Orbital Mechanics - Making Orbit").
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
    PRINT "Mission complete in " + ROUND(timeToOrbit / 60, 1) + " real-time minutes".
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
}

//opening up ship
FUNCTION lstage13 {
    LOG_STAGE("Launch stage: 13, Opening up ship!", "Ship Configuration - Deploy Systems").
    // open hangar
    WAIT 2.
    LOG_INFO("Opening lower hangar bay door").
    AG_TOGGLE(AGX_LEGEND["hangar_door"]).

    LOG_INFO("Solar motors on").
    // its really in side_solar_open group

    WAIT 1.
    // deploy side solar
    LOG_INFO("Deploying side solar panels").
    AG_TOGGLE(AGX_LEGEND["side_solar_open"]).
    WAIT 5.
    // unlock boom motors and power on
    LOG_INFO("Boom motors on").
    AG_TOGGLE(AGX_LEGEND["motor_locks_off"]).
    AG_TOGGLE(AGX_LEGEND["motor_apu_on"]).
    WAIT 1.
    // deploy boom
    LOG_INFO("Deploying boom").
    AG_TOGGLE(AGX_LEGEND["boom_open"]).
    WAIT 9.
    LOG_INFO("Deploying boom solar panels").
    AG_TOGGLE(AGX_LEGEND["boom_solar_deploy"]).
    WAIT 6.
    // lock motors
    LOG_INFO("Locking motors").
    AG_TOGGLE(AGX_LEGEND["motor_locks_on"]).
    AG_TOGGLE(AGX_LEGEND["motor_apu_on"]).
    WAIT 2.
    // deploy comms
    LOG_INFO("Comms deploying").
    AG_TOGGLE(AGX_LEGEND["comms_long"]).
    AG_TOGGLE(AGX_LEGEND["comms_hga"]).
    WAIT 2.
    // deploy science cooling system
    LOG_INFO("Deploying cooling system").
    WAIT 2.
    AG_TOGGLE(AGX_LEGEND["cooling_system"]).
}

//set course for eve
FUNCTION lstage14 {
    //---------------------Set Course for Eve!-------------------------//
    LOG_STAGE("Launch stage: 14, Course for Eve!", "Interplanetary Transfer - Navigation").
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

    // Mission successful - disable crash monitoring
    STOP_CRASH_MONITOR().
}