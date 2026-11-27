// Common utilities for launch script
GLOBAL CRASH_MONITOR_ACTIVE IS TRUE.
GLOBAL INITIAL_PART_COUNT IS 0.
GLOBAL PART_LOSS_THRESHOLD IS 3.

// Global variables for non-blocking crash monitoring
GLOBAL CRASH_CHECK_TIME IS 0.
GLOBAL CRASH_CHECK_INTERVAL IS 15.

GLOBAL PROPSDONE IS FALSE.
GLOBAL JETSDONE IS FALSE.
GLOBAL CACHED_GROUNDSPEED IS 0.


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

// Initialize all variables before LOCK to prevent undefined errors
SET centerline_angular_deviation TO 0.
SET centerline_linear_deviation TO 0.
SET current_heading_target TO 90.

// Now create LOCK expressions that will update them
LOCK centerline_angular_deviation TO (CENTERLINE_EQ_A * ship:geoposition:lat + CENTERLINE_EQ_B * ship:geoposition:lng + CENTERLINE_EQ_C) / sqrt(CENTERLINE_EQ_A^2 + CENTERLINE_EQ_B^2).
LOCK centerline_linear_deviation TO -2 * constant:pi * KERBIN:RADIUS * centerline_angular_deviation / 360.
LOCK current_heading_target TO centerline_pid:output + 90.

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

//Function to shut down engines when they have very little thrust.
FUNCTION monitorEngines {
  LIST ENGINES IN engList.
  
  LOCAL props_shutdown IS FALSE.
  LOCAL jets_shutdown IS FALSE.
  
  // Check propeller engines
  FOR eng IN engList {
    IF eng:NAME = "WBILargeElectricPart" AND eng:IGNITION AND eng:THRUST <= 15 {
      SET props_shutdown TO TRUE.
    }
  }

  // Check jet engines  
  FOR eng IN engList {
    IF eng:NAME = "turboFanEngine" AND eng:IGNITION AND eng:THRUST <= 5 {
      SET jets_shutdown TO TRUE.
    }
  }
  
  // Use action groups to control engine systems
  IF props_shutdown {
    AG_SET_STATE(AGX_LEGEND["fan_system"], FALSE).
    SET PROPSDONE TO TRUE.
  }
  
  IF jets_shutdown {
    AG_SET_STATE(AGX_LEGEND["jets"], FALSE).
    SET JETSDONE TO TRUE.
  }
  
  // Return what was shut down (prioritize props since they shut down first)
  IF props_shutdown {
    RETURN "props".
  } ELSE IF jets_shutdown {
    RETURN "jets".
  } ELSE {
    RETURN "none".
  }
}


//With Helium!
GLOBAL FUNCTION fillBalloons {
    FOR  PART in SHIP:PARTS {
     if PART:HASMODULE("HLEnvelopePartModule") {
        // PRINT "part: " + part:name.
        PART:GETMODULE("HLEnvelopePartModule"):DOACTION("buoyancy max", true).
    }
}
}

//Dont fill this one as much for balance.
FUNCTION lvlBalloon {
    FOR  PART in SHIP:PARTS {
        if PART:NAME = "hl10NoseCone" {
            // Loop TO press the "buoyancy --" butTOn 39 times
            SET butTOnPresses TO 0.
            until butTOnPresses >= 39 {
                PART:GETMODULE("HLEnvelopePartModule"):DOACTION("buoyancy --", true).
                
                PRINT "LEVELING BALLOONS: "+ ROUND(butTOnPresses * 2.631) + " / 100".
                SET butTOnPresses TO butTOnPresses + 1.
                WAIT 0.02.
            }
        }
    }
}

FUNCTION CONTROL_AUTHORITY {
  PARAMETER deflection_limit IS 15.
  LOCAL target_names IS LIST("HL10Rudder", "hl10rudder", "wingShuttleElevon1", "wingShuttleElevon2").

  FOR part IN SHIP:PARTS {
    IF target_names:CONTAINS(part:NAME) AND part:HASMODULE("SyncModuleControlSurface") {
      part:GETMODULE("SyncModuleControlSurface"):SETFIELD("authority limiter", deflection_limit).
    }.
  }.
}

FUNCTION MediumControlSurfaces {
  CONTROL_AUTHORITY(10).
}

// Find and turn off aero control surfaces. They slow us down while taxiing.
FUNCTION ControlSurfacesOff {
  CONTROL_AUTHORITY(0).
}