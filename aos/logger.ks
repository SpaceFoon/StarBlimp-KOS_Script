// logger.ks - Flight data logging functionality

// Initial logging setup
if(Logging) {
LOG "TIME" + "," +
 "SHIP:ALTITUDE" + "," +
 "TargetAltitude" + "," +
 "AltitudePitch" + "," +
 "CACHED_GROUNDSPEED" + "," +
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

// Asynchronous logging loop
WHEN TIME:SECONDS > oneSecondsLater THEN {
if(Logging) {
SET CACHED_GROUNDSPEED TO SHIP:GROUNDSPEED.
SET oneSecondsLater to TIME:SECONDS + 1.
LOG TIME:SECONDS + "," +
 SHIP:ALTITUDE + "," +
//  TargetAltitude + "," +
//  AltitudePitch + "," +
 CACHED_GROUNDSPEED + "," +
 VERTICALSPEED + "," +
 AIRSPEED + "," +
 SpeedPitch + "," +
 SHIP:Q + "," +
//  SHIP:SENSORS:PRES + "," +
//  AirResistPitch + "," +
 SHIP:APOAPSIS + "," +
 AVAILABLETHRUST + "," +
 SHIP:MASS + "," +
 SHIP:WETMASS + "," +
 SHIP:DRYMASS + "," +
 PitchingSteer
  to "0:/Launch.csv". 
  }.
RETURN TRUE.
}.