SET CLIMB TO 0.
SET TOTALTHRUST TO 0.
UNTIL CLIMB > 0{
    wait .3.
    CLEARSCREEN.
    PRINT("Launch stage: 6, Slow Climb.").
    PRINT round(airspeed, 2).
    PRINT ROUND(SHIP:verticalspeed, 2).
    list engines in engList.
    for eng in engList {
        SET TOTALTHRUST TO TOTALTHRUST + ENG:THRUST.
    }
    PRINT "Thrust: " + ROUND(TOTALTHRUST).
 IF SHIP:airspeed > 250{
    PRINT("LOCK TO HEADING(90, 25, 0).").
    LOCK STEERING TO HEADING(90, 25, 0).
 }
 IF SHIP:airspeed > 240 AND SHIP:airspeed < 250{
    PRINT("LOCK TO HEADING(90, 22, 0).").
    LOCK STEERING TO HEADING(90, 22, 0).
 }
  IF SHIP:airspeed < 240 AND SHIP:airspeed > 230 {
    PRINT("LOCK TO HEADING(90, 20, 0).").
    LOCK STEERING TO HEADING(90, 20, 0).
 }
  IF SHIP:airspeed < 230 AND SHIP:airspeed > 220{
    PRINT("LOCK TO HEADING(90, 18, 0).").
    LOCK STEERING TO HEADING(90, 18, 0).
 }
 IF SHIP:airspeed < 220{
    PRINT("LOCK TO HEADING(90, 15, 0).").
    LOCK STEERING TO HEADING(90, 15, 0).
 }
 IF ship:altitude > 5500 {
    //save rcs
    rcs off.
    PRINT("RCS OFF").
    set CLIMB TO 1.
 }
}