// WE DID IT REDDIT!
//https://www.reddit.com/r/Kos/comments/31r8hj/time_and_easiest_way_to_produce_a_timespan_from/

// left-pad with zeroes. Assumes you want a length of 2 if not specified
FUNCTION padZ { PARAMETER t, l is 2.
    RETURN (""+t):PADLEFT(l):REPLACE(" ","0").
}

// returns elapsed time in the format "[T+YY-DDD HH:MM:SS]"
FUNCTION formatMET
{
  LOCAL ts IS TIME + MISSIONTIME - TIME:SECONDS.
  RETURN "[T+" 
    + padZ(ts:YEAR - 1) + "-" // subtracts 1 to get years elapsed, not game year
    + padZ(ts:DAY - 1,3) + " " // subtracts 1 to get days elapsed, not day of year. What is the 3 for?
    + padZ(ts:HOUR) + ":"
    + padZ(ts:MINUTE) + ":"
    + padZ(ROUND(ts:SECOND))+ "]".
}
print formatMET.

FUNCTION formatUNI
{
  LOCAL ts IS TIME.
  RETURN "[Y" 
    + round(ts:YEAR) + ", D"
    + padZ(ts:DAY) + ", "
    + padZ(ts:HOUR) + ":"
    + padZ(ts:MINUTE) + ":"
    + padZ(ROUND(ts:SECOND))+ "]".
}
print formatUNI.