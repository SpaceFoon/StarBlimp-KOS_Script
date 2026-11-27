// Eve transfer and aerobraking functions

// ===============================================
// Warp helpers
// ===============================================
DECLARE FUNCTION WARP_TO_NODE {
  PARAMETER nd.
  PARAMETER lead IS 10.

  IF NOT HASNODE { PRINT "No node for warp_to_node.". RETURN. }

  IF nd:ETA <= lead { SET WARP TO 0. RETURN. }

  UNTIL nd:ETA <= lead {
    LOCAL e IS nd:ETA - lead.
    LOCAL w IS 0.
    IF e > 6*HOUR       { SET w TO 6. }
    ELSE IF e > 1*HOUR  { SET w TO 5. }
    ELSE IF e > 20*MINUTE { SET w TO 4. }
    ELSE IF e > 3*MINUTE  { SET w TO 3. }
    ELSE IF e > 40        { SET w TO 2. }
    ELSE IF e > 10        { SET w TO 1. }
    ELSE { SET w TO 0. }
    IF WARP <> w { SET WARP TO w. }
    WAIT 0.2.
  }
  SET WARP TO 0.
}.

DECLARE FUNCTION WARP_TO_ALT {
  PARAMETER targetAlt.
  PARAMETER leadAlt IS 5000.

  IF SHIP:ALTITUDE <= targetAlt + leadAlt { SET WARP TO 0. RETURN. }

  UNTIL SHIP:ALTITUDE <= targetAlt + leadAlt {
    LOCAL a IS SHIP:ALTITUDE - (targetAlt + leadAlt).
    LOCAL w IS 0.
    IF a > 2*SHIP:BODY:RADIUS { SET w TO 6. }
    ELSE IF a > 500000        { SET w TO 5. }
    ELSE IF a > 150000        { SET w TO 4. }
    ELSE IF a > 40000         { SET w TO 3. }
    ELSE IF a > 5000          { SET w TO 2. }
    ELSE                      { SET w TO 1. }
    IF WARP <> w { SET WARP TO w. }
    UPDATE_CRASH_MONITOR().
    UPDATE_INTRO_SEQUENCE().
    WAIT 0.2.
  }
  SET WARP TO 0.
}.

// ===============================================
// DV / thrust utilities
// ===============================================
DECLARE FUNCTION EFFECTIVE_ISP {
  LOCAL thrustSum IS 0.
  LOCAL ispWeighted IS 0.
  LIST ENGINES IN engs.
  FOR e IN engs {
    // Use engines that can make thrust now (in vac near Eve edge this is fine)
    IF e:MAXTHRUST <= 0 { CONTINUE. }
    SET thrustSum TO thrustSum + e:MAXTHRUST.
    SET ispWeighted TO ispWeighted + (e:ISP * e:MAXTHRUST).
  }
  IF thrustSum <= 0 { RETURN 0. }
  RETURN ispWeighted / thrustSum.
}.

DECLARE FUNCTION PROP_MASS {
  LOCAL m IS 0.
  SET m TO m + SHIP:RESOURCES["LiquidFuel"]:AMOUNT * SHIP:RESOURCES["LiquidFuel"]:DENSITY.
  SET m TO m + SHIP:RESOURCES["Oxidizer"]:AMOUNT * SHIP:RESOURCES["Oxidizer"]:DENSITY.
  SET m TO m + SHIP:RESOURCES["MonoPropellant"]:AMOUNT * SHIP:RESOURCES["MonoPropellant"]:DENSITY.
  SET m TO m + SHIP:RESOURCES["XenonGas"]:AMOUNT * SHIP:RESOURCES["XenonGas"]:DENSITY.
  SET m TO m + SHIP:RESOURCES["SolidFuel"]:AMOUNT * SHIP:RESOURCES["SolidFuel"]:DENSITY.
  RETURN m.
}.

DECLARE FUNCTION DV_REMAINING {
  LOCAL isp IS EFFECTIVE_ISP().
  IF isp <= 0 { RETURN 0. }
  LOCAL m0 IS SHIP:MASS.
  LOCAL mp IS PROP_MASS().
  LOCAL mdry IS m0 - mp.
  IF mdry <= 0 { RETURN 0. }
  RETURN isp * G0 * LN(m0 / mdry).
}.

DECLARE FUNCTION CALC_BURN_TIME {
// Most-accurate analytic burn time for a node at (assumed) steady throttle.
// Uses per-engine AVAILABLETHRUST and ISP at current conditions.
// t = (m0 * ve / F) * (1 - exp(-Δv / ve)),  where ve = g0 * Isp_eq
// Isp_eq = (Σ Fi) / (Σ Fi / Ispi)

  PARAMETER nd IS NEXTNODE.
  PARAMETER myThrottle IS 1.0.      // 0.0..1.0 (1.0 = full)

  IF nd = 0 { RETURN 0. }.
  LOCAL dv IS nd:DELTAV:MAG.
  IF dv <= 0 { RETURN 0. }.

  // Collect active engines and compute thrust-weighted Isp
  LIST ENGINES IN es.
  LOCAL FkN IS 0.                  // Σ Fi (kN)
  LOCAL sumFoverIsp IS 0.          // Σ (Fi / Ispi) (kN/s)

  FOR e IN es {
    IF e:IGNITION AND NOT e:FLAMEOUT {
      LOCAL Fi IS e:AVAILABLETHRUST * myThrottle.   // kN
      LOCAL Ispi IS e:ISP.                         // s (current atm)
      IF Fi > 0 AND Ispi > 0 {
        SET FkN TO FkN + Fi.
        SET sumFoverIsp TO sumFoverIsp + Fi / Ispi.
      }
    }
  }

  IF FkN <= 0 OR sumFoverIsp <= 0 { RETURN 0. }.

  // Effective exhaust velocity
  LOCAL g0 IS 9.82.                         // m/s^2
  LOCAL IspEq IS FkN / sumFoverIsp.         // s
  LOCAL ve IS IspEq * g0.                   // m/s

  // Mass & force in consistent units
  LOCAL F   IS FkN * 1000.                  // N
  LOCAL m0  IS SHIP:MASS * 1000.            // kg

  // Analytic burn time under constant F and Isp
  // t = (m0 * ve / F) * (1 - exp(-dv / ve))
  LOCAL eConst IS CONSTANT:E.
  RETURN (m0 * ve / F) * (1 - eConst ^ (-dv / ve)).
}.



// ===============================================
// Precise node executor (auto-warp + Δv feedback)
// ===============================================
DECLARE FUNCTION EXECUTE_MANEUVER {
  PARAMETER execNode IS NEXTNODE.
  PARAMETER autoWarp IS FALSE.
  PARAMETER SETTLE IS 15.
  PARAMETER USE_RCS_TRIM IS TRUE.

  IF NOT HASNODE {
    PRINT "ERROR: no maneuver node.".
    RETURN.
  }
  
  IF execNode:DELTAV:MAG <= 0 {
    PRINT "Node Δv is zero; removing.".
    REMOVE execNode.
    RETURN.
  }
  IF execNode:ETA < -2 {
    PRINT "Node time passed.".
    RETURN.
  }

  LOCAL tburn IS CALC_BURN_TIME(execNode).
  IF tburn <= 0 { PRINT "ERROR: zero thrust or Δv.". RETURN. }

  IF autoWarp = TRUE { WARP_TO_NODE(execNode, tburn/2 + SETTLE). }

  // Recompute immediately before settle
  SET tburn TO CALC_BURN_TIME(execNode).
  IF tburn <= 0 { PRINT "ERROR: cannot compute burn time.". RETURN. }

  // Start steering to node immediately and hold
  LOCK STEERING TO execNode:BURNVECTOR.
  SET STEERINGMANAGER:MAXSTOPPINGTIME TO 0.5.
  SET STEERINGMANAGER:ROLLTS TO 1.0.
  SAS OFF.
  LOG_INFO("Steering locked to node, waiting for burn time...").
  
  // Wait until it's time to burn, but keep steering locked
  WAIT UNTIL execNode:ETA <= (tburn/2).
  LOG_INFO("Burn time reached, starting engines...").

  // Burn with Δv feedback
  LOCK THROTTLE TO 1.
  UNTIL execNode:DELTAV:MAG < 10 {
    LOCK STEERING TO execNode:BURNVECTOR.
    WAIT 0.05.
  }
  LOCK THROTTLE TO 0.2.
  UNTIL execNode:DELTAV:MAG < 0.5 {
    LOCK STEERING TO execNode:BURNVECTOR.
    WAIT 0.05.
  }
  LOCK THROTTLE TO 0.05.
  UNTIL execNode:DELTAV:MAG < 0.05 {
    LOCK STEERING TO execNode:BURNVECTOR.
    WAIT 0.05.
  }
  LOCK THROTTLE TO 0.

  // Optional micro-trim with RCS
  IF USE_RCS_TRIM AND execNode:DELTAV:MAG > 0.02 {
    LOCAL savedRcs IS RCS.
    RCS ON.
    UNTIL execNode:DELTAV:MAG < 0.02 {
      SET SHIP:CONTROL:FORE TO 0.1. WAIT 0.1.
      SET SHIP:CONTROL:FORE TO 0.0. WAIT 0.1.
    }
    IF NOT savedRcs { RCS OFF. }
  }

  UNLOCK THROTTLE.
  UNLOCK STEERING.

  LOCAL rem IS execNode:DELTAV:MAG.
  REMOVE execNode.
  PRINT "Burn done. Δv remaining: " + ROUND(rem,3) + " m/s".
}.

// ===============================================
// MCC planner/tuner
// ===============================================
DECLARE FUNCTION FIND_EVE_PATCH {
  LOCAL res IS 0.
  FOR p IN SHIP:ORBIT:PATCHES {
    IF p:BODY:NAME = "Eve" {
      SET res TO p.
      BREAK.
    }
  }
  RETURN res.
}.

DECLARE FUNCTION NODE_COST {
  PARAMETER nd.
  LOCAL ep IS FIND_EVE_PATCH().
  IF ep = 0 { RETURN 1.0E12. }
  LOCAL peAlt IS ep:PERIAPSIS - ep:BODY:RADIUS.
  LOCAL peErr IS ABS(peAlt - DESIRED_PE_ALT).
  IF NOT TUNE_INCLINATION { RETURN peErr. }
  LOCAL incDeg IS ep:INCLINATION.
  RETURN peErr + (incDeg * 1500).
}.

DECLARE FUNCTION TUNE_NODE {
  PARAMETER nd.
  PARAMETER step IS 4.0.
  PARAMETER minStep IS 0.05.

  LOCAL bestDV IS nd:DELTAV.
  LOCAL bestCost IS NODE_COST(nd).

  UNTIL step < minStep {
    LOCAL improved IS FALSE.

    LOCAL dirs IS LIST().
    dirs:ADD(V(step,0,0)).   // +prograde
    dirs:ADD(V(-step,0,0)).  // -prograde
    dirs:ADD(V(0,step,0)).   // +radial
    dirs:ADD(V(0,-step,0)).  // -radial
    dirs:ADD(V(0,0,step)).   // +normal
    dirs:ADD(V(0,0,-step)).  // -normal

    FOR d IN dirs {
      SET nd:DELTAV TO nd:DELTAV + d.
      WAIT 0.
      LOCAL c IS NODE_COST(nd).
      IF c < bestCost {
        SET bestCost TO c.
        SET bestDV TO nd:DELTAV.
        SET improved TO TRUE.
      }
      SET nd:DELTAV TO nd:DELTAV - d.
      WAIT 0.
    }

    IF improved {
      SET nd:DELTAV TO bestDV.
      WAIT 0.
    } ELSE {
      SET step TO step / 2.
    }
  }
}.

DECLARE FUNCTION PLAN_AND_EXECUTE_EVE_CORRECTION {
  // Ensure we're in Sun SOI
  WAIT UNTIL SHIP:BODY:NAME = "Sun".

  // Node at mid-transfer if Eve is next patch; else fallback to +30 min
  LOCAL etaMid IS 1800.
  // This access assumes patched conics present; if not, fallback holds.
  IF SHIP:ORBIT:NEXTPATCH:BODY:NAME = "Eve" {
    SET etaMid TO MAX(SHIP:ORBIT:NEXTPATCH:ETA * 0.5, 1800).
  }

  LOCAL tNode IS TIME:SECONDS + etaMid.
  LOCAL n IS NODE(tNode, 0, 0, 0).
  ADD n.

  TUNE_NODE(n).

  // If still no Eve encounter, seed small nudge then re-tune
  IF FIND_EVE_PATCH() = 0 {
    LOCAL seeds IS LIST(V(20,0,0), V(-20,0,0), V(0,0,20), V(0,0,-20)).
    FOR s IN seeds {
      SET n:DELTAV TO n:DELTAV + s.
      WAIT 0.
      IF FIND_EVE_PATCH() <> 0 { BREAK. }
      SET n:DELTAV TO n:DELTAV - s.
      WAIT 0.
    }
    TUNE_NODE(n).
  }

  LOCAL ep IS FIND_EVE_PATCH().
  IF ep = 0 {
    PRINT "Warning: no Eve encounter after tune.".
  } ELSE {
    LOCAL peAlt IS ep:PERIAPSIS - ep:BODY:RADIUS.
    PRINT "Planned Eve Periapsis: " + ROUND(peAlt/1000,1) + " km".
    PRINT "Planned Inclination: " + ROUND(ep:INCLINATION,2) + "°".
  }

  EXECUTE_MANEUVER(n, 5, TRUE).
}.

// ===============================================
// Arrival: burn to reserve at edge of atmo, then aerobrake repeatedly
// ===============================================
DECLARE FUNCTION BURN_TO_RESERVE_AT_ENTRY {
  PARAMETER reserveDV IS DV_RESERVE.
  PARAMETER atmTop IS EVE_ATM_TOP.

  WARP_TO_ALT(atmTop).

  IF RETRO_SURFACE {
    LOCK STEERING TO -SHIP:VELOCITY:SURFACE.
  } ELSE {
    LOCK STEERING TO -SHIP:VELOCITY:ORBIT.
  }
  SET STEERINGMANAGER:MAXSTOPPINGTIME TO 0.5.
  SET STEERINGMANAGER:ROLLTS TO 1.0.
  SAS OFF.
  WAIT 2.

  IF EFFECTIVE_ISP() <= 0 OR SHIP:AVAILABLETHRUST <= 0 {
    PRINT "No usable engines for entry burn.".
    UNLOCK STEERING.
    RETURN.
  }

  LOCK THROTTLE TO 1.
  UNTIL DV_REMAINING() <= reserveDV OR SHIP:ALTITUDE < (atmTop - 1000) {
    IF RETRO_SURFACE {
      LOCK STEERING TO -SHIP:VELOCITY:SURFACE.
    } ELSE {
      LOCK STEERING TO -SHIP:VELOCITY:ORBIT.
    }
    UPDATE_CRASH_MONITOR().
    UPDATE_INTRO_SEQUENCE().
    WAIT 0.05.
  }

  LOCK THROTTLE TO 0.2.
  UNTIL DV_REMAINING() <= reserveDV {
    UPDATE_CRASH_MONITOR().
    UPDATE_INTRO_SEQUENCE().
    WAIT 0.05.
    IF SHIP:ALTITUDE < (atmTop - 200) { BREAK. }
  }

  LOCK THROTTLE TO 0.
  UNLOCK THROTTLE.
  UNLOCK STEERING.

  PRINT "Entry burn complete".
  PRINT "Remaining Δv: " + ROUND(DV_REMAINING(),1) + " m/s".
  PRINT "Target reserve: " + reserveDV + " m/s".
}.

DECLARE FUNCTION MULTIPASS_AEROBRAKE_TO_SUBORBITAL {
  PARAMETER atmTop IS EVE_ATM_TOP.
  PARAMETER maxPasses IS 20.

  LOCAL pass IS 0.

  UNTIL (SHIP:APOAPSIS - SHIP:BODY:RADIUS) < atmTop OR pass >= maxPasses {
    SET pass TO pass + 1.
    PRINT "Aerobrake pass " + pass + " ...".

    IF SHIP:ALTITUDE > atmTop + 2000 {
      WARP_TO_ALT(atmTop).
    }

    IF RETRO_SURFACE {
      LOCK STEERING TO -SHIP:VELOCITY:SURFACE.
    } ELSE {
      LOCK STEERING TO -SHIP:VELOCITY:ORBIT.
    }
    SET STEERINGMANAGER:MAXSTOPPINGTIME TO 0.5.
    SET STEERINGMANAGER:ROLLTS TO 1.0.
    SAS OFF.

    WAIT UNTIL SHIP:ALTITUDE < atmTop.
    WAIT UNTIL SHIP:ALTITUDE > atmTop AND SHIP:ORBIT:RADIALVELOCITY > 0.

    UNLOCK STEERING.

    LOCAL apAlt IS (SHIP:APOAPSIS - SHIP:BODY:RADIUS).
    PRINT "After pass " + pass + ": Ap = " + ROUND(apAlt/1000,1) + " km".
    IF apAlt > atmTop {
      WARP_TO_ALT(atmTop + 20000).
    }
  }

  IF (SHIP:APOAPSIS - SHIP:BODY:RADIUS) < atmTop {
    PRINT "Sub-orbital achieved.".
  } ELSE {
    PRINT "Max passes reached.".
  }
}.

DECLARE FUNCTION EVE_ENTRY_AND_AEROBRAKE {
  BURN_TO_RESERVE_AT_ENTRY(DV_RESERVE, EVE_ATM_TOP).
  MULTIPASS_AEROBRAKE_TO_SUBORBITAL(EVE_ATM_TOP, 20).
}.