// function calculateBurnTime {

//     set totalDeltaV to nextnode:DELTAV.

//     set initialMass to ship:MASS.

//     set totalThrust to 0.
//     set totalISP to 0.
//     LIST ENGINES IN engList.
//     for eng in engList {
//         if eng:IGNITION {
//             set totalThrust to totalThrust + eng:MAXTHRUST.
//             set totalISP to totalISP + (eng:ISP * eng:MAXTHRUST).
//              print eng:ISP.
//              print eng:MAXTHRUST.
//         }
//     }
//     set averageISP to totalISP / totalThrust.

//     // Calculate the final mass after the burn using the Tsiolkovsky rocket equation
//     set finalMass to initialMass / (CONSTANT:E ^ (totalDeltaV / (averageISP * CONSTANT:G0))).

//     set massFlowRate to totalThrust / (averageISP * CONSTANT:G0).

//     set burnTime to (initialMass - finalMass) / massFlowRate.

//     return burnTime.
// }

// calculateBurnTime().

// Calculate the burn time to complete a burn of a fixed Δv

// https://www.reddit.com/r/Kos/comments/3ftcwk/compute_burn_time_with_calculus/

// Base formulas:
// Δv = ∫ F / (m0 - consumptionRate * t) dt
// consumptionRate = F / (Isp * g)
// ∴ Δv = ∫ F / (m0 - (F * t / g * Isp)) dt

// Integrate:
// ∫ F / (m0 - (F * t / g * Isp)) dt = -g * Isp * log(g * m0 * Isp - F * t)
// F(t) - F(0) = known Δv
// Expand, simplify, and solve for t

FUNCTION MANEUVER_TIME {
  PARAMETER dV.

  LIST ENGINES IN en.

  LOCAL f IS en[0]:MAXTHRUST * 1000.  // Engine Thrust (kg * m/s²)
  LOCAL m IS SHIP:MASS * 1000.        // Starting mass (kg)
  LOCAL e IS CONSTANT:E.            // Base of natural log
  LOCAL p IS en[0]:ISP.               // Engine ISP (s)
  LOCAL g IS 9.80665.                 // Gravitational acceleration constant (m/s²)

  RETURN g * m * p * (1 - e^(-dV/(g*p))) / f.
}

FUNCTION calculateBurnTime {
  set totalDeltaV to nextnode:DELTAV.

  LIST ENGINES IN en.

  LOCAL f IS en[0]:MAXTHRUST * 1000.  // Engine Thrust (kg * m/s²)
  LOCAL m IS SHIP:MASS * 1000.        // Starting mass (kg)
  LOCAL e IS CONSTANT:E.            // Base of natural log
  LOCAL p IS en[0]:ISP.               // Engine ISP (s)
  LOCAL g IS 9.80665.                 // Gravitational acceleration constant (m/s²)
  return g * m * p * (1 - e^(-totalDeltaV/(g*p))) / f.

}


PRINT "Time for a 100m/s burn: " + MANEUVER_TIME(100).
PRINT "Time for a 200m/s burn: " + MANEUVER_TIME(200).
PRINT "Time for a 300m/s burn: " + MANEUVER_TIME(300).
PRINT "Time for a 400m/s burn: " + MANEUVER_TIME(400).
PRINT "Time for a 500m/s burn: " + MANEUVER_TIME(500).
PRINT "Time for a 1000m/s burn: " + MANEUVER_TIME(1000).