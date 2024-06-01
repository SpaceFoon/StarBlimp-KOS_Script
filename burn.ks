function calculateBurnTime {

    set totalDeltaV to nextnode:DELTAV.

    set initialMass to ship:MASS.

    set totalThrust to 0.
    set totalISP to 0.
    LIST ENGINES IN engList.
    for eng in engList {
        if eng:IGNITION {
            set totalThrust to totalThrust + eng:thrust.
            set totalISP to totalISP + (eng:ISP * eng:MAXTHRUST).
             print eng:ISP.
        }
    }
    set averageISP to totalISP / totalThrust.

    // Calculate the final mass after the burn using the Tsiolkovsky rocket equation
    set finalMass to initialMass / (CONSTANT:E ^ (totalDeltaV / (averageISP * CONSTANT:G0))).

    set massFlowRate to totalThrust / (averageISP * CONSTANT:G0).

    set burnTime to (initialMass - finalMass) / massFlowRate.

    return burnTime.
}

calculateBurnTime().