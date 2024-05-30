// Function to monitor and shut down engines
set minThrust to 10.0.
function monitorEngines {
    list engines in engList.  // Get a list of all engines
    for eng in engList {
        if eng:PART:NAME = "WBILargeElectricPart"{
            print "Engine: " + eng:NAME + " Thrust: " + eng:THRUST.
            if eng:THRUST < minThrust {
                print "Engine thrust too low, shutting down: " + eng:NAME.
                eng:SHUTDOWN().
            }
        }
        if eng:PART:NAME = "turboFanEngine" {
            print "Engine: " + eng:NAME + " Thrust: " + eng:THRUST.
            if eng:THRUST < minThrust {
                print "Engine thrust too low, shutting down: " + eng:NAME.
                eng:SHUTDOWN().
                INTAKES OFF.
            }
        }
    }
}

// Main loop
until false {
    monitorEngines().
    wait 1.
}
LIST ENGINES IN engList. // Get a list of all engines
FOR eng IN engList {
    IF eng:name = "WBILargeElectricPart" {
        eng:setfield("IntakeStatus", "Closed").
    }
}

