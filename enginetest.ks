// Define the minimum thrust threshold
set minThrust to 10.0.  // Adjust this value as needed

// Function to get all active engines
function getActiveEngines {
    set engines to list().
    for part in SHIP:PARTS {
        if part:HASMODULE("ModuleEngines") {
            local engine is part:GETMODULE("ModuleEngines").
            print "mod found:" + part.
            if engine:IGNITION = true{
                engines:add(engine).
            }
        }
    }
    return engines.
}

// Function to monitor and shut down engines
function monitorEngines {
    local activeEngines is getActiveEngines().
    for engine in activeEngines {
        if engine:THRUST < minThrust {
            print "Engine thrust too low, shutting down: " + engine:PART:NAME.
            engine:SHUTDOWN().
        }
    }
}

// Main loop
until false {
    monitorEngines().
    print engines.
    WAIT 0.5.  // Adjust the wait time as needed
}
