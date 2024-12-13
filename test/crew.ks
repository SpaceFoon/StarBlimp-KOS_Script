    // Check crew count
    local crewCount is 0.
    if SHIP:crewcapacity > 0 {
        set crewCount to SHIP:CREW:LENGTH.
    } else {
        set crewCount to 0.
    }
    
    PRINT "Crew count: " + crewCount.