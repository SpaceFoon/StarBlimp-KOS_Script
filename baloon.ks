//Find all baloons and fill them
for PART in SHIP:PARTS {
     if PART:HASMODULE("HLEnvelopePartModule") {
        print "part: " + part:name.
        PART:GETMODULE("HLEnvelopePartModule"):DOACTION("buoyancy max", true).
    }
}
//Dont fill this one as much to balance.
for PART in SHIP:PARTS {
    if PART:NAME = "hl10NoseCone" {
        // Loop to press the "buoyancy --" button 20 times
        set buttonPresses to 0.
        until buttonPresses >= 39 {
            PART:GETMODULE("HLEnvelopePartModule"):DOACTION("buoyancy --", true).
            print "buoyancy -- action triggered.".
            set buttonPresses to buttonPresses + 1.
            WAIT 0.1.  // Add a short delay to prevent rapid execution
        }

    }
}


