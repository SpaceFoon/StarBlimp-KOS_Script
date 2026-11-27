// Call once at startup to set all constants as global variables
FUNCTION INIT_AOS_CONSTANTS {
	// Set global variables directly for better performance
	SET DESIRED_PE_ALT TO 82000.      // target Eve periapsis (m)
	SET TUNE_INCLINATION TO TRUE.     // lightly penalize Eve patch inclination in tuning
	SET EVE_ATM_TOP TO 90000.         // Eve atmosphere top (m)
	SET DV_RESERVE TO 50.             // leave ~this much Δv after entry burn (m/s)
	SET RETRO_SURFACE TO FALSE.       // TRUE: surface retrograde, FALSE: orbital retrograde

	SET G0 TO 9.80665.
	SET TargetAltitude TO 85000.
	SET GravCst TO KERBIN:MU / KERBIN:RADIUS^2.
	SET TargetOrbitalSpeed TO 600000 * SQRT(GravCst/(600000+TargetAltitude)).
	SET Staggincount TO 3.
	SET PitchingSteer TO 90.
	SET AdjustedThrottle TO 1.
	SET DragCoef TO 1.
	SET CrossSection TO 1.25.
}

