// Centralized action group mapping for the Airship Operating System.
// Provides a globally accessible lexicon keyed by descriptive names.

DECLARE GLOBAL AGX_LEGEND TO LEXICON().

// Primary flight systems
SET AGX_LEGEND["legs"] TO 1.
SET AGX_LEGEND["fan_system"] TO 2.
SET AGX_LEGEND["jets"] TO 3.
SET AGX_LEGEND["reverse_fan"] TO 4.
SET AGX_LEGEND["flaps_down"] TO 5.
SET AGX_LEGEND["flaps_up"] TO 6.
SET AGX_LEGEND["pitch_w/airbrake"] TO 7.
SET AGX_LEGEND["e_stop_kos"] TO 8.
// SET AGX_LEGEND["landing_lights"] TO 9.
// Never use fireworks with kos.
SET AGX_LEGEND["celebration"] TO 10.

// Search Light
SET AGX_LEGEND["m.t.s"] TO 11.

// Balloon stuff
// Emergency main ballast blow: You go up fast.
SET AGX_LEGEND["embt_blow"] TO 12.
// Crash dive: You go down fast.
SET AGX_LEGEND["crash_dive"] TO 13.

SET AGX_LEGEND["maineng_cutoff"] TO 14.
// Tiny apu for emergency power.
SET AGX_LEGEND["e-generator"] TO 15.
SET AGX_LEGEND["nuke_reactor"] TO 16.
SET AGX_LEGEND["gas_generator"] TO 17.
// robotics
SET AGX_LEGEND["motor_apu_on"] TO 19.
SET AGX_LEGEND["motor_locks_on"] TO 20.
SET AGX_LEGEND["motor_apu_off"] TO 44.
SET AGX_LEGEND["motor_locks_off"] TO 45.

// air
SET AGX_LEGEND["compressors"] TO 27.
SET AGX_LEGEND["intakes_open"] TO 28.
SET AGX_LEGEND["intakes_closed"] TO 29.
SET AGX_LEGEND["spoilers"] TO 30.
SET AGX_LEGEND["jet_shroud"] TO 31.



// Lighting
SET AGX_LEGEND["landing_lights"] TO 9.
SET AGX_LEGEND["nav_lights"] TO 26.
SET AGX_LEGEND["lights_runway"] TO 51.
SET AGX_LEGEND["lights_gondo"] TO 52.
SET AGX_LEGEND["lights_hab"] TO 53.
SET AGX_LEGEND["lights_accent"] TO 54.
SET AGX_LEGEND["lights_spot"] TO 55.
SET AGX_LEGEND["innerflood_lights"] TO 56.
SET AGX_LEGEND["lights_ground_effect"] TO 57.

// Mechanical systems
SET AGX_LEGEND["hangar_door"] TO 36.
// this also powers on the motors and unlocks them 
SET AGX_LEGEND["side_solar_open"] TO 58.
// this doesn't power off or lock anything
SET AGX_LEGEND["side_solar_close"] TO 83.

SET AGX_LEGEND["boom_open"] TO 59.
SET AGX_LEGEND["boom_solar_deploy"] TO 60.
SET AGX_LEGEND["boom_close"] TO 84.
SET AGX_LEGEND["boom_solar_retract"] TO 85.
SET AGX_LEGEND["drill_motors"] TO 80.
SET AGX_LEGEND["drill_motor_locks"] TO 81.

// Communications
SET AGX_LEGEND["comms_long"] TO 22.
SET AGX_LEGEND["comms_hga"] TO 23.
SET AGX_LEGEND["comms_long_retract"] TO 47.
SET AGX_LEGEND["comms_short_retract"] TO 48.

// All scnners
SET AGX_LEGEND["scanners_on"] TO 25.
// Thermal / safety systems
SET AGX_LEGEND["cooling_system"] TO 50.

// About trigger for marquee lights
SET AGX_LEGEND["abort"] TO 112.

// Decorative marquee light layout
SET AGX_LEGEND["marquee_lower_row"] TO LIST(101, 102, 136, 111, 111, 136, 103, 104, 105, 106, 107, 108, 109).
SET AGX_LEGEND["marquee_upper_row"] TO LIST(126, 127, 128, 129, 130, 131, 132, 133, 134).

// Light sequence for startup (excludes hab, spot, innerflood)
SET AGX_LEGEND["startup_lights"] TO LIST(9, 26, 51, 52, 54, 57).

// Function to turn on startup lights with 0.5 second delays
FUNCTION STARTUP_LIGHT_SEQUENCE {
  LOCAL startup_lights IS AGX_LEGEND["startup_lights"].
  FOR light_ag IN startup_lights {
    AG_TOGGLE(light_ag).
    WAIT 0.5.
  }.
}.
