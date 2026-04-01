// Crash monitoring and recovery system

FUNCTION START_CRASH_MONITOR {
    // Record initial part count with error handling
    LOCAL part_count_attempts IS 0.
    UNTIL part_count_attempts > 3 OR INITIAL_PART_COUNT > 0 {
        SET part_count_attempts TO part_count_attempts + 1.
        SET INITIAL_PART_COUNT TO SHIP:PARTS:LENGTH.
        IF INITIAL_PART_COUNT <= 0 {
            LOG_WARN("Count retry " + part_count_attempts).
            WAIT 0.1.
        }
    }
    
    IF INITIAL_PART_COUNT <= 0 {
        LOG_ERROR("Crash mon disabled").
        SET CRASH_MONITOR_ACTIVE TO FALSE.
        RETURN.
    }
    
    LOG_INFO("Crash monitor started - tracking " + INITIAL_PART_COUNT + " parts (non-blocking)").
    SET CRASH_CHECK_TIME TO TIME:SECONDS + CRASH_CHECK_INTERVAL.
}

// Non-blocking crash check function - call this periodically in your main loop
FUNCTION UPDATE_CRASH_MONITOR {
    IF NOT CRASH_MONITOR_ACTIVE { RETURN. }
    IF TIME:SECONDS < CRASH_CHECK_TIME { RETURN. }
    
    // Time for next check
    SET CRASH_CHECK_TIME TO TIME:SECONDS + CRASH_CHECK_INTERVAL.
    
    // Quick part count check
    LOCAL current_parts IS SHIP:PARTS:LENGTH.
    IF current_parts <= 0 { RETURN. } // Skip if invalid data
    
    LOCAL parts_lost IS INITIAL_PART_COUNT - current_parts.
    
    // Quick critical systems check
    LIST ENGINES IN eng_list.
    IF eng_list:LENGTH = 0 {
        LOG_ERROR("NO ENGINES! Revert").
        KUNIVERSE:REVERTTOLAUNCH().
        RETURN.
    }
    
    // Check for significant part loss
    IF parts_lost >= PART_LOSS_THRESHOLD {
        LOG_ERROR("CRASH! -" + parts_lost + " (" + current_parts + "/" + INITIAL_PART_COUNT + ")").
        KUNIVERSE:REVERTTOLAUNCH().
        RETURN.
    }
    
    // Check for total destruction
    IF current_parts <= 1 {
        LOG_ERROR("DESTROYED! " + current_parts + " left!").
        KUNIVERSE:REVERTTOLAUNCH().
        RETURN.
    }
}

FUNCTION STOP_CRASH_MONITOR {
    SET CRASH_MONITOR_ACTIVE TO FALSE.
    LOG_INFO("Crash monitoring disabled - mission complete").
}

// Additional crash detection for critical systems and Kerbalism failures
FUNCTION CHECK_CRITICAL_PARTS {
    // Simplified engine check with error handling
    LOCAL engine_count IS 0.
    LOCAL broken_engines IS 0.
    
    // Safely check engines
    IF DEFINED eng_list { UNSET eng_list. }
    LIST ENGINES IN eng_list.
    
    IF eng_list:LENGTH = 0 {
        LOG_ERROR("No engines found in engine list").
        RETURN FALSE.
    }
    
    FOR eng IN eng_list {
        SET engine_count TO engine_count + 1.
        
        // Simple checks that are less likely to stall
        LOCAL is_broken IS FALSE.
        
        // Check for Kerbalism reliability failures (with error handling)
        IF eng:HASMODULE("Reliability") {
            LOCAL rel_mod IS eng:GETMODULE("Reliability").
            IF rel_mod:HASEVENT("repair") {
                SET is_broken TO TRUE.
                LOG_WARN("Broken: " + eng:TITLE).
            }
        }
        
        // Check if engine is disabled/non-functional
        IF NOT eng:IGNITION AND eng:FLAMEOUT {
            SET is_broken TO TRUE.
        }
        
        IF is_broken {
            SET broken_engines TO broken_engines + 1.
        }
    }
    
    // Simplified command module check
    LOCAL command_count IS 0.
    LOCAL broken_command IS 0.
    
    // Count command modules safely
    FOR cmd_part IN SHIP:PARTS {
        IF cmd_part:HASMODULE("ModuleCommand") {
            SET command_count TO command_count + 1.
            
            // Simple Kerbalism check
            IF cmd_part:HASMODULE("Reliability") {
                LOCAL cmd_rel_mod IS cmd_part:GETMODULE("Reliability").
                IF cmd_rel_mod:HASEVENT("repair") {
                    SET broken_command TO broken_command + 1.
                    LOG_WARN("Broken cmd: " + cmd_part:TITLE).
                }
            }
        }
    }
    
    // Critical failure conditions (simplified)
    IF engine_count = 0 {
        LOG_ERROR("NO ENGINES! Revert").
        RETURN FALSE.
    }
    
    IF command_count = 0 {
        LOG_ERROR("NO COMMAND! Revert").
        RETURN FALSE.
    }
    
    // Check if too many engines are broken by Kerbalism
    IF broken_engines >= engine_count {
        LOG_ERROR("Engs broke:" + broken_engines + "/" + engine_count + " Revert").
        RETURN FALSE.
    }
    
    // Check if command modules are broken by Kerbalism
    IF broken_command >= command_count {
        LOG_ERROR("Cmd broke:" + broken_command + "/" + command_count + " Revert").
        RETURN FALSE.
    }
    
    // Warn about partial failures but continue
    IF broken_engines > 0 {
        LOG_WARN(broken_engines + "/" + engine_count + " engines bad").
    }
    
    IF broken_command > 0 {
        LOG_WARN(broken_command + "/" + command_count + " cmd broken").
    }
    
    RETURN TRUE.
}