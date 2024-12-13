// Function to print the status of custom action groups
function printCustomActionGroupStatus {
    local groupNames is LIST("Custom01", "Custom02", "Custom03", "Custom04", "Custom05", "Custom06", "Custom07", "Custom08", "Custom09", "Custom10").

    for name in groupNames {
        local groupStatus is AGX:GETGROUP(name).
        if groupStatus {
            print name + ": Activated".
        } else {
            print name + ": Deactivated".
        }
    }
}

// Function to print the status of built-in action groups
function printBuiltInActionGroupStatus {
    local builtInGroupNames is LIST("AG1", "AG2", "AG3", "AG4", "AG5", "AG6", "AG7", "AG8", "AG9", "AG10").

    for name in builtInGroupNames {
        local groupStatus is AGX:GETGROUP(name).
        if groupStatus {
            print name + ": Activated".
        } else {
            print name + ": Deactivated".
        }
    }
}

// Main script
print "Checking custom action groups:".
printCustomActionGroupStatus().
print "".

print "Checking built-in action groups:".
printBuiltInActionGroupStatus().
