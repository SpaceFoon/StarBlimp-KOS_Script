// General PID values
print "PID P Gain (Proportional): " + STEERINGMANAGER:PITCHPID:KP.
print "PID I Gain (Integral): " + STEERINGMANAGER:PITCHPID:KI.
print "PID D Gain (Derivative): " + STEERINGMANAGER:PITCHPID:KD.



// Yaw PID values
print "Yaw P Gain: " + STEERINGMANAGER:PITCHPID:KP.
print "Yaw I Gain: " + STEERINGMANAGER:PITCHPID:KI.
print "Yaw D Gain: " + STEERINGMANAGER:PITCHPID:KD.

// Roll PID values
print "Roll P Gain: " + steeringmanager:ROLLPID:KP.
print "Roll I Gain: " + steeringmanager:ROLLPID:KI.
print "Roll D Gain: " + steeringmanager:ROLLPID:KD.

//Special values
print "Max Stopping Time: " + steeringmanager:MAXSTOPPINGTIME.
print "Pitch S: " + STEERINGMANAGER:PITCHTS.
print "Roll S: " + STEERINGMANAGER:ROLLTS.

print "Steering: " + STEERING.
print "Throttle: " + throttle.