// Music player for KOS by Fooney.
// The "song" is a list of "parts". Each "part" is a list of "notes".
// This was designed for transposing songs from sheet music.
// 


setBPM(75).
// Set up note length in UT time. This is for any music in 4. needs more work to get other time signatures to work.
function SETBPM {
    parameter BPM.
	print "set to bpm: " + BPM.
    SET wholeNote TO 4 * (60 / BPM).
	// PRINT "wholeNote duration: " + wholeNote.
    SET halfNote TO 2 * (60 / BPM).
    SET quarterNote TO 1 * (60 / BPM). //One beat in 4
    SET eighthNote TO 0.5 * (60 / BPM).
	// PRINT "eighthNote duration: " + eighthNote.
    SET sixteenthNote TO 0.25 * (60 / BPM).
    SET thirtysecondNote TO 0.125 * (60 / BPM).
	// PRINT "thirtysecondNote duration: " + thirtysecondNote.
}

SET voices TO LIST(getVoice(0), getVoice(1), getVoice(2), getVoice(3),getVoice(4)).

// Play a single note
function playNote {
    parameter note, duration.
    print "Playing note " + note + " for " + duration + " seconds".
    voices[4]:PLAY(NOTE(note, duration)).
}

// Play a chord
function playChord {
    parameter chord, duration.
    for index in range(0, chord:LENGTH - 1) {
        voices[index]:PLAY(NOTE(index, duration)).
    }
}


// Chord list
SET cchord TO list("C3", "F3", "F2", "C4").  // Csus4 without the third or C-F dyad
SET dchord TO list("D3", "G3", "G2", "D4").  // Dsus4 without the third or D-G dyad
SET bchord TO list("F3", "D3", "B3", "D4").  // B diminished (Bdim)


// All parts will play at the same time.
// need to set up loading from another file.

// SETBPM(75).
SET pianoPart TO Lexicon(
	"note1", LIST("bpmTimeChange", 75),
    "note2", LIST(cchord, wholeNote),
    "note3", LIST(dchord, wholeNote),
    "note4", LIST(bchord, wholeNote),
	"note5", LIST ("END", 0)
).
SET voicePart TO Lexicon(
	"note1", LIST("bpmTimeChange", 75),
	"note2", LIST("R", eighthNote),
    "note3", LIST("a4", eighthNote),
    "note4", LIST("b4", eighthNote),
    "note5", LIST("c3", eighthNote),
    "note6", LIST("d3", eighthNote),
	"note7", LIST ("END", 0)
    // "note6", LIST("e4", eighthNote),
    // "note7", LIST("f4", halfNote)
).
SET song TO Lexicon(
	"part1", pianoPart,
	"part2", voicePart
).
SET songTiming to list(LIST("step"), LIST("action"), list("duration")).//step and action
// print songTiming.
// Main loop
setBPM(75).
function buildSong {
	// local startTime is 0. //Main clock for note timing
	// set songTiming[0] to currentStep.
    local stepTime is thirtysecondNote. // step time in UT. Changes with BPM. If you need smaller just double BPM.
    // print "step time: " + stepTime.


	local currentStep is 0. //This is current UT play time in seconds.

	for partKey in song:keys {
    	local part is song[partKey].
    	// print "Processing part: " + partKey.
 	// local value is part:values.
		FOR lineKey in part:keys{
			local notelist is part[lineKey].
			local thisNote is notelist[0].
			local thisDur is notelist[1].

			if thisNote = "bpmTimeChange"{
					songTiming[0]:add(currentStep).
					songTiming[1]:add(thisNote).
					songTiming[2]:add(thisDur).
				 setbpm(notelist[1]).
			}else{
				 if thisNote:istype("string"){
					songTiming[0]:add(currentStep).
					songTiming[1]:add(thisNote).
					songTiming[2]:add(thisDur).
					set currentStep to currentStep + thisDur.
					// playNote(notelist[0], notelist[1]).
					// print "this is a single note".
				} else if thisNote:istype("list"){
					songTiming[0]:add(currentStep).
					songTiming[1]:add(thisNote).
					songTiming[2]:add(thisDur).
					set currentStep to currentStep + thisDur.
					// print "this is chords".
				} else {
					print "NOTE MISSED" + thisNote.
				}
			}
		}
}
// print "songTiming"+songTiming.
writeJson(songTiming, "output.json").
    // print "Total Time: " + totalTime.
    // set totalSteps to round(totalTime / stepTime).
    // print "Total Steps: " + totalSteps.
}
buildSong().

// print songTiming[0]:length.
for step in songTiming[0]{
	print step.
}
function playSong {
local currentTime is 0.
local startTime is time:seconds.
local stepTime is thirtysecondNote.
local stepIndex is 0.
local songLng is songTiming[0]:length.

for step in songTiming[0]{
	set stepIndex to stepIndex + 1.
		if stepIndex >= songLng -5 {
			local stepIndexplusOne is stepIndex + 1.
		if not (DEFINED songTiming[0][stepIndexplusOne]){
			print "end of song.....".
		}else{
			
			local nextNoteTime to songTiming[0][stepIndexplusOne].
		}
		
		
		print "nextNoteTime: " + nextNoteTime.
		until currentTime >= nextNoteTime{
			// print "waiting: "+ stepTime.
			
			set currentTime to currentTime + stepTime.
			print "currentTime: " + currentTime.
			// print "stepTime: " + stepTime.
			// set stepIndex to stepIndex + 1.
			print "stepIndex: " + stepIndex.
			wait stepTime.
			// break.
			}
	// }else if stepIndex <= songLng{
	} else {
		print "Ended on step: " + stepIndex.
		print "songLng = " + songLng.
	}
	local totalsteps is songTiming[0]:length.
	print "stepIndex: " + stepIndex +" step " + step 
	+" totalsteps: " + "" + totalsteps.
	local currentAction is songTiming[1][stepIndex].
	print "songTiming[1][stepIndex] " + currentAction.
	local actionDuration is songTiming[2][stepIndex].
	print "songTiming[2][stepIndex] " + actionDuration.
	// if songTiming[1][i] <= songLng{
		// local waitdur1 to 0.
		// local waitdur2 to 0.
		// print "Current action type: " + currentAction:typename.
		if currentAction = "bpmTimeChange"{
			print "this is a bpm change".
			SETBPM(actionDuration).
			
		}else if currentAction:istype("string"){
			print "this is a single note".
		playNote(currentAction, actionDuration).
		//  set waitdur1 to actionDuration.
		} else if currentAction:istype("list"){
			print "this is chords"+ currentAction + actionDuration.
		playChord(currentAction, actionDuration).
		// set waitdur2 to actionDuration.
		// } else if currentAction = "bpmTimeChange"{
		// 	print "this is a bpm change".
		// 	SETBPM(actionDuration).
		}
	set currentTime to currentTime + songTiming[0][stepIndex].
	print "current real time: " + currentTime.
	// set timePassed to tim
	//set waitTime to songTiming[0][stepIndex + 1].
	// wait stepTime.
	print "stepIndex < songLng " + stepIndex + +" "+ songLng.

	// set stepIndex to stepIndex + 1.
	// print "stepindex after loop::: " + stepIndex.
	}
	}
playSong().

