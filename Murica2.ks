FUNCTION SETDURATION {
    parameter BPM.
    // parameter timeSigniture.
    // IF timeSigniture = 44 OR timeSigniture = 24{
        SET wholeNote TO 4 * (60 / BPM).
        SET halfNote TO 2 * (60 / BPM).
        SET quarternote TO 1 * (60 / BPM).
        SET eigththNote TO .5 * (60 / BPM).
        SET sixteenthNote TO .25 * (60 / BPM).
        SET thirtysecondNote TO .125 * (60 / BPM).
    // }
}


// List of voices for chords
// SET chordVoices TO LIST().
// for i IN RANGE(1, 5) {
//     LOCAL voice IS GetVoice(i).
//     configureVoice(voice, 0.7, "sine", 0.01, 0.3, 0.2, 0.5).
//     chordVoices:ADD(voice).
// }


SET singleNoteVoice TO GetVoice(0).

// Configure voice parameters
// SET chordVoice:VOLUME TO 0.5.
// SET chordVoice:WAVE TO "sawtooth".
// SET chordVoice:ATTACK TO 0.001.
// Configure other ADSR parameters for the chord voice

SET singleNoteVoice:VOLUME TO 0.7.
SET singleNoteVoice:WAVE TO "sine".
SET singleNoteVoice:ATTACK TO 0.005.
// Configure other ADSR parameters for the single note voice

// Construct notes


// Play notes simultaneously
// chordVoice:PLAY(chordNote).
// singleNoteVoice:PLAY(singleNote).


// Function to play a chord (or cluster)
function playChord {
    parameter chord, duration.
    // parameter chord, duration.
    // Play each note in the chord with a different voice
    for chord in chord.{
        for i in range(0, 3) {
                // SET chordVoice TO GETVOICE(i+ 1).
                // print chordVoice.
                if i = 0{
                    set chordVoice1 to getVoice(0).
                    chordVoice1:PLAY(NOTE(chord[0], duration)).
                }
                if i = 1{
                    set chordVoice2 to getVoice(1).
                    chordVoice2:PLAY(NOTE(chord[1], duration)).
                }
                if i = 2{
                    set chordVoice3 to getVoice(2).
                    chordVoice3:PLAY(NOTE(chord[2], duration)).
                }
                if i = 3{
                    set chordVoice4 to getVoice(3).
                    chordVoice4:PLAY(NOTE(chord[3], duration)).
                }
            }
    }
    WAIT duration.
}
// singleNoteVoice:PLAY(NOTE("d3", wholeNote, 2)).
    // SET wholeNote TO 4 * (60 / BPM).
    // SET halfNote TO 2 * (60 / BPM).
    // SET quarternote TO 1 * (60 / BPM).
    // SET eigththNote TO .5 * (60 / BPM).
    // SET sixteenthNote TO .25 * (60 / BPM).
    // SET thirtysecondNote TO .125 * (60 / BPM).
    // V0:PLAY( NOTE(freq, duration) ).
    // Waits for the duration of the note
 SETDURATION(75).
SET cchord TO LIST(
    "c3",
    "f3", 
    "F2", 
    "c4"
).
SET dchord TO LIST(
    "d3",
    "g3",
    "g2",
    "d4"
).
SET fchord TO LIST(
    "f3",
    "d3",
    "b3",
    "d4"
).
SETDURATION(75).
// Function to calculate the total duration of a song
function calculateTotalDuration {
    parameter voicePart.
    LOCAL totalDuration IS 0. // Initialize total duration to zero

    // Iterate through each part in the song
    for step IN voicePart {
        // Check if the part is a single note or a chord
        if part:LENGTH > 1 {
            // Iterate through each element in the part and sum up the durations
            for i IN RANGE(1, step:LENGTH) {
                SET totalDuration TO totalDuration + step[i]:DURATION. // Assuming each element has a DURATION property
            }
        }
    }

    RETURN totalDuration.
}

// Example usage:
SET totalDurationResult TO calculateTotalDuration(voicePart).
PRINT "Total duration of the song: " + totalDurationResult.

// playChord(chord2, wholeNote).
Set voicePart to LIST(
    SETDURATION(75),
    //| = bar.
    playChord(cchord, wholeNote),
    note("R", quarternote + eigththNote),
    
    note("C4", eigththNote), // a-
    // hudtext("A", 30, 2, 50, yellow, false),
    note("C4", eigththNote), // m-
    // hudtext("A-", 30, 2, 50, yellow, false),
    note("A3", eigththNote), // er
    // hudtext("A-mer", 30, 2, 50, yellow, false),
    //|
    note("F3", eigththNote), // -
    // hudtext("A-mer -", 30, 2, 50, yellow, false),
    note("C4", eigththNote), // i-
    // hudtext("A-mer - i-", 30, 2, 50, yellow, false),
    note("C4", halfNote),// ca
    hudtext("A-mer - i-ca", 15, 2, 50, yellow, false)
    //|
    //  SLIDENOTE("C5", "F5", 0.45, 0.5), // half note that slides from C5 to F5 as it goes.
    // note("R", quarternote + eigththNote),
    // note("d4", eigththNote),// a-
    // note("d4", eigththNote),// m-
    // note("b3", eigththNote),// er
    // //|
    // note("g3", eigththNote), // -
    // note("d4", eigththNote), // i-
    // note("d4", halfNote), // ca
    // //|
    // note("R", quarternote + eigththNote),
    // note("d4", eigththNote),// a-
    // note("d4", eigththNote),// m-
    // note("b3", eigththNote),// er
    // //|
    // note("f3", eigththNote),// -
    // note("d4", eigththNote),// i-
    // note("d4", halfNote),// ca
    // //|
    // note("R", quarternote + eigththNote),
    // note("R", eigththNote),// a-
    // note("e4", eigththNote),// m-
    // note("e4", eigththNote),// er
    // //|
    // note("c3", eigththNote),// -
    // note("g3", eigththNote),// i-
    // note("e4", eigththNote),// ca
    // note("e4", halfNote),

    // SETDURATION(165),

    // // Americaaaa 
    //  note("R", halfNote),
    // note("f4", quarternote), //Mer
    // note("f4", eigththNote), //i
    // note("d4", eigththNote + quarternote), //ca
    // note("R", halfNote),
    // // \
    // note("c4", quarternote), //fuck
    // note("d4", halfNote), //yea
    // note("R", quarternote),
    // //|
    // note("c4", eigththNote),  // com
    // note("c4", eigththNote), // in
    // note("c4", eigththNote), // a
    // note("c4", quarternote + eigththNote), // gain
    // note("c4", eigththNote), // to 
    // note("c4", eigththNote), // save 
    // note("a3", eigththNote), // the 
    // note("c4", eigththNote), // mo
    // note("a3", eigththNote), // ther
    // note("c4", eigththNote), // fuck
    // note("a3", eigththNote), // n
    // note("f4", quarternote), // day
    // note("d4", quarternote), // yea

    //    // Americaaaa 
    //  note("R", halfNote),
    // note("f4", quarternote), //Mer
    // note("f4", eigththNote), //i
    // note("d4", eigththNote + quarternote), //ca
    // note("R", halfNote),
    // // \
    // note("c4", quarternote), //fuck
    // note("d4", halfNote), //yea
    // note("R", quarternote),

    // note("c4", eigththNote),
    // note("c4", quarternote),
    // note("c4", eigththNote + quarternote),
    // note("a3", quarternote),
    // //|
    // note("c4", quarternote),
    // note("a3", quarternote),
    // note("f4", quarternote),
    // note("d4", quarternote)
    // note("e4", quarternote).
    // note("e4", eigththNote).
    // note("d4", quarternote 
    // + eigththNote).
    // // note("d4", eigththNote).
    // note("R", quarternote).
    // note("c4", quarternote).
    // note("d4", halfNote).
    // Rest for 1/8
    // WAIT 0.125.
    
    // FUCK YEAH (E for 1/2 note)
    // note("E4", 0.5).   // E4
).
SET pianopart to LIST(
    // SETDURATION(75),
cchord, halfNote + quarternote,
dchord, halfNote + quarternote,
fchord, halfNote + quarternote
).

// function playPianoPart {
//     for chordInfo in pianoPart {
//         playChord(chordInfo[0], chordInfo[1]).
//     }
// }
//  playPianoPart().
singleNoteVoice:PLAY(voicePart).

// Function to set note durations based on BPM
function setDuration {
    parameter BPM.
    SET wholeNote TO 4 * (60 / BPM).
    SET halfNote TO 2 * (60 / BPM).
    SET quarterNote TO 1 * (60 / BPM).
    SET eighthNote TO 0.5 * (60 / BPM).
    SET sixteenthNote TO 0.25 * (60 / BPM).
    SET thirtySecondNote TO 0.125 * (60 / BPM).
}

// Function to configure a voice with given parameters
function configureVoice {
    parameter voice, vol, wave, attack, decay, sustain, release.
    SET voice:VOLUME TO vol.
    SET voice:WAVE TO wave.
    SET voice:ATTACK TO attack.
    SET voice:DECAY TO decay.
    SET voice:SUSTAIN TO sustain.
    SET voice:RELEASE TO release.
}

// // Function to adjust volume based on dynamics
// function getVolume {
//     parameter dynamic.
//     if dynamic = "pp" { return 0.3. }
//     if dynamic = "p" { return 0.5. }
//     if dynamic = "mp" { return 0.6. }
//     if dynamic = "mf" { return 0.7. }
//     if dynamic = "f" { return 0.8. }
//     if dynamic = "ff" { return 1.0. }
//     return 0.7. // Default volume
// }

// // Function to play a single note with dynamics, articulation, and fade
// function playSingleNote {
//     parameter note, duration, dynamic, articulation, fadeInfo.
//     LOCAL volume IS getVolume(dynamic).
//     LOCAL adjustedDuration IS duration.

//     // Adjust duration for staccato
//     if articulation = "staccato" {
//         SET adjustedDuration TO adjustedDuration * 0.5.
//     }

//     // Adjust volume based on fade information
//     if fadeInfo:START_VOLUME >= 0 AND fadeInfo:END_VOLUME >= 0 AND fadeInfo:STEPS > 0 {
//         SET volume TO LERP(fadeInfo:START_VOLUME, fadeInfoEND_VOLUME, fadeInfo:CURRENT_STEP / fadeInfo:STEPS).
//         set fadeInfo:CURRENT_STEP to fadeInfo:CURRENT_STEP + 1.
//     }

//     // Set voice parameters
//     SET singleNoteVoice:VOLUME TO volume.
//     IF articulation = "staccato"{
//         SET singleNoteVoice:DECAY TO (0.05).
//         SET singleNoteVoice:SUSTAIN TO (0.0).
//         SET singleNoteVoice:RELEASE TO (0.1).
//     } else{
//         SET singleNoteVoice:DECAY TO (0.3).
//         SET singleNoteVoice:SUSTAIN TO (0.7).
//         SET singleNoteVoice:RELEASE TO (0.5).
//     }


//     // Play the note
//     singleNoteVoice:PLAY(NOTE(note, adjustedDuration)).
//     WAIT duration.
// }

// // Function to play a four-note chord with dynamics, articulation, and fade
// function playChord {
//     parameter chord, duration, dynamic, articulation, fadeInfo.
//     LOCAL volume IS getVolume(dynamic).
//     LOCAL adjustedDuration IS duration.

//     // Adjust duration for staccato
//     if articulation = "staccato" {
//         set adjustedDuration to 0.5.
//     }

//     // Adjust volume based on fade information
//     if fadeInfo:START_VOLUME >= 0 AND fadeInfo:END_VOLUME >= 0 AND fadeInfo:STEPS > 0 {
//         set volume to LERP(fadeInfo:START_VOLUME, fadeInfo:END_VOLUME, fadeInfo:CURRENT_STEP / fadeInfo:STEPS).
//         set fadeInfo:CURRENT_STEP to 1.
//     }

//     for i IN RANGE(0, 4) {
//         LOCAL voice IS chordVoices[i].
//         // Set voice parameters
//         set voice:VOLUME TO volume.

//         IF articulation = "staccato"{
//             SET voice:DECAY TO (0.05).
//             SET voice:SUSTAIN TO (0.0).
//             SET voice:RELEASE TO (0.1).
//         } else{
//             SET voice:DECAY TO (0.3).
//             SET voice:SUSTAIN TO (0.7).
//             SET voice:RELEASE TO (0.5).
//         }
//         // Play the chord note
//         voice:PLAY(NOTE(chord[i], adjustedDuration)).
//     }
//     WAIT duration.
// }

// // Function to play a song with dynamics, articulation, and fade
// function playSong {
//     parameter song.
//     print song.
//     LOCAL fadeInfo IS LIST().

//     for part IN song {
//         if part:LENGTH = 5 {
//             // Single note with dynamics, articulation, and optional fade
//             if part:LENGTH = 6 { // Contains fade information
//                  SET fadeInfo:START_VOLUME TO getVolume(part[4]).
//                  SET fadeInfo:END_VOLUME TO getVolume(part[5]).
//                  SET fadeInfo:STEPS TO part[6].
//                  SET fadeInfo:CURRENT_STEP TO 0.
//             }
//             playSingleNote(part[0], part[1], part[2], part[3], fadeInfo).
//         } else if part:LENGTH = 9 {
//             // Four-note chord with dynamics, articulation, and optional fade
//             if part:LENGTH = 10 { // Contains fade information
//                 SET fadeInfo:START_VOLUME TO getVolume(part[7]).
//                 SET fadeInfo:END_VOLUME TO getVolume(part[8]).
//                 SET fadeInfo:STEPS TO part[9].
//                 SET fadeInfo:CURRENT_STEP TO 1.
//             }
//             playChord(LIST(part[0], part[1], part[2], part[3]), part[4], part[5], part[6], fadeInfo).
//         }
//     }
// }

// // Set the BPM and note durations
// setDuration(75).

// // Configure voices for single notes and chords
// SET singleNoteVoice TO GetVoice(5).
// configureVoice(singleNoteVoice, 0.7, "sine", 0.01, 0.3, 0.7, 0.5).

// // List of voices for chords
// SET chordVoices TO LIST().
// for i IN RANGE(1, 5) {
//     LOCAL voice IS GetVoice(i).
//     configureVoice(voice, 0.7, "sine", 0.01, 0.3, 0.7, 0.5).
//     chordVoices:ADD(voice).
// }

// // Define the song as a list of parts
// SET song TO LIST(
//     // setDuration(75),
//     // Single note with dynamics, articulation, and fade info (start volume, end volume, steps)
//     LIST("C4", quarterNote, "mf", "staccato", "mf", "ff", 4),
//     // Four-note chord with dynamics, articulation, and fade info
//     LIST("C3", "F3", "F2", "C4", halfNote, "ff", "legato", "mf", "ff", 4),
//     LIST("D4", eighthNote, "f", "staccato", "f", "mf", 2),
//     // Single note without fade info
//     LIST("E4", sixteenthNote, "mp", "staccato"),
//     // Four-note chord without fade info
//     LIST("F3", "D3", "B3", "D4", halfNote, "ff", "legato")
// ).

// // Play the song
// playSong(song).