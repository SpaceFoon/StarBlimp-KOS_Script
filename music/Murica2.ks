// The America song from Team America
FUNCTION SETDURATION {
    parameter BPM.
        SET wholeNote TO 4 * (60 / BPM).
        SET halfNote TO 2 * (60 / BPM).
        SET quarternote TO 1 * (60 / BPM).
        SET eigththNote TO .5 * (60 / BPM).
        SET sixteenthNote TO .25 * (60 / BPM).
        SET thirtysecondNote TO .125 * (60 / BPM).
}

SET singleNoteVoice TO GetVoice(0).
SET singleNoteVoice:VOLUME TO 0.7.
SET singleNoteVoice:WAVE TO "sine".
SET singleNoteVoice:ATTACK TO 0.005.
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
).