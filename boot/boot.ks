CLEARSCREEN.

// Open the terminal for user interaction
CORE:DOEVENT("Open Terminal").

DECLARE FUNCTION PAD_LEFT {
  PARAMETER value.
  PARAMETER width IS 2.

  LOCAL text IS value + "".
  UNTIL text:LENGTH >= width {
    SET text TO "0" + text.
  }.
  RETURN text.
}.

DECLARE FUNCTION FORMAT_SECONDS {
  PARAMETER seconds.

  LOCAL total IS FLOOR(seconds).
  IF total < 0 {
    SET total TO 0.
  }.
  LOCAL hours IS FLOOR(total / 3600).
  LOCAL mins IS FLOOR(MOD(total, 3600) / 60).
  LOCAL secs IS MOD(total, 60).
  RETURN PAD_LEFT(hours, 2) + ":" + PAD_LEFT(mins, 2) + ":" + PAD_LEFT(secs, 2).
}.

DECLARE FUNCTION PROGRESS_BAR {
  PARAMETER current.
  PARAMETER total.
  PARAMETER width IS 24.

  IF total <= 0 {
    RETURN "".
  }.
  LOCAL ratio IS current / total.
  IF ratio < 0 {
    SET ratio TO 0.
  }.
  IF ratio > 1 {
    SET ratio TO 1.
  }.
  LOCAL filledCount IS ROUND(width * ratio).
  IF filledCount > width {
    SET filledCount TO width.
  }.
  LOCAL bar IS "".
  LOCAL idx IS 0.
  UNTIL idx >= width {
    IF idx < filledCount {
      SET bar TO bar + "█".
    } ELSE {
      SET bar TO bar + "░".
    }.
    SET idx TO idx + 1.
  }.
  RETURN bar.
}.

DECLARE FUNCTION PRINT_HEADER {
  PARAMETER hdrShipName.
  PARAMETER hdrBootStart.

  PRINT "╔════════════════════════════════════╗".
  PRINT "║ Airship Flight Computer Boot       ║".
  PRINT "╠════════════════════════════════════╣".
  PRINT "║ Ship      : " + hdrShipName + "║".
  PRINT "║ Processor : " + CORE:VOLUME:NAME + "║".
  PRINT "║ MET       : " + FORMAT_SECONDS(TIME:SECONDS - hdrBootStart) + "║".
  PRINT "╚════════════════════════════════════╝".
}.

DECLARE FUNCTION PRINT_STAGE {
  PARAMETER stageInfo.
  PARAMETER stageNumber.
  PARAMETER stageTotal.
  PARAMETER stageBootStart.

  LOCAL bar IS PROGRESS_BAR(stageNumber, stageTotal, 24).
  PRINT "".
  PRINT "Stage " + PAD_LEFT(stageNumber, 2) + " of " + PAD_LEFT(stageTotal, 2) + ": " + stageInfo["label"].
  PRINT stageInfo["detail"].
  PRINT "Progress [" + bar + "]".
  PRINT "Elapsed  " + FORMAT_SECONDS(TIME:SECONDS - stageBootStart).
}.

DECLARE FUNCTION PRINT_SUMMARY {
  PARAMETER sumShipName.
  PARAMETER sumBootStart.
  PARAMETER sumLaunchPath.

  CLEARSCREEN.
  PRINT "╔════════════════════════════════════╗".
  PRINT "║ Boot Summary                       ║".
  PRINT "╠════════════════════════════════════╣".
  PRINT "║ Ship      : " + sumShipName + "║".
  PRINT "║ Processor : " + CORE:VOLUME:NAME + "║".
  PRINT "║ Runtime   : " + FORMAT_SECONDS(TIME:SECONDS - sumBootStart) + "║".
  IF sumLaunchPath <> "" {
    PRINT "║ Launch KS : " + sumLaunchPath + "║".
  } ELSE {
    PRINT "║ Launch KS : <missing>" + "║".
  }.
  PRINT "╚════════════════════════════════════╝".
}.

LOCAL bootStartTime IS TIME:SECONDS.
LOCAL bootShipName IS SHIP:NAME.

IF NOT SHIP:UNPACKED {
  PRINT "Awaiting vessel unpack...".
  WAIT UNTIL SHIP:UNPACKED.
}.
WAIT 0.2.

SET TERMINAL:WIDTH TO 80.
SET TERMINAL:HEIGHT TO 40.

LOCAL stages IS LIST().
LOCAL stageBlock IS LEXICON().

SET stageBlock TO LEXICON().
SET stageBlock["label"] TO "Powering kOS core".
SET stageBlock["detail"] TO "Spinning up flight computer...".
SET stageBlock["delay"] TO 0.6.
stages:ADD(stageBlock).

SET stageBlock TO LEXICON().
SET stageBlock["label"] TO "Linking navigation libraries".
SET stageBlock["detail"] TO "Loading ag_lib, ui, and crash monitors".
SET stageBlock["delay"] TO 0.6.
stages:ADD(stageBlock).

SET stageBlock TO LEXICON().
SET stageBlock["label"] TO "Enumerating storage volumes".
SET stageBlock["detail"] TO "Processor volume: " + CORE:VOLUME:NAME.
SET stageBlock["delay"] TO 0.5.
stages:ADD(stageBlock).

SET stageBlock TO LEXICON().
SET stageBlock["label"] TO "Synchronizing sensors".
SET stageBlock["detail"] TO "Ship online: " + bootShipName.
SET stageBlock["delay"] TO 0.6.
stages:ADD(stageBlock).

SET stageBlock TO LEXICON().
SET stageBlock["label"] TO "Scanning for Airship OS".
SET stageBlock["detail"] TO "Looking for aos/launch.ks".
SET stageBlock["delay"] TO 0.7.
stages:ADD(stageBlock).

SET stageBlock TO LEXICON().
SET stageBlock["label"] TO "Verifying launch manifest".
SET stageBlock["detail"] TO "Confirming aos/launch.ks".
SET stageBlock["delay"] TO 0.6.
stages:ADD(stageBlock).

SET stageBlock TO LEXICON().
SET stageBlock["label"] TO "Finalizing boot state".
SET stageBlock["detail"] TO "Handing off to mission software".
SET stageBlock["delay"] TO 0.6.
stages:ADD(stageBlock).

LOCAL totalStages IS stages:LENGTH.
LOCAL stageIndex IS 0.

FOR stageEntry IN stages {
  SET stageIndex TO stageIndex + 1.
  CLEARSCREEN.
  PRINT_HEADER(bootShipName, bootStartTime).
  PRINT_STAGE(stageEntry, stageIndex, totalStages, bootStartTime).

  LOCAL waitTime IS 0.5.
  IF stageEntry:HASKEY("delay") {
    SET waitTime TO stageEntry["delay"].
  }.
  WAIT waitTime.
}.

LOCAL bootLaunchPath IS "/aos/launch.ks".

PRINT_SUMMARY(bootShipName, bootStartTime, bootLaunchPath).
WAIT 1.5.

IF EXISTS(bootLaunchPath) {
  PRINT "".
  PRINT "Launching Airship OS from " + bootLaunchPath + ".".
  WAIT 1.
  RUNPATH(bootLaunchPath).
} ELSE {
  PRINT "".
  PRINT "ERROR: launch.ks not found at " + bootLaunchPath + ".".
  PRINT "Verify the Airship Operating System is installed on this vessel.".
}.

