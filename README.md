# StarBlimp-KOS_Script

For those who do not know, KOS (Kerbal Operation System) is built on KS (Kerbal Script) and designed to automate any function in Kerbal Space Program. It's similar to BASIC and comes with an in-game terminal and even Telenet support.

## main/launch.ks

KOS script that automates the entire launch procedure, from pre-launch preparations to entering the atmosphere of another planet.

This 2500-line script is packed with functionality, including all aspects of flight control, steering, life support management, power systems, communications, and warp control for the entire journey. It handles complex orbital mechanics and rocketry calculations, making the entire process fully automated and self-correcting. It also provides a clean terminal interface for real-time monitoring (and a cool factor). No manual input is required for the entire journey.

The included craft is extremely heavy and complex, requiring perfect handling, especially during takeoff. Minor mistakes can lead to total loss of the vehicle and all crew (16-74 personnel). The ship features four stages just to get airborne and an additional seven stages to achieve orbit. This script manages over 70 tasks for the various systems required to sustain a 200+ day journey to Eve with realism mods like Kerbalism and FAR.

Most importantly, this script flies more efficiently than any flight computer mod or manual piloting. After testing, this script gets more Delta-V every time. I also crashed a lot, while this never forgets takeoff flaps and crashes into the sea.

In my efforts to maximize performance, I created a script to record a detailed launch profile, which can be plotted in Python for data comparison and testing.

### music/musicPlayer.ks

A "midi player" I was working on. You transpose a song (or each part) to a list of notes, and this will play it with basic generated tones. It can even play chords and multiple parts. It can be upgraded to include features like "attack" for Staccato and volume controls that support crescendoing.
The only MIDI player for KOS!

### /test

A collection of scripts I found or wrote/modified to test before adding to the big script.
