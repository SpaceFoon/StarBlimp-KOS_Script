function antisun {
    // Calculate vector towards the Sun
    set sunVector to body("Sun"):position - ship:position.
    set awayFromSun to sunVector:normalized.

    // Lock steering to point the ship away from the Sun
    lock steering to lookdirup(-awayFromSun, ship:up:vector).
wait 100.
}
antisun().