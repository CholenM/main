#pragma once

// Opt in only after verifying pre-home test poses and the hardwired stop.
#define SCARA_ENABLE_PREHOME_NUDGE 0

// Status-only by default. Copy to machine_config.h for commissioning.
//
// Z is a bounded, unreferenced step axis: pos.z = 0 is the boot datum, not a
// measured height. It has a bottom switch only; there is no upper switch and
// no Z homing phase. Positive API steps must raise the tool. Downward motion
// stops on the bottom switch. Set min/max from measured travel, then verify
// the direction by jogging a few steps with the drill unpowered. Z homeSign is
// unused; leave it at 0.
//
// For J1/J2 measure positiveLevel (CCW viewed from above), homeSign (+/-1),
// travel from the backed-off datum, and safe seek/release/backoff step budgets.
// Only then enable MOTION_COMMISSIONED.
constexpr bool MOTION_COMMISSIONED = false;
constexpr Axis AXES[] = {
  // label step dir switch positiveLevel homeSign min    max   seek release backoff
  {"z",      2,  15, 27,    LOW,           0,    -2000, 2000,   0,    0,      0},
  {"j1",     16, 17, 26,    LOW,           0,        0,    0,   0,    0,      0},
  {"j2",     21, 22, 25,    LOW,           0,        0,    0,   0,    0,      0},
};
