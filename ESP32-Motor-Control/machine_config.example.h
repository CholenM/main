#pragma once

// Status-only by default. Copy to machine_config.h for commissioning.
// Z is mechanically fixed; its home sign and all motion fields must remain zero.
// For J1/J2 measure positiveLevel (CCW viewed from above), homeSign (+/-1),
// travel from the backed-off datum, and safe seek/release/backoff step budgets.
// Only then enable MOTION_COMMISSIONED.
constexpr bool MOTION_COMMISSIONED = false;
constexpr Axis AXES[] = {
  // label step dir switch positiveLevel homeSign min max seek release backoff
  {"z",      2,  15, 27,    LOW,           0,       0,  0,  0,   0,      0},
  {"j1",     16, 17, 26,    LOW,           0,       0,  0,  0,   0,      0},
  {"j2",     21, 22, 25,    LOW,           0,       0,  0,  0,   0,      0},
};
