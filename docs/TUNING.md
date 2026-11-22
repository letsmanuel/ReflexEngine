# Tuning guide

Most knobs live in `src/character_tables.h` (joint limits, drive strengths, mass split), `include/reflex/injury.h` (`InjuryConfig`) and the constants at the top of `src/character_control.cpp`.

| Want | Change |
|---|---|
| Hits shove harder / softer | `CharacterDesc::hitImpulseScale`, `maxHitVelocityChange` |
| Tougher / more fragile body | `InjuryConfig::maxHealth`, `vitalWeight`, `vitalityPerDamage` |
| What incapacitates | `InjuryConfig::incapacitatingTorsoDamage` (HP of torso damage, default 25) |
| Leg hit severity | `painPerDamage`, capacity formula in `InjuryModel::RecomputeCapacity` |
| Shock / faster collapse | `shockPerDamage`, `consciousnessFallRate` |
| Stronger / weaker muscles | `HumanoidParams::strength`, `DriveFor()` in `character_tables.h` |
| Stiffer or floppier joints | scale `k`/`d` in `DriveFor()`; use `SetAuthority` at runtime |
| More / less balance help | gains of the height spring (`2600`, `420`), horizontal accel gains and the pelvis torque (`900`, `110`) in `character_control.cpp`; `SetGlobalAuthority` scales all of it |
| Step eagerness | `stepThreshold` (margin at which a step starts), step duration `0.30 s`, cooldown |
| Walking / limp speed | `kWalkSpeed`, `ComputeGait()` in `balance.h` |
| How fast an incapacitated body sinks | sink time (2 s) in the `Incapacitated` behaviour; leg/arm weakness factors there |
| How reactions look | offsets in the behaviours in `behavior.cpp` (rotation vectors in the parent frame; arms hang down at rest, raising an arm forward is a negative X rotation) |
| Faster simulation | `JoltBackendOptions::velocitySteps/positionSteps`; build Release |

## Honest notes
- The pelvis upright torque and the height/horizontal virtual forces are not real muscle forces. They are the reason a ragdoll can stand and are scaled by the control level, so they vanish as the character loses consciousness or is incapacitated.
- Use `reflex_sim` (`tools/sim.cpp`) to iterate: `reflex_sim torso 6 0 -1` shoots the chest from +Z and prints a time series (pelvis, COM, support margin, chest bend, leg load).
- Run the scenario tests after each change; they encode the behaviours in `Agents.md` (leg hit, head near miss, torso hit, arm hit, fatal, incapacitated).
