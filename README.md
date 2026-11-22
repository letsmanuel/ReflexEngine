# ReflexEngine

An open-source C++20 library for **dynamic, physics-driven character reactions**: hit reactions, balance, stumbling and self-preservation. Characters *perceive -> assess -> decide -> act* every tick, so a shot in the leg produces a limp towards cover, a bullet past the head makes them turn and lean away, a chest wound sends them to the ground curled up, and fatal damage fades control until they collapse. None of it is a canned animation: it emerges from an injury model, a balance controller and swappable behaviours. MIT licensed, no proprietary code.

> Status: **v0.1 work in progress.** The demo draws a primitive humanoid, not a rigged model yet. See `docs/DECISIONS.md` for limitations. (Demo GIF: coming with the first release.)

## What reacts how

| Event | Reaction |
|---|---|
| Shot in a leg | support capacity of that leg drops, weight shifts to the other leg, limp (short fast steps, drag when very weak), moves out of the line of fire / towards cover, crouches |
| Bullet passes the head | turns away and leans away from that side (as far as balance allows), arms up, chin tucked |
| Light torso hit | hunches over the wound, arms protect it, slows down, braces |
| Bullet in the chest | incapacitated: slowly sinks to the ground, curls up protecting head and wound; on the back it struggles but never gets up |
| Arm hit | that arm is cradled across the body, the other arm balances, the wounded side turns away |
| Heavy / fatal damage | control fades gradually, then a limp collapse |

Every decision has a human-readable reason string, e.g. `Right leg support 0.30 -> shift weight to left, limp, move away from fire, crouch for cover`.

## Quick start

```bash
git clone https://github.com/letsmanuel/ReflexEngine.git
cd ReflexEngine
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --config Release
ctest --test-dir build -C Release          # unit + headless scenario tests
./build/demo/reflex_demo                   # demo (build/demo/Release/reflex_demo.exe on Windows)
```
Always build **Release** for the demo; Jolt is much slower in Debug.

Demo controls: **left click** shoot (hit or near miss), **right drag** orbit, wheel zoom, **R** reset, **Tab** debug overlay (per-limb health/capacity/authority, sliders, behaviours + reason, COM and support polygon), **A** toggle game-animation vs procedural control.

## API sample

```cpp
reflex::World world(*backend);
reflex::Character c = world.CreateCharacter(reflex::CharacterDesc::DefaultHumanoid());
c.SetAnimationPose(pose);                       // game's own animation (optional)
c.ApplyHit({.bodyPart = reflex::BodyPart::RightLowerLeg, .point = p, .direction = d, .impulse = 3.2f, .damage = 35.0f});
c.ReportNearMiss({.closestPoint = p2, .direction = d2, .distance = 0.2f});
c.SetGlobalAuthority(0.8f, 0.2f);               // 0 = passive ragdoll, 1 = fully driven
world.Step(dt);
c.GetOutputPose(pose);                          // pose to render
```

## Docs
[Architecture](docs/ARCHITECTURE.md) | [Integration guide](docs/INTEGRATION.md) | [Design decisions & limitations](docs/DECISIONS.md) | [Tuning](docs/TUNING.md) | [Third-party licenses](THIRD_PARTY_LICENSES.md)

## Features
- Injury model: per-part health, pain, functional capacity, leg support capacity, shock, consciousness, control level, incapacitation
- Perception: hits, near misses, threat direction, exposed side, user-supplied cover query
- Balance: COM, support polygon, capture-point stepping, weight shifting, limp gait
- Control authority: global and per-part, blending between game animation, procedural reaction and passive ragdoll, smooth hand-back
- Backend abstraction (`IPhysicsBackend`), Jolt Physics included
- Deterministic fixed-step simulation; about 0.03 ms per character tick on one core (Release)

## License
MIT. Dependencies: Jolt Physics (MIT), raylib (zlib), doctest (MIT).
