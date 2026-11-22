# Design decisions

## Physics backend: Jolt Physics
Chosen over Bullet because it is MIT licensed, actively maintained, has swing-twist and hinge constraints with **position motors driven by stiffness/damping springs** (implicit, so stiff PD drives stay stable at 120 Hz) and a deterministic single-threaded mode. It is hidden behind `IPhysicsBackend`; only `src/backend/jolt_backend.cpp` includes Jolt. Built with portable SSE2 only and cross-platform determinism on.

## Balance: virtual forces plus stepping, not pure torque control
Fully torque-driven ragdoll balance is a research problem. ReflexEngine uses a standard hybrid:
- **Joint PD drives** (physics motors) for posture; leg targets come from two-bone IK.
- **Virtual model control assist**: a spring-damper force on the pelvis for height and COM position with equal and opposite reaction forces on the supporting feet (net force zero; the ground provides the external force), plus an upright/heading torque on the pelvis. The pelvis torque is an external "cheat" and is documented as such in `TUNING.md`.
- **Capture-point stepping** (linear inverted pendulum) places recovery and walking steps. The commanded velocity is reduced as the capture point leaves the support polygon so the body never runs ahead of its feet.
All assist is scaled by the control level, so as consciousness drops, or an incapacitating wound fades it out, the character goes to the ground physically.

## Incapacitation instead of "stand or die"
Torso damage >= 25 HP (a bullet in the chest) without death incapacitates: the assist fades over ~2 s (slow sink), limb muscles are weakened, the character curls up protectively (crawl pose with limb cycling) or, on its back, struggles with a torso that is too weak to sit up. There is no recovery path; `BreakFall` is disabled for incapacitated characters.

## Hits are exaggerated
Real bullets carry almost no momentum. `CharacterDesc::hitImpulseScale` (default 12) and `maxHitVelocityChange` make impacts visible; set the scale to 1 for realism.

## Reaction = utility selection per channel
Behaviours score themselves; the best one per channel (locomotion, recovery, upper body, posture, fall) wins, with hysteresis. Outputs are targets only. A dominant fall behaviour suppresses controlled channels. Reason strings are assembled from the winners' clauses.

## Fixed timestep
`World::Step` accumulates time and runs fixed 1/120 s ticks. Identical inputs give bit-identical results on one machine (tested).

## Body model
17 parts. Bodies are axis-aligned at the bind pose so a joint's drive target is the relative rotation delta.

## Known limitations
- Verified on MSVC and GCC (MinGW) locally; see the Actions tab for Linux/macOS CI.
- The demo draws a primitive humanoid, not a rigged model.
- Crawling is a pose with limb motion, not locomotion. The on-the-back struggle is covered by unit tests, but physically lying on the back was not reproduced in the headless simulator.
- Tuned for flat ground only.
