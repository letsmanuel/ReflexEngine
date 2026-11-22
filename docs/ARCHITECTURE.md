# Architecture

Each tick a character runs **perceive -> assess -> decide -> act** (`src/character_control.cpp`).

```
 events (ApplyHit / ReportNearMiss)
        |
        v
 InjuryModel --------+          ThreatModel (fire direction, side, cover)
 (health, pain,      |                   |
  capacity, shock,   v                   v
  control, incap.)  BodyStateSummary + ThreatPicture
                            |
                            v
              DecisionLayer (behaviours) --> ReactionTargets + reason string
                            |
                            v
   balance controller: stepping, leg loads, IK, assist forces, ground pose
                            |
                            v
   AuthorityBlender -> JointDrive per joint -> IPhysicsBackend (Jolt)
```

| Module | Header | Role |
|---|---|---|
| math / types | `math.h`, `types.h` | Vec3/Quat; 17-part humanoid. Local frame: +Z forward, +X left, +Y up |
| Skeleton | `skeleton.h` | bones, bone->part mapping, poses, default rig |
| Physics layer | `physics_backend.h`, `backend/jolt_backend.h` | bodies, joints, motor drives, ray casts |
| Injury | `injury.h` | per-part health/pain/capacity, leg support, shock, consciousness, control, incapacitation |
| Threat | `threat.h` | fire direction, recency, exposed side, near-miss side, cover |
| Balance math | `balance.h` | COM, support polygon, capture point, leg loads, gait asymmetry |
| Behaviours | `behavior.h` | Brace, StumbleRecover, LimpAway, SeekCover, TurnAndLeanAway, ProtectHead, ProtectWound, BreakFall, Collapse, Incapacitated |
| Authority | `authority.h` | global / per-part authority, blending, animation hand-back |
| Character / World | `character.h`, `world.h` | ties it together, fixed-step simulation, bullet tracing |

## Control authority
`effective = global * perPart * injuryControl * injuryCapacity(part) * behaviourScale`. Stiffness, damping and torque limit scale with it; below 0.02 the motor is off (passive ragdoll). Joint targets are `Slerp(procedural, animation, animationWeight)`.

## Behaviour channels
Locomotion, Recovery, UpperBody, Posture, Fall. One winner per channel. A Fall winner scoring >= 0.6 (Collapse, Incapacitated) suppresses Locomotion, Recovery and Posture.
