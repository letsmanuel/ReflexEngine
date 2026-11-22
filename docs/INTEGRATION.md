# Integration guide

## 1. Add the library
```cmake
add_subdirectory(ReflexEngine)           # or FetchContent from github.com/letsmanuel/ReflexEngine
target_link_libraries(my_game PRIVATE reflex::reflex)
```
Options: `REFLEX_BUILD_SHARED` (DLL / .so), `REFLEX_BUILD_TESTS`, `REFLEX_BUILD_DEMO`.

## 2. Create a world and a character
```cpp
#include "reflex/backend/jolt_backend.h"
#include "reflex/world.h"

auto backend = reflex::CreateJoltBackend();
// create your static scenery through the backend (ground, walls):
reflex::BodyDesc ground; ground.isStatic = true;
ground.shape.halfExtents = {50, 0.5f, 50}; ground.transform.position = {0, -0.5f, 0};
backend->CreateBody(ground);

reflex::World world(*backend);
reflex::CharacterDesc desc = reflex::CharacterDesc::DefaultHumanoid(1.75f, 75.0f);
reflex::Character npc = world.CreateCharacter(desc);
```
To use your own rig fill `desc.skeleton` (bones with bind transforms, parents first) and `desc.mapping` (bone index for each of the 17 `BodyPart`s; a mapped bone's origin is the joint where the part attaches to its parent).

## 3. Per frame
```cpp
npc.SetAnimationPose(pose);               // optional: your authored animation (bone 0 local = world)
world.Step(dt);                           // fixed 1/120 s ticks inside
npc.GetOutputPose(outPose);               // local transform for every bone: render this
```

## 4. Events
```cpp
npc.ApplyHit({.bodyPart = reflex::BodyPart::RightLowerLeg, .point = p, .direction = d,
              .impulse = 3.2f, .damage = 35.0f});
npc.ReportNearMiss({.closestPoint = c, .direction = d, .distance = 0.2f});
// or let the world find out:
reflex::BulletResult r = world.TraceBullet(origin, dir, 100.0f, reflex::BulletParams{});
```

## 5. Authority and animation blending
```cpp
npc.SetGlobalAuthority(0.8f, 0.2f);                  // value, blend seconds
npc.SetAuthority(reflex::BodyPart::LeftUpperArm, 0.3f, 0.1f);
npc.SetAnimationWeight(1.0f, 0.2f);                  // follow your animation
npc.ReleaseToAnimation(0.5f);                        // hand control back after a reaction
npc.TakeProceduralControl(0.2f);
```
Authority 0 = passive ragdoll. Injuries lower authority automatically.

## 6. Cover, debug, custom behaviours
```cpp
npc.Threat().SetCoverQuery([&](const reflex::Vec3& from, const reflex::Vec3& towardShooter, reflex::Vec3& out) { ... });
const auto& info = npc.GetDebugInfo();   // info.reason, info.activeBehaviors, COM, support polygon, per-part state
npc.Decisions().AddBehavior(std::make_unique<MyBehavior>());  // derive from reflex::Behavior
```

## 7. Your own physics engine
Implement `reflex::IPhysicsBackend` (bodies, joints with position motors, forces, ray casts, `Step`) and pass it to `World`.

## Notes
- Units: metres, kilograms, seconds, Y up. Character local frame: +Z forward, +X left.
- Reset: `npc.Reset()` or `npc.Teleport(transform)`.
- Build Release for real-time use; Debug builds of Jolt are several times slower.
