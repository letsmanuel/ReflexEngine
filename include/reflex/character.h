#pragma once
/// @file character.h
/// A physics-driven character that thinks and reacts like a living body.
///
/// Typical use:
/// @code
/// reflex::World world(backend);
/// reflex::Character c = world.CreateCharacter(desc);
/// c.SetAnimationPose(pose);                    // optional: the game's own animation
/// c.ApplyHit({.bodyPart = BodyPart::RightLowerLeg, .point = p, .direction = d, .impulse = 3.0f, .damage = 35.0f});
/// c.ReportNearMiss({.closestPoint = p2, .direction = d2, .distance = 0.2f});
/// c.SetGlobalAuthority(0.8f, 0.2f);
/// world.Step(dt);
/// c.GetOutputPose(pose);                       // pose to render
/// @endcode

#include <memory>
#include <string>
#include <vector>

#include "reflex/authority.h"
#include "reflex/balance.h"
#include "reflex/behavior.h"
#include "reflex/events.h"
#include "reflex/export.h"
#include "reflex/injury.h"
#include "reflex/math.h"
#include "reflex/physics_backend.h"
#include "reflex/skeleton.h"
#include "reflex/threat.h"
#include "reflex/types.h"

namespace reflex {

struct CharacterDesc {
    SkeletonDesc skeleton;
    BodyMapping mapping;
    HumanoidParams params;
    InjuryConfig injury;
    /// Spawn transform. `position` is the point on the ground below the character, `rotation` its heading
    /// (the character's local +Z is forward, +X is its left).
    Transform spawn;
    /// Hits push the body harder than real bullets would, so reactions are visible. 1 = physically accurate.
    float hitImpulseScale = 12.0f;
    /// Cap on the velocity change a single hit may cause on the struck body part (m/s).
    float maxHitVelocityChange = 2.5f;

    /// A complete, ready-to-use humanoid (skeleton, mapping, proportions).
    static CharacterDesc DefaultHumanoid(float height = 1.75f, float mass = 75.0f);
};

/// Snapshot of everything the debug UI wants to show.
struct CharacterDebugInfo {
    PerPart<PartState> parts;             ///< health / pain / capacity per body part
    PerPart<float> authority;             ///< effective authority per joint
    float supportCapacity[2] = {1, 1};    ///< per leg (SideIndex)
    float legLoad[2] = {0.5f, 0.5f};      ///< fraction of body weight carried by each leg
    float armCapacity[2] = {1, 1};
    float shock = 0, consciousness = 1, control = 1, vitality = 1, fatigue = 0;
    Vec3 com, comVelocity, capturePoint;
    SupportPolygon supportPolygon;
    bool footContact[2] = {true, true};
    bool stepping[2] = {false, false};
    Vec3 footTarget[2];
    float tilt = 0;
    float supportMargin = 0;
    bool fallen = false;
    std::vector<ActiveBehavior> activeBehaviors;
    std::string reason;                   ///< human-readable reasoning of the current decision
    ReactionTargets targets;
};

class REFLEX_API Character {
public:
    Character() = default;
    bool IsValid() const;

    // ---- Inputs ---------------------------------------------------------------------------------------
    /// The game's own animation pose (optional). Used when the animation weight is > 0.
    void SetAnimationPose(const Pose& pose);
    /// A projectile struck the character. Applies an impulse, damage and updates the threat picture.
    void ApplyHit(const HitEvent& hit);
    /// A large non-penetrating blow (explosion, thrown object, vehicle): `impulse` in N*s is shared by the whole body
    /// (centre of mass velocity change = impulse / mass) with extra spin where it lands. Adds shock but no wound.
    void ApplyImpact(const Vec3& impulse, const Vec3& worldPoint);
    /// A projectile passed close to the character without hitting.
    void ReportNearMiss(const NearMissEvent& miss);

    // ---- Control authority ----------------------------------------------------------------------------
    /// 0 = passive ragdoll, 1 = fully driven. Blends over `blendTime` seconds.
    void SetGlobalAuthority(float value, float blendTime = 0.0f);
    void SetAuthority(BodyPart part, float value, float blendTime = 0.0f);
    /// Blend towards the game's animation (1) or the procedural reaction (0).
    void SetAnimationWeight(float value, float blendTime = 0.0f);
    /// Smoothly hand control back to animation after a reaction.
    void ReleaseToAnimation(float blendTime = 0.5f);
    void TakeProceduralControl(float blendTime = 0.2f);
    float GetGlobalAuthority() const;
    float GetAuthority(BodyPart part) const;
    float GetAnimationWeight() const;

    // ---- Outputs --------------------------------------------------------------------------------------
    /// Pose to render: local transform for every bone of the character's skeleton.
    void GetOutputPose(Pose& out) const;
    /// World transform of a body part's physics body.
    Transform GetBodyPartTransform(BodyPart part) const;
    const CharacterDebugInfo& GetDebugInfo() const;
    /// Collision shape of a body part (for drawing the ragdoll or hit volumes).
    ShapeDesc GetPartShape(BodyPart part) const;

    // ---- Utilities ------------------------------------------------------------------------------------
    /// Restore full health, authority and pose, and stand the character up at its spawn transform.
    void Reset();
    /// Teleport the standing character to a new ground position / heading.
    void Teleport(const Transform& groundTransform);
    /// Which body part does this ray hit? (casts against the character's hit volumes only)
    bool RayCastParts(const Vec3& origin, const Vec3& direction, float maxDistance, BodyPart& outPart,
                      Vec3& outPoint, float& outDistance) const;
    /// Closest approach of a ray segment to the character's body parts.
    float ClosestApproach(const Vec3& origin, const Vec3& direction, float maxDistance, BodyPart& outPart,
                          Vec3& outPoint) const;

    InjuryModel& Injury();
    const InjuryModel& Injury() const;
    ThreatModel& Threat();
    DecisionLayer& Decisions();
    AuthorityBlender& Authority();
    const CharacterDesc& Desc() const;

    struct Impl;
    explicit Character(std::shared_ptr<Impl> impl) : impl_(std::move(impl)) {}
    std::shared_ptr<Impl> impl_;
};

} // namespace reflex
