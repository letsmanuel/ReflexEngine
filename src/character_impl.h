#pragma once
// Internal: private state of a Character. Not part of the public API.

#include "reflex/character.h"
#include "reflex/physics_backend.h"

namespace reflex {

struct Character::Impl {
    struct PartData {
        BodyHandle body = kInvalidHandle;
        JointHandle joint = kInvalidHandle;
        ShapeDesc shape;
        float mass = 1.0f;
        float length = 0.0f;
        Vec3 jointBind;      ///< bind position of the joint (model space)
        Vec3 centerBind;     ///< bind position of the body centre (model space)
        Vec3 jointInChild;   ///< joint position in the body's own frame
        Vec3 jointInParent;  ///< joint position in the parent body's frame
        Quat boneBindRot;    ///< bind world rotation of the mapped bone
    };

    struct Leg {
        bool swinging = false;
        float t = 0.0f, duration = 0.3f, lift = 0.07f;
        Vec3 start, target, planted;  ///< ground positions (x, z used) of the ankle
        float stanceTimer = 0.0f;
        bool contact = true;
        float groundY = 0.0f;
        Vec3 ankle;                   ///< current ankle world position
    };

    // identity
    IPhysicsBackend* backend = nullptr;
    std::uint32_t id = 0;
    CharacterDesc desc;
    Transform spawn;
    float scale = 1.0f;
    float totalMass = 75.0f;

    // sub-models
    InjuryModel injury;
    ThreatModel threat;
    AuthorityBlender authority;
    DecisionLayer decisions;

    // animation input
    bool hasAnim = false;
    Pose animPose;
    std::vector<Transform> animWorld;

    // geometry
    PartData parts[kBodyPartCount];
    float thighLen[2] = {0.44f, 0.44f}, shinLen[2] = {0.4f, 0.4f};
    Vec3 hipInPelvis[2];
    float ankleHeight = 0.09f;
    float standPelvisHeight = 0.97f;

    // controller state
    Leg leg[2];
    int lastSwing = -1;
    float stepCooldown = 0.0f;
    float baseYaw = 0.0f, curYaw = 0.0f;
    float time = 0.0f;
    ReactionTargets prevTargets;
    float effLean = 0.0f;
    float sCrouch = 0.0f, sStance = 0.0f, sTurn = 0.0f;
    Vec3 sLean;           ///< smoothed lean vector (direction * angle)
    Vec3 sMoveVel;        ///< smoothed desired horizontal velocity
    float smoothedMoveSpeed = 0.0f;
    Vec3 smoothedMoveDir;
    PerPart<Vec3> smoothedOffset;
    float sLimp = 0.0f;
    CharacterDebugInfo debug;

    void Build(IPhysicsBackend* be, std::uint32_t charId, const CharacterDesc& d);
    void ResetState();
    void Reset();
    void Teleport(const Transform& t);
    void ApplyHit(const HitEvent& hit);
    void ReportNearMiss(const NearMissEvent& miss);
    void SetAnimationPose(const Pose& pose);
    bool RayCastParts(const Vec3& origin, const Vec3& dir, float maxDist, BodyPart& part, Vec3& point,
                      float& dist) const;
    float ClosestApproach(const Vec3& origin, const Vec3& dir, float maxDist, BodyPart& part, Vec3& point) const;
    void GetOutputPose(Pose& out) const;

    /// One fixed control tick: perceive -> assess -> decide -> act (writes joint drives and assist forces).
    void Tick(float dt);
    void Destroy();
};

} // namespace reflex
