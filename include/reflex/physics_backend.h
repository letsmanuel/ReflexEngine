#pragma once
/// @file physics_backend.h
/// Abstraction over the rigid-body physics engine. Implement IPhysicsBackend to plug in your own
/// engine (PhysX, Bullet, ...). A Jolt implementation ships with the library (see backend/jolt_backend.h).
///
/// Conventions: right-handed, Y up, metres/kilograms/seconds. Bodies are positioned at their centre
/// of mass (their shape is centred on the body origin). Capsules are aligned with the body's local Y axis.

#include <cstdint>

#include "reflex/math.h"

namespace reflex {

using BodyHandle = std::uint32_t;
using JointHandle = std::uint32_t;
constexpr std::uint32_t kInvalidHandle = 0xFFFFFFFFu;

enum class ShapeType : std::uint8_t { Box, Sphere, Capsule };

struct ShapeDesc {
    ShapeType type = ShapeType::Box;
    Vec3 halfExtents{0.5f, 0.5f, 0.5f}; ///< Box
    float radius = 0.1f;                ///< Sphere, Capsule
    float halfHeight = 0.2f;            ///< Capsule: half the length of the cylindrical part
};

struct BodyDesc {
    ShapeDesc shape;
    float mass = 1.0f;
    Transform transform;
    bool isStatic = false;
    float friction = 0.9f;
    float restitution = 0.0f;
    float linearDamping = 0.02f;
    float angularDamping = 0.1f;
    /// Bodies sharing a non-zero collisionGroup may have pairwise collisions disabled
    /// via SetCollisionPairEnabled (used to stop adjacent ragdoll parts from fighting).
    std::uint32_t collisionGroup = 0;
    std::uint32_t subGroup = 0;   ///< unique index inside the group, < 32
    std::uint64_t userData = 0;   ///< returned from ray casts
};

enum class JointType : std::uint8_t {
    Hinge, ///< one degree of freedom around `axis`
    Ball   ///< swing-twist: cone swing limits plus twist limits around `axis`
};

struct JointDesc {
    JointType type = JointType::Ball;
    BodyHandle parent = kInvalidHandle;
    BodyHandle child = kInvalidHandle;
    Vec3 anchor;                   ///< world-space joint position
    Vec3 axis{0, 1, 0};            ///< world-space hinge axis / twist axis
    Vec3 normalAxis{1, 0, 0};      ///< world-space axis perpendicular to `axis` (Ball: swing plane reference)
    float minAngle = -0.5f;        ///< Hinge: min angle (rad). Ball: min twist.
    float maxAngle = 0.5f;         ///< Hinge: max angle (rad). Ball: max twist.
    float swingNormal = 0.6f;      ///< Ball: half cone angle around the normal axis (rad)
    float swingPlane = 0.6f;       ///< Ball: half cone angle around the plane axis (rad)
};

/// Motor ("PD") drive of a joint. `target` is the desired rotation of the child relative to the
/// parent expressed as a DELTA from the pose the joint had at creation (identity = bind pose),
/// given in the parent body's frame. Hinges only use the component around their axis.
struct JointDrive {
    Quat target;
    float stiffness = 0.0f;  ///< N*m/rad. 0 disables the drive (passive joint).
    float damping = 0.0f;    ///< N*m*s/rad
    float maxTorque = 0.0f;  ///< N*m cap
};

struct BodyState {
    Transform transform;
    Vec3 linearVelocity;
    Vec3 angularVelocity;
};

struct RayHit {
    BodyHandle body = kInvalidHandle;
    std::uint64_t userData = 0;
    Vec3 point;
    Vec3 normal{0, 1, 0};
    float distance = 0.0f;
};

class IPhysicsBackend {
public:
    virtual ~IPhysicsBackend() = default;

    virtual void SetGravity(const Vec3& g) = 0;

    virtual BodyHandle CreateBody(const BodyDesc& desc) = 0;
    virtual void DestroyBody(BodyHandle body) = 0;
    virtual JointHandle CreateJoint(const JointDesc& desc) = 0;
    virtual void DestroyJoint(JointHandle joint) = 0;
    virtual void SetCollisionPairEnabled(BodyHandle a, BodyHandle b, bool enabled) = 0;

    virtual void SetJointDrive(JointHandle joint, const JointDrive& drive) = 0;

    virtual BodyState GetBodyState(BodyHandle body) const = 0;
    /// Teleport a body and set its velocities (used for resets).
    virtual void SetBodyState(BodyHandle body, const BodyState& state) = 0;
    virtual void AddForce(BodyHandle body, const Vec3& force) = 0;
    virtual void AddTorque(BodyHandle body, const Vec3& torque) = 0;
    /// Instant impulse at a world point.
    virtual void AddImpulse(BodyHandle body, const Vec3& impulse, const Vec3& worldPoint) = 0;

    /// Nearest hit along a ray. `staticOnly` restricts the test to static scenery (ground, walls).
    virtual bool RayCast(const Vec3& origin, const Vec3& direction, float maxDistance, bool staticOnly,
                         RayHit& outHit) const = 0;

    /// Advance the simulation by dt seconds (fixed timestep recommended).
    virtual void Step(float dt) = 0;
};

} // namespace reflex
