#pragma once
/// @file world.h
/// Owns characters, steps the simulation with a fixed time step.

#include <memory>
#include <vector>

#include "reflex/character.h"
#include "reflex/export.h"
#include "reflex/physics_backend.h"

namespace reflex {

/// Result of firing a bullet through the world with World::TraceBullet.
struct BulletResult {
    enum class Type { Miss, HitScenery, HitCharacter, NearMiss } type = Type::Miss;
    Vec3 point;            ///< impact point (or closest approach for a near miss)
    float distance = 0.0f;
    BodyPart part = BodyPart::Chest;
    Character character;   ///< character that was hit / nearly hit
};

struct BulletParams {
    float speed = 400.0f;  ///< m/s
    float mass = 0.008f;   ///< kg
    float damage = 35.0f;  ///< health points on a hit
    /// Damage multiplier per body part group (head shots hurt more).
    float headMultiplier = 2.5f;
    float limbMultiplier = 1.0f;
    float torsoMultiplier = 1.0f;
    float nearMissRadius = 0.9f; ///< a pass within this distance of a body part counts as a near miss
};

class REFLEX_API World {
public:
    /// Use a backend you own (it must outlive the world).
    explicit World(IPhysicsBackend& backend);
    /// Take ownership of the backend.
    explicit World(std::unique_ptr<IPhysicsBackend> backend);
    ~World();
    World(const World&) = delete;
    World& operator=(const World&) = delete;

    Character CreateCharacter(const CharacterDesc& desc);
    void DestroyCharacter(Character& c);
    int CharacterCount() const;
    Character GetCharacter(int index) const;

    /// Advance by `dt` seconds. Internally uses a fixed step (see SetFixedTimeStep) so results are
    /// deterministic regardless of the frame rate.
    void Step(float dt);
    void SetFixedTimeStep(float seconds) { fixedStep_ = seconds; }
    float FixedTimeStep() const { return fixedStep_; }
    double Time() const { return time_; }

    /// Fire a bullet: finds the first thing it hits, applies a HitEvent to a struck character or reports
    /// a NearMissEvent to every character it passed close to.
    BulletResult TraceBullet(const Vec3& origin, const Vec3& direction, float maxDistance,
                             const BulletParams& params = {});

    IPhysicsBackend& Backend() { return *backend_; }

private:
    struct Impl;
    std::unique_ptr<IPhysicsBackend> owned_;
    IPhysicsBackend* backend_;
    std::vector<std::shared_ptr<Character::Impl>> characters_;
    float fixedStep_ = 1.0f / 120.0f;
    float accumulator_ = 0.0f;
    double time_ = 0.0;
    std::uint32_t nextId_ = 1;
};

} // namespace reflex
