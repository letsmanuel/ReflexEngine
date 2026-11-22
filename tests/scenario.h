#pragma once
// Headless scenario helper shared by the scenario tests and the tools/sim debug program.

#include <memory>

#include "reflex/backend/jolt_backend.h"
#include "reflex/world.h"

namespace reflex::test {

struct Scenario {
    std::unique_ptr<IPhysicsBackend> backend;
    std::unique_ptr<World> world;
    Character c;
    float dt = 1.0f / 120.0f;

    explicit Scenario(const CharacterDesc& desc = CharacterDesc::DefaultHumanoid(), bool withGround = true) {
        backend = CreateJoltBackend();
        if (withGround) AddGround();
        world = std::make_unique<World>(*backend);
        c = world->CreateCharacter(desc);
    }

    void AddGround() {
        BodyDesc g;
        g.shape.type = ShapeType::Box;
        g.shape.halfExtents = {60, 0.5f, 60};
        g.isStatic = true;
        g.transform.position = {0, -0.5f, 0};
        g.friction = 1.0f;
        backend->CreateBody(g);
    }

    void Run(float seconds) {
        int n = static_cast<int>(seconds / dt + 0.5f);
        for (int i = 0; i < n; ++i) world->Step(dt);
    }

    Vec3 Pelvis() const { return c.GetBodyPartTransform(BodyPart::Pelvis).position; }
    Vec3 Head() const { return c.GetBodyPartTransform(BodyPart::Head).position; }
    const CharacterDebugInfo& Info() const { return c.GetDebugInfo(); }
    /// Torso tilt from vertical in radians.
    float Tilt() const { return Info().tilt; }
    /// True while the pelvis is more than 55% of its standing height and the torso is not tipped over.
    bool Upright() const { return Pelvis().y > 0.55f * 0.97f && Tilt() < 0.9f; }
    /// Position of the character in the character's own frame relative to where it started.
    Vec3 Com() const { return Info().com; }
};

} // namespace reflex::test
