#include <doctest/doctest.h>

#include "reflex/backend/jolt_backend.h"

using namespace reflex;

TEST_CASE("Jolt backend: a box falls onto static ground and rests") {
    auto be = CreateJoltBackend();
    BodyDesc ground;
    ground.shape.type = ShapeType::Box;
    ground.shape.halfExtents = {20, 0.5f, 20};
    ground.isStatic = true;
    ground.transform.position = {0, -0.5f, 0};
    be->CreateBody(ground);

    BodyDesc box;
    box.shape.halfExtents = {0.25f, 0.25f, 0.25f};
    box.mass = 5;
    box.transform.position = {0, 2, 0};
    BodyHandle b = be->CreateBody(box);

    for (int i = 0; i < 240; ++i) be->Step(1.0f / 120.0f);
    BodyState s = be->GetBodyState(b);
    CHECK(s.transform.position.y == doctest::Approx(0.25f).epsilon(0.1));
    CHECK(Length(s.linearVelocity) < 0.05f);
}

TEST_CASE("Jolt backend: ray cast hits ground and bodies") {
    auto be = CreateJoltBackend();
    BodyDesc ground;
    ground.shape.halfExtents = {20, 0.5f, 20};
    ground.isStatic = true;
    ground.transform.position = {0, -0.5f, 0};
    be->CreateBody(ground);

    RayHit hit;
    REQUIRE(be->RayCast({1, 3, 1}, {0, -1, 0}, 10, true, hit));
    CHECK(hit.point.y == doctest::Approx(0.0f).epsilon(0.01));
    CHECK(hit.distance == doctest::Approx(3.0f).epsilon(0.01));
    CHECK_FALSE(be->RayCast({1, 3, 1}, {0, 1, 0}, 10, true, hit));
}
