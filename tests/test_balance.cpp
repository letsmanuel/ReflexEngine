#include <doctest/doctest.h>

#include "reflex/balance.h"

using namespace reflex;

TEST_CASE("Balance: weighted centre") {
    Vec3 p[2] = {{0, 0, 0}, {3, 0, 0}};
    float m[2] = {1, 2};
    Vec3 c = WeightedCentre(p, m, 2);
    CHECK(c.x == doctest::Approx(2.0f));
}

TEST_CASE("Balance: capture point of a pendulum at rest is under the COM") {
    Vec3 cp = CapturePoint({0.1f, 1.0f, -0.2f}, {0, 0, 0}, 1.0f);
    CHECK(cp.x == doctest::Approx(0.1f));
    CHECK(cp.z == doctest::Approx(-0.2f));
}

TEST_CASE("Balance: capture point leads the COM along its velocity by sqrt(h/g)") {
    float h = 0.98f;
    Vec3 cp = CapturePoint({0, 1.0f, 0}, {1.0f, 0, 0}, h, 9.8f);
    CHECK(cp.x == doctest::Approx(std::sqrt(h / 9.8f)).epsilon(1e-3));
}

TEST_CASE("Balance: support polygon of two feet") {
    // Two feet, each a 0.1 x 0.25 rectangle, 0.3 apart.
    Vec3 pts[8];
    int n = 0;
    for (float cx : {-0.15f, 0.15f})
        for (float dx : {-0.05f, 0.05f})
            for (float dz : {-0.125f, 0.125f}) pts[n++] = {cx + dx, 0, dz};
    SupportPolygon poly = SupportPolygon::FromPoints(pts, n);
    CHECK(poly.count == 4);
    CHECK(poly.Contains({0, 0, 0}));
    CHECK(poly.SignedDistance({0, 0, 0}) < 0.0f);
    CHECK_FALSE(poly.Contains({0.5f, 0, 0}));
    CHECK(poly.SignedDistance({0.5f, 0, 0}) == doctest::Approx(0.3f).epsilon(1e-3));
    Vec3 closest = poly.ClosestPoint({0.5f, 0, 0});
    CHECK(closest.x == doctest::Approx(0.2f));
}

TEST_CASE("Balance: degenerate support polygons have no inside") {
    Vec3 one[1] = {{0, 0, 0}};
    SupportPolygon p1 = SupportPolygon::FromPoints(one, 1);
    CHECK(p1.count == 1);
    CHECK_FALSE(p1.Contains({0, 0, 0}));
    CHECK(p1.SignedDistance({0, 0, 1}) == doctest::Approx(1.0f));
    Vec3 line[2] = {{0, 0, 0}, {1, 0, 0}};
    SupportPolygon p2 = SupportPolygon::FromPoints(line, 2);
    CHECK_FALSE(p2.Contains({0.5f, 0, 0}));
    CHECK(p2.SignedDistance({0.5f, 0, 1}) == doctest::Approx(1.0f));
    SupportPolygon none = SupportPolygon::FromPoints(nullptr, 0);
    CHECK(none.count == 0);
}

TEST_CASE("Balance: leg loads follow support capacity") {
    LegLoads l = ComputeLegLoads(1.0f, 0.15f);
    CHECK(l.left > 0.8f);
    CHECK(l.right < 0.2f);
    CHECK(l.left + l.right == doctest::Approx(1.0f));
    CHECK(l.deficit == doctest::Approx(0.0f));

    LegLoads sym = ComputeLegLoads(1.0f, 1.0f);
    CHECK(sym.left == doctest::Approx(0.5f));
}

TEST_CASE("Balance: the injured leg is never loaded above what it can bear, even with a bias") {
    LegLoads l = ComputeLegLoads(1.0f, 0.15f, -1.0f); // bias asks for more weight on the right
    CHECK(l.right <= 0.15f + 0.85f * 0.15f + 1e-4f);
}

TEST_CASE("Balance: two weak legs raise a support deficit") {
    LegLoads l = ComputeLegLoads(0.3f, 0.3f);
    CHECK(l.deficit > 0.3f);
    LegLoads none = ComputeLegLoads(0.0f, 0.0f);
    CHECK(none.deficit == doctest::Approx(1.0f));
}

TEST_CASE("Balance: step target is clamped to the reachable radius") {
    Vec3 stance{0, 0, 0};
    Vec3 target = ComputeStepTarget({2.0f, 0, 0}, stance, {1, 0, 0}, 0.6f, 0.05f);
    CHECK(Length(target - stance) == doctest::Approx(0.6f).epsilon(1e-3));
    Vec3 near = ComputeStepTarget({0.2f, 0, 0}, stance, {1, 0, 0}, 0.6f, 0.05f);
    CHECK(near.x == doctest::Approx(0.25f));
}

TEST_CASE("Balance: gait is asymmetric for an injured leg") {
    GaitParams g = ComputeGait(1.0f, 0.15f);
    const int L = SideIndex(Side::Left), R = SideIndex(Side::Right);
    CHECK(g.stepLengthScale[R] < g.stepLengthScale[L]);
    CHECK(g.swingTimeScale[R] < g.swingTimeScale[L]); // shorter, faster steps on the injured side
    CHECK(g.stanceTimeScale[R] < g.stanceTimeScale[L]);
    CHECK(g.drag[R]);
    CHECK_FALSE(g.drag[L]);
    CHECK(g.speedScale < 0.9f);

    GaitParams healthy = ComputeGait(1.0f, 1.0f);
    CHECK(healthy.speedScale == doctest::Approx(1.0f));
    CHECK_FALSE(healthy.drag[L]);
    CHECK(healthy.stepLengthScale[L] == doctest::Approx(healthy.stepLengthScale[R]));
}
