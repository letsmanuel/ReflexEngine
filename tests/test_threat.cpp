#include <doctest/doctest.h>

#include "reflex/threat.h"

using namespace reflex;

static Transform Origin() { return Transform{{0, 1, 0}, Quat::Identity()}; } // facing +Z, left = +X

TEST_CASE("Threat: nothing registered means no threat") {
    ThreatModel t;
    ThreatPicture p = t.Evaluate(Origin(), {0, 1.7f, 0});
    CHECK_FALSE(p.active);
}

TEST_CASE("Threat: a frontal hit gives the shooter position in front of the character") {
    ThreatModel t;
    HitEvent h;
    h.bodyPart = BodyPart::Chest;
    h.direction = {0, 0, -1}; // bullet travels towards -Z, i.e. towards the character facing +Z
    t.RegisterHit(h);
    ThreatPicture p = t.Evaluate(Origin(), {0, 1.7f, 0});
    REQUIRE(p.active);
    CHECK(p.frontExposure > 0.9f);
    CHECK(std::abs(p.exposedSide) < 0.1f);
    CHECK(p.towardSource.z == doctest::Approx(1.0f));
    CHECK(p.recency > 0.95f);
}

TEST_CASE("Threat: shot from the character's left reports the left side as exposed") {
    ThreatModel t;
    HitEvent h;
    h.direction = {-1, 0, 0}; // travelling toward -X, shooter on +X = character's left
    t.RegisterHit(h);
    ThreatPicture p = t.Evaluate(Origin(), {0, 1.7f, 0});
    CHECK(p.exposedSide > 0.9f);
}

TEST_CASE("Threat: heading rotates the local frame") {
    ThreatModel t;
    HitEvent h;
    h.direction = {-1, 0, 0};
    t.RegisterHit(h);
    // Character turned 90 degrees to face +X: shooter on +X is now in front.
    Transform f{{0, 1, 0}, Quat::FromAxisAngle({0, 1, 0}, kPi * 0.5f)};
    ThreatPicture p = t.Evaluate(f, {0, 1.7f, 0});
    CHECK(p.frontExposure > 0.9f);
}

TEST_CASE("Threat: recency decays over time and the threat eventually expires") {
    ThreatModel t;
    HitEvent h;
    h.direction = {0, 0, -1};
    t.RegisterHit(h);
    t.Update(3.0f);
    float mid = t.Evaluate(Origin(), {0, 1.7f, 0}).recency;
    CHECK(mid < 0.8f);
    CHECK(mid > 0.2f);
    t.Update(30.0f);
    CHECK_FALSE(t.Evaluate(Origin(), {0, 1.7f, 0}).active);
}

TEST_CASE("Threat: near miss on the left of the head is reported on the left, with proximity") {
    ThreatModel t;
    NearMissEvent m;
    m.closestPoint = {0.2f, 1.7f, 0.3f};
    m.direction = {0, 0, -1};
    m.distance = 0.15f;
    m.nearestPart = BodyPart::Head;
    t.RegisterNearMiss(m);
    ThreatPicture p = t.Evaluate(Origin(), {0, 1.7f, 0});
    REQUIRE(p.nearMissActive);
    CHECK(p.nearMissSide == Side::Left);
    CHECK(p.nearMissProximity > 0.7f);
    CHECK(p.nearMissHead);
    CHECK(p.confidence < 1.0f); // weaker evidence than a hit
}

TEST_CASE("Threat: near miss on the right side") {
    ThreatModel t;
    NearMissEvent m;
    m.closestPoint = {-0.25f, 1.7f, 0.3f};
    m.direction = {0, 0, -1};
    m.distance = 0.25f;
    t.RegisterNearMiss(m);
    CHECK(t.Evaluate(Origin(), {0, 1.7f, 0}).nearMissSide == Side::Right);
}

TEST_CASE("Threat: cover points shielding from the fire direction are found") {
    ThreatModel t;
    t.AddCoverPoint({{3, 0, 4}, {0, 0, 1}});   // shields against fire from +Z (behind it is -Z side)
    t.AddCoverPoint({{-9, 0, 0}, {0, 0, -1}}); // faces the wrong way
    HitEvent h;
    h.direction = {0, 0, -1}; // shooter at +Z
    t.RegisterHit(h);
    ThreatPicture p = t.Evaluate(Origin(), {0, 1.7f, 0});
    REQUIRE(p.hasCover);
    CHECK(p.coverPoint.x == doctest::Approx(3.0f));
    CHECK(p.coverDistance == doctest::Approx(5.0f));
}

TEST_CASE("Threat: user cover query takes precedence") {
    ThreatModel t;
    t.SetCoverQuery([](const Vec3&, const Vec3&, Vec3& out) {
        out = {1, 0, 1};
        return true;
    });
    HitEvent h;
    h.direction = {0, 0, -1};
    t.RegisterHit(h);
    ThreatPicture p = t.Evaluate(Origin(), {0, 1.7f, 0});
    REQUIRE(p.hasCover);
    CHECK(p.coverPoint.x == doctest::Approx(1.0f));
}

TEST_CASE("Threat: latched near-miss side does not flip when the character moves afterwards") {
    ThreatModel t;
    NearMissEvent m;
    m.closestPoint = {0.2f, 1.7f, 0.3f};
    m.direction = {0, 0, -1};
    m.distance = 0.15f;
    m.nearestPart = BodyPart::Head;
    t.RegisterNearMiss(m, Origin(), {0, 1.7f, 0});
    // The character has since moved 0.5 m to the left: the stored point is now on its right.
    Transform moved{{0.5f, 1, 0}, Quat::Identity()};
    ThreatPicture p = t.Evaluate(moved, {0.5f, 1.7f, 0});
    CHECK(p.nearMissSide == Side::Left);
}
