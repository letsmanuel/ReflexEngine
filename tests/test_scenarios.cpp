// Headless scenario tests: a full character in a physics world reacting to hits and near misses.
// The shooter always stands at +Z (the character faces +Z) and fires towards -Z, so "away from fire" is -Z or sideways.

#include <doctest/doctest.h>

#include <algorithm>
#include <cmath>
#include <string>

#include "scenario.h"

using namespace reflex;
using namespace reflex::test;

namespace {

struct Track {
    float minPelvisY = 10, maxTilt = 0;
    int fallenSteps = 0;
};

Track RunTracked(Scenario& s, float seconds) {
    Track t;
    int n = static_cast<int>(seconds / s.dt);
    for (int i = 0; i < n; ++i) {
        s.world->Step(s.dt);
        t.minPelvisY = std::min(t.minPelvisY, s.Pelvis().y);
        t.maxTilt = std::max(t.maxTilt, s.Tilt());
        if (s.Info().fallen) ++t.fallenSteps;
    }
    return t;
}

void Shoot(Scenario& s, BodyPart part, float damage, Vec3 dir = {0, 0, -1}) {
    HitEvent h;
    h.bodyPart = part;
    h.point = s.c.GetBodyPartTransform(part).position - dir * 0.1f;
    h.direction = dir;
    h.speed = 400.0f;
    h.mass = 0.008f;
    h.impulse = 3.2f;
    h.damage = damage;
    s.c.ApplyHit(h);
}

bool HasBehavior(const CharacterDebugInfo& d, const char* name) {
    for (const auto& a : d.activeBehaviors)
        if (std::string(a.name) == name) return true;
    return false;
}

bool Contains(const std::string& s, const char* sub) { return s.find(sub) != std::string::npos; }

} // namespace

TEST_CASE("Scenario: a standing character stays standing") {
    Scenario s;
    Track t = RunTracked(s, 4.0f);
    CHECK(t.minPelvisY > 0.90f);
    CHECK(t.maxTilt < 0.25f);
    CHECK(Length(Horizontal(s.Pelvis())) < 0.25f);
    CHECK(s.Info().footContact[0]);
    CHECK(s.Info().footContact[1]);
}

TEST_CASE("Scenario: the character holds upright against pushes") {
    Scenario s;
    s.Run(1.0f);
    for (int k = 0; k < 4; ++k) {
        // Pushes without damage from different directions.
        Vec3 dirs[4] = {{1, 0, 0}, {-1, 0, 0}, {0, 0, 1}, {0, 0, -1}};
        HitEvent h;
        h.bodyPart = BodyPart::Chest;
        h.point = s.c.GetBodyPartTransform(BodyPart::Chest).position;
        h.direction = dirs[k];
        h.impulse = 40.0f;
        h.damage = 0.0f;
        s.c.ApplyHit(h);
        Track t = RunTracked(s, 2.0f);
        CHECK(t.minPelvisY > 0.65f);
        CHECK(t.maxTilt < 0.8f);
    }
    s.Run(2.0f);
    CHECK(s.Upright());
    CHECK(s.Pelvis().y > 0.75f);
}

TEST_CASE("Scenario: shot in the right leg -> stays upright, shifts weight left, moves away from fire") {
    Scenario s;
    s.Run(1.0f);
    Vec3 start = s.Pelvis();
    Shoot(s, BodyPart::RightLowerLeg, 35.0f);

    float leftLoadSum = 0;
    int samples = 0;
    float minY = 10, maxTilt = 0;
    int n = static_cast<int>(2.0f / s.dt);
    for (int i = 0; i < n; ++i) {
        s.world->Step(s.dt);
        leftLoadSum += s.Info().legLoad[0];
        ++samples;
        minY = std::min(minY, s.Pelvis().y);
        maxTilt = std::max(maxTilt, s.Tilt());
    }
    CHECK(leftLoadSum / samples > 0.6f); // weight carried by the healthy left leg
    CHECK(s.Info().supportCapacity[1] < 0.6f);
    CHECK(s.Info().supportCapacity[0] > 0.95f);
    CHECK(HasBehavior(s.Info(), "LimpAway"));
    CHECK(Contains(s.Info().reason, "limp"));
    CHECK(Contains(s.Info().reason, "shift weight to left"));

    Track t = RunTracked(s, 6.0f);
    CHECK(std::min(minY, t.minPelvisY) > 0.6f);
    CHECK(std::max(maxTilt, t.maxTilt) < 0.9f);
    CHECK(t.fallenSteps == 0);
    CHECK(s.Upright());
    // It moved out of the line of fire (sideways and/or away), never towards the shooter.
    Vec3 moved = s.Pelvis() - start;
    CHECK(Length(Horizontal(moved)) > 0.25f);
    CHECK(moved.z < 0.15f);
    // It is crouching to stay covered.
    CHECK(s.Pelvis().y < 0.93f);
}

TEST_CASE("Scenario: shot in the left leg mirrors the reaction") {
    Scenario s;
    s.Run(1.0f);
    Shoot(s, BodyPart::LeftUpperLeg, 35.0f);
    float rightLoad = 0;
    int samples = 0;
    int n = static_cast<int>(2.0f / s.dt);
    for (int i = 0; i < n; ++i) {
        s.world->Step(s.dt);
        rightLoad += s.Info().legLoad[1];
        ++samples;
    }
    CHECK(rightLoad / samples > 0.6f);
    Track t = RunTracked(s, 5.0f);
    CHECK(t.fallenSteps == 0);
    CHECK(s.Upright());
}

TEST_CASE("Scenario: near miss on the left of the head -> turn and lean away to the right") {
    Scenario s;
    s.Run(1.0f);
    NearMissEvent m;
    m.closestPoint = s.Head() + Vec3{0.2f, 0, 0.3f};
    m.direction = {0, 0, -1};
    m.distance = 0.15f;
    m.nearestPart = BodyPart::Head;
    s.c.ReportNearMiss(m);

    float maxRightLean = 0, minForwardX = 1.0f;
    Track tr;
    int n = static_cast<int>(2.5f / s.dt);
    for (int i = 0; i < n; ++i) {
        s.world->Step(s.dt);
        Vec3 feet = (s.c.GetBodyPartTransform(BodyPart::LeftFoot).position +
                     s.c.GetBodyPartTransform(BodyPart::RightFoot).position) * 0.5f;
        maxRightLean = std::max(maxRightLean, -(s.Head().x - feet.x)); // positive = head moved to the character's right (-X)
        Vec3 fwd = s.c.GetBodyPartTransform(BodyPart::Chest).rotation.Rotate({0, 0, 1});
        minForwardX = std::min(minForwardX, fwd.x);
        tr.minPelvisY = std::min(tr.minPelvisY, s.Pelvis().y);
        tr.maxTilt = std::max(tr.maxTilt, s.Tilt());
    }
    CHECK(maxRightLean > 0.04f);   // leaned to its right
    CHECK(minForwardX < -0.05f);   // turned its chest towards the right
    CHECK(tr.minPelvisY > 0.65f);  // and stayed on its feet
    CHECK(tr.maxTilt < 0.9f);
    CHECK(HasBehavior(s.Info(), "TurnAndLeanAway"));
    CHECK(HasBehavior(s.Info(), "ProtectHead"));
    CHECK(Contains(s.Info().reason, "turn away to the right"));
}

TEST_CASE("Scenario: near miss on the right of the head mirrors the reaction") {
    Scenario s;
    s.Run(1.0f);
    NearMissEvent m;
    m.closestPoint = s.Head() + Vec3{-0.2f, 0, 0.3f};
    m.direction = {0, 0, -1};
    m.distance = 0.15f;
    m.nearestPart = BodyPart::Head;
    s.c.ReportNearMiss(m);
    float maxLeftLean = 0;
    int n = static_cast<int>(2.0f / s.dt);
    for (int i = 0; i < n; ++i) {
        s.world->Step(s.dt);
        Vec3 feet = (s.c.GetBodyPartTransform(BodyPart::LeftFoot).position +
                     s.c.GetBodyPartTransform(BodyPart::RightFoot).position) * 0.5f;
        maxLeftLean = std::max(maxLeftLean, s.Head().x - feet.x);
    }
    CHECK(maxLeftLean > 0.04f);
    CHECK(s.Upright());
}

TEST_CASE("Scenario: light torso hit -> hunch, protect the wound, stay upright, no impossible lean") {
    Scenario s;
    s.Run(1.0f);
    Shoot(s, BodyPart::Chest, 20.0f);
    float maxHunch = 0, maxOffset = 0, maxBend = 0;
    bool sawProtect = false, sawHunchReason = false;
    Track tr;
    int n = static_cast<int>(4.0f / s.dt);
    for (int i = 0; i < n; ++i) {
        s.world->Step(s.dt);
        sawProtect |= HasBehavior(s.Info(), "ProtectWound");
        sawHunchReason |= Contains(s.Info().reason, "hunch");
        Vec3 chest = s.c.GetBodyPartTransform(BodyPart::Chest).position;
        Vec3 pel = s.Pelvis();
        maxHunch = std::max(maxHunch, chest.z - pel.z);
        Vec3 feet = (s.c.GetBodyPartTransform(BodyPart::LeftFoot).position +
                     s.c.GetBodyPartTransform(BodyPart::RightFoot).position) * 0.5f;
        maxOffset = std::max(maxOffset, Length(Horizontal(s.Com() - feet)));
        Quat rel = s.c.GetBodyPartTransform(BodyPart::Pelvis).rotation.Conjugate() *
                   s.c.GetBodyPartTransform(BodyPart::Chest).rotation;
        maxBend = std::max(maxBend, Length(rel.ToRotationVector()));
        tr.minPelvisY = std::min(tr.minPelvisY, pel.y);
        tr.maxTilt = std::max(tr.maxTilt, s.Tilt());
    }
    CHECK(maxHunch > 0.03f);
    CHECK(maxBend < 0.62f);      // a human spine cannot fold further
    CHECK(maxOffset < 0.4f);     // the COM never hangs far beyond the feet
    CHECK(tr.minPelvisY > 0.65f);
    CHECK(sawProtect);
    CHECK(sawHunchReason);
    Track later = RunTracked(s, 3.0f);
    CHECK(later.fallenSteps == 0);
    CHECK(s.Upright());
}

TEST_CASE("Scenario: shot in the chest -> incapacitated: sinks to the ground, curls up, never stands again") {
    Scenario s;
    s.Run(1.0f);
    Shoot(s, BodyPart::Chest, 35.0f);
    s.Run(0.5f);
    CHECK(s.Pelvis().y > 0.6f); // it does not drop like a stone
    s.Run(5.0f);
    CHECK(s.Pelvis().y < 0.45f);
    CHECK(s.Info().control > 0.5f); // alive and conscious, just incapacitated
    CHECK(HasBehavior(s.Info(), "Incapacitated"));
    CHECK(Contains(s.Info().reason, "incapacitated"));
    // It keeps trying but never gets back up.
    float maxY = 0;
    int n = static_cast<int>(10.0f / s.dt);
    for (int i = 0; i < n; ++i) {
        s.world->Step(s.dt);
        maxY = std::max(maxY, s.Pelvis().y);
    }
    CHECK(maxY < 0.5f);
    CHECK(s.Info().fallen);
    // Hands stay near the body (covering head / chest), not flung out.
    Vec3 pel = s.Pelvis();
    CHECK(Length(s.c.GetBodyPartTransform(BodyPart::LeftHand).position - pel) < 1.4f);
    CHECK(Length(s.c.GetBodyPartTransform(BodyPart::RightHand).position - pel) < 1.4f);
}

TEST_CASE("Scenario: arm hit -> that arm is cradled, the other arm is used for balance, no fall") {
    Scenario s;
    s.Run(1.0f);
    Shoot(s, BodyPart::RightLowerArm, 40.0f);
    Track t = RunTracked(s, 4.0f);
    CHECK(t.fallenSteps == 0);
    CHECK(s.Upright());
    CHECK(Contains(s.Info().reason, "cradle the right arm"));
    CHECK(s.Info().armCapacity[1] < 0.5f);
    // The cradled hand is held up in front of the body; the other arm is out to the side.
    Transform rh = s.c.GetBodyPartTransform(BodyPart::RightHand);
    Transform lh = s.c.GetBodyPartTransform(BodyPart::LeftHand);
    Vec3 pel = s.Pelvis();
    CHECK(rh.position.y > lh.position.y - 0.05f);
    CHECK(std::abs(lh.position.x - pel.x) > 0.25f);
}

TEST_CASE("Scenario: several non-critical hits do not make the character fall over") {
    Scenario s;
    s.Run(1.0f);
    Shoot(s, BodyPart::LeftLowerArm, 20.0f);
    s.Run(1.0f);
    Shoot(s, BodyPart::RightUpperArm, 20.0f);
    s.Run(1.0f);
    Shoot(s, BodyPart::LeftLowerLeg, 25.0f);
    Track t = RunTracked(s, 6.0f);
    CHECK(t.fallenSteps == 0);
    CHECK(s.Upright());
    CHECK(s.Info().control > 0.5f);
}

TEST_CASE("Scenario: fatal damage leads to collapse") {
    Scenario s;
    s.Run(1.0f);
    Shoot(s, BodyPart::Chest, 400.0f);
    // Control fades gradually rather than instantly...
    s.Run(0.1f);
    CHECK(s.Info().control > 0.0f);
    CHECK(s.Info().control < 1.0f);
    s.Run(4.0f);
    CHECK(s.Info().control < 0.05f);
    CHECK(s.Info().fallen);
    CHECK(s.Pelvis().y < 0.35f);
    CHECK(HasBehavior(s.Info(), "Collapse"));
    CHECK(Contains(s.Info().reason, "Critical"));
}

TEST_CASE("Scenario: heavy but survivable damage degrades control and eventually collapses") {
    Scenario s;
    s.Run(1.0f);
    Shoot(s, BodyPart::Chest, 95.0f);
    s.Run(0.3f);
    float early = s.Info().control;
    s.Run(6.0f);
    CHECK(s.Info().control < early + 0.01f);
    CHECK(s.Info().control < 0.5f);
}

TEST_CASE("Scenario: zero global authority makes a passive ragdoll that falls") {
    Scenario s;
    s.Run(1.0f);
    s.c.SetGlobalAuthority(0.0f, 0.0f);
    s.Run(3.0f);
    CHECK(s.Pelvis().y < 0.35f);
}

TEST_CASE("Scenario: authority can be blended back up to recover") {
    Scenario s;
    s.Run(1.0f);
    s.c.SetGlobalAuthority(0.3f, 0.0f);
    s.Run(0.5f);
    s.c.SetGlobalAuthority(1.0f, 0.5f);
    Track t = RunTracked(s, 3.0f);
    CHECK(t.fallenSteps == 0);
    CHECK(s.Upright());
}

TEST_CASE("Scenario: authored animation pose drives the character and control can be handed back") {
    Scenario s;
    const CharacterDesc& d = s.c.Desc();
    s.Run(1.0f);
    // Animation: arms raised to the sides (T-pose): rotate upper arms about Z.
    Pose pose;
    for (const BoneDesc& b : d.skeleton.bones) pose.local.push_back(b.bindLocal);
    pose.local[static_cast<size_t>(d.mapping[BodyPart::LeftUpperArm])].rotation = Quat::FromAxisAngle({0, 0, 1}, 1.5f);
    pose.local[static_cast<size_t>(d.mapping[BodyPart::RightUpperArm])].rotation = Quat::FromAxisAngle({0, 0, 1}, -1.5f);
    s.c.SetAnimationPose(pose);
    s.c.SetAnimationWeight(1.0f, 0.2f);
    float handBefore = s.c.GetBodyPartTransform(BodyPart::LeftHand).position.y;
    s.Run(1.5f);
    Transform lh = s.c.GetBodyPartTransform(BodyPart::LeftHand);
    CHECK(lh.position.y > handBefore + 0.3f);              // the hand follows the animation upwards
    CHECK(lh.position.x > 0.5f);                           // out to the character's left
    CHECK(s.Upright());

    // Hand control back to the procedural reaction smoothly.
    s.c.TakeProceduralControl(0.5f);
    s.Run(2.0f);
    CHECK(s.c.GetAnimationWeight() == doctest::Approx(0.0f));
    CHECK(s.c.GetBodyPartTransform(BodyPart::LeftHand).position.y < 1.0f);
    CHECK(s.Upright());
}

TEST_CASE("Scenario: the output pose covers every bone and follows the physics bodies") {
    Scenario s;
    s.Run(1.0f);
    Pose out;
    s.c.GetOutputPose(out);
    REQUIRE(out.local.size() == s.c.Desc().skeleton.bones.size());
    std::vector<Transform> world;
    PoseToWorld(s.c.Desc().skeleton, out, world);
    for (int i = 0; i < kBodyPartCount; ++i) {
        BodyPart p = static_cast<BodyPart>(i);
        Vec3 bone = world[static_cast<size_t>(s.c.Desc().mapping[p])].position;
        Transform body = s.c.GetBodyPartTransform(p);
        CHECK(Length(bone - body.position) < 0.6f);
    }
    // The head bone is above the pelvis bone.
    CHECK(world[static_cast<size_t>(s.c.Desc().mapping[BodyPart::Head])].position.y >
          world[static_cast<size_t>(s.c.Desc().mapping[BodyPart::Pelvis])].position.y + 0.4f);
}

TEST_CASE("Scenario: bullets traced through the world hit body parts or report near misses") {
    Scenario s;
    s.Run(1.0f);
    // Aim at the right lower leg from the front.
    Vec3 target = s.c.GetBodyPartTransform(BodyPart::RightLowerLeg).position;
    BulletParams params;
    BulletResult r = s.world->TraceBullet(target + Vec3{0, 0, 8}, {0, 0, -1}, 20.0f, params);
    REQUIRE(r.type == BulletResult::Type::HitCharacter);
    CHECK((r.part == BodyPart::RightLowerLeg || r.part == BodyPart::RightUpperLeg));
    CHECK(s.c.Injury().SupportCapacity(Side::Right) < 0.7f);

    // A shot passing 0.2 m to the left of the head is a near miss.
    Scenario s2;
    s2.Run(1.0f);
    Vec3 head = s2.Head();
    BulletResult r2 = s2.world->TraceBullet(head + Vec3{0.30f, 0, 8}, {0, 0, -1}, 20.0f, params);
    CHECK(r2.type == BulletResult::Type::NearMiss);
    s2.Run(0.2f);
    CHECK(Contains(s2.Info().reason, "left"));
    CHECK(s2.c.Injury().ControlLevel() == doctest::Approx(1.0f));
}

TEST_CASE("Scenario: simulation is deterministic") {
    auto run = []() {
        Scenario s;
        s.Run(1.0f);
        Shoot(s, BodyPart::RightLowerLeg, 35.0f);
        s.Run(3.0f);
        return s.Pelvis();
    };
    Vec3 a = run(), b = run();
    CHECK(a.x == b.x);
    CHECK(a.y == b.y);
    CHECK(a.z == b.z);
}

TEST_CASE("Scenario: reset restores the character") {
    Scenario s;
    s.Run(1.0f);
    Shoot(s, BodyPart::Chest, 400.0f);
    s.Run(3.0f);
    CHECK(s.Pelvis().y < 0.35f);
    s.c.Reset();
    s.Run(2.0f);
    CHECK(s.Info().control == doctest::Approx(1.0f));
    CHECK(s.Pelvis().y > 0.9f);
    CHECK(s.Upright());
}

TEST_CASE("Scenario: two characters coexist and react independently") {
    Scenario s;
    CharacterDesc d2 = CharacterDesc::DefaultHumanoid();
    d2.spawn.position = {3, 0, 0};
    Character c2 = s.world->CreateCharacter(d2);
    s.Run(1.0f);
    Shoot(s, BodyPart::Chest, 400.0f);
    s.Run(4.0f);
    CHECK(s.Pelvis().y < 0.35f);
    CHECK(c2.GetBodyPartTransform(BodyPart::Pelvis).position.y > 0.85f);
    CHECK(c2.GetDebugInfo().control == doctest::Approx(1.0f));
}

TEST_CASE("Scenario: launched backwards by a huge force -> flies, lands, stays down, then gets up; nothing explodes") {
    Scenario s;
    s.Run(1.0f);
    Vec3 start = s.Pelvis();
    // 15 m/s velocity change towards -Z (backwards for a character facing +Z), delivered at the chest.
    s.c.ApplyImpact({0, 0, -15.0f * 75.0f}, s.c.GetBodyPartTransform(BodyPart::Chest).position);
    bool down = false, finite = true;
    float maxSpeed = 0, minY = 10;
    int n = static_cast<int>(5.0f / s.dt);
    for (int i = 0; i < n; ++i) {
        s.world->Step(s.dt);
        Vec3 p = s.Pelvis();
        finite &= std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
        down |= HasBehavior(s.Info(), "KnockedDown");
        maxSpeed = std::max(maxSpeed, Length(s.Info().comVelocity));
        minY = std::min(minY, p.y);
    }
    CHECK(finite);
    CHECK(maxSpeed > 8.0f);
    CHECK(maxSpeed < 40.0f);               // no runaway energy
    CHECK(s.Pelvis().z < start.z - 2.0f);  // thrown backwards
    CHECK(s.Info().control > 0.8f);        // conscious and uninjured: shock only
    CHECK(s.c.Injury().TorsoDamage() == doctest::Approx(0.0f));
    // It ends up on the ground...
    CHECK(minY < 0.45f);
    // ...is held down for a few seconds, protecting itself...
    CHECK(down);
    // ...and eventually gets back on its feet.
    s.Run(6.0f);
    CHECK(s.Pelvis().y > 0.7f);
    CHECK(s.Upright());
}

TEST_CASE("Scenario: an extreme launch stays stable") {
    Scenario s;
    s.Run(1.0f);
    s.c.ApplyImpact({0, 0.3f * 75.0f * 30.0f, -75.0f * 30.0f}, s.c.GetBodyPartTransform(BodyPart::Chest).position);
    bool finite = true;
    float maxSpeed = 0;
    int n = static_cast<int>(1.6f / s.dt); // 30 m/s: the 120 m test ground ends after ~2 s
    for (int i = 0; i < n; ++i) {
        s.world->Step(s.dt);
        Vec3 p = s.Pelvis();
        finite &= std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) && p.y > -0.5f;
        maxSpeed = std::max(maxSpeed, Length(s.Info().comVelocity));
    }
    CHECK(finite);
    CHECK(maxSpeed < 80.0f);
    CHECK(s.Pelvis().y > -0.2f); // never falls through the ground
}

TEST_CASE("Scenario: hands do not flop beyond their wrist limits") {
    Scenario s;
    s.c.SetGlobalAuthority(0.0f, 0.0f); // passive ragdoll: worst case for floppy hands
    float maxBend = 0;
    int n = static_cast<int>(3.0f / s.dt);
    for (int i = 0; i < n; ++i) {
        s.world->Step(s.dt);
        for (Side sd : {Side::Left, Side::Right}) {
            Quat rel = s.c.GetBodyPartTransform(LowerArm(sd)).rotation.Conjugate() *
                       s.c.GetBodyPartTransform(Hand(sd)).rotation;
            maxBend = std::max(maxBend, Length(rel.ToRotationVector()));
        }
    }
    CHECK(maxBend < 0.75f);
}
