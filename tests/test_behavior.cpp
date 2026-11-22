#include <doctest/doctest.h>

#include <string>

#include "reflex/behavior.h"

using namespace reflex;

namespace {

struct Scene {
    InjuryModel injury;
    ThreatModel threat;
    BodyStateSummary body;
    DecisionLayer layer;

    Scene() {
        body.position = {0, 0.95f, 0};
        body.com = {0, 0.95f, 0};
        body.heading = Quat::Identity();
    }

    const Decision& Think(float dt = 1.0f / 60.0f) {
        Transform frame{body.position, body.heading};
        ThreatPicture pic = threat.Evaluate(frame, body.position + Vec3{0, 0.75f, 0});
        BehaviorContext ctx{injury, pic, body, 0.0f};
        return layer.Decide(ctx, dt);
    }
};

bool Has(const Decision& d, const char* name) {
    for (const auto& a : d.active)
        if (std::string(a.name) == name) return true;
    return false;
}

bool Contains(const std::string& s, const char* sub) { return s.find(sub) != std::string::npos; }

void ShootFromFront(Scene& s, BodyPart part, float damage) {
    HitEvent h;
    h.bodyPart = part;
    h.direction = {0, 0, -1};
    h.damage = damage;
    s.threat.RegisterHit(h);
    s.injury.ApplyDamage(part, damage);
}

} // namespace

TEST_CASE("Behavior: a healthy, unthreatened character idles") {
    Scene s;
    const Decision& d = s.Think();
    CHECK(d.active.empty());
    CHECK(Contains(d.reason, "No threat"));
    CHECK_FALSE(d.targets.limping);
}

TEST_CASE("Behavior: leg hit -> limp away from fire, favour the healthy leg, crouch") {
    Scene s;
    ShootFromFront(s, BodyPart::RightLowerLeg, 35.0f);
    const Decision& d = s.Think();
    REQUIRE(Has(d, "LimpAway"));
    CHECK(d.targets.limping);
    CHECK(d.targets.limpSide == Side::Right);
    CHECK(d.targets.legLoadBias > 0.3f); // positive = more weight on the LEFT leg
    CHECK(d.targets.crouch > 0.2f);
    CHECK(d.targets.moveSpeed > 0.1f);
    CHECK(d.targets.moveSpeed < 0.8f); // slower than a healthy walk
    // Shot came from +Z (travelling -Z): moving away from fire means moving to -Z or sideways, never towards the shooter.
    CHECK(d.targets.moveDirection.z <= 0.01f);
    CHECK(Contains(d.reason, "Right leg support"));
    CHECK(Contains(d.reason, "shift weight to left"));
    CHECK(Contains(d.reason, "limp"));
    CHECK(Contains(d.reason, "move away from fire"));
    CHECK(Contains(d.reason, "crouch"));
}

TEST_CASE("Behavior: left leg hit mirrors the weight shift") {
    Scene s;
    ShootFromFront(s, BodyPart::LeftUpperLeg, 35.0f);
    const Decision& d = s.Think();
    REQUIRE(Has(d, "LimpAway"));
    CHECK(d.targets.limpSide == Side::Left);
    CHECK(d.targets.legLoadBias < -0.3f);
}

TEST_CASE("Behavior: leg hit with cover available limps towards cover") {
    Scene s;
    s.threat.AddCoverPoint({{4, 0, -1}, {0, 0, 1}});
    ShootFromFront(s, BodyPart::RightLowerLeg, 35.0f);
    const Decision& d = s.Think();
    REQUIRE(Has(d, "LimpAway"));
    CHECK(d.targets.moveDirection.x > 0.3f);
    CHECK(Contains(d.reason, "move to cover"));
}

TEST_CASE("Behavior: near miss on the left of the head -> turn and lean to the right, protect head") {
    Scene s;
    NearMissEvent m;
    m.closestPoint = {0.2f, 1.7f, 0.3f};
    m.direction = {0, 0, -1};
    m.distance = 0.15f;
    m.nearestPart = BodyPart::Head;
    s.threat.RegisterNearMiss(m);
    const Decision& d = s.Think();
    REQUIRE(Has(d, "TurnAndLeanAway"));
    REQUIRE(Has(d, "ProtectHead"));
    // Right of the character is -X in world space (it faces +Z).
    CHECK(d.targets.leanDirection.x < -0.5f);
    CHECK(d.targets.lean > 0.1f);
    CHECK(d.targets.turnYaw < -0.2f); // negative yaw = turn right
    CHECK(d.targets.jointOffset[BodyPart::Head].x > 0.1f); // chin tuck
    CHECK(Contains(d.reason, "turn away to the right"));
    CHECK(Contains(d.reason, "lean away from the left"));
    CHECK_FALSE(d.targets.limping);
}

TEST_CASE("Behavior: near miss on the right mirrors the reaction") {
    Scene s;
    NearMissEvent m;
    m.closestPoint = {-0.2f, 1.7f, 0.3f};
    m.direction = {0, 0, -1};
    m.distance = 0.15f;
    m.nearestPart = BodyPart::Head;
    s.threat.RegisterNearMiss(m);
    const Decision& d = s.Think();
    CHECK(d.targets.leanDirection.x > 0.5f);
    CHECK(d.targets.turnYaw > 0.2f);
}

TEST_CASE("Behavior: reaction to a near miss is expressed in the character's own frame") {
    Scene s;
    s.body.heading = Quat::FromAxisAngle({0, 1, 0}, kPi); // facing -Z: its left is now world -X
    NearMissEvent m;
    m.closestPoint = {-0.2f, 1.7f, -0.3f}; // on the character's left
    m.direction = {0, 0, 1};
    m.distance = 0.15f;
    m.nearestPart = BodyPart::Head;
    s.threat.RegisterNearMiss(m);
    const Decision& d = s.Think();
    CHECK(d.targets.leanDirection.x > 0.5f); // lean to the character's right = world +X
    CHECK(d.targets.turnYaw < -0.2f);
}

TEST_CASE("Behavior: torso hit -> hunch over the wound, protect with arms, slow down, brace") {
    Scene s;
    ShootFromFront(s, BodyPart::Chest, 40.0f);
    const Decision& d = s.Think();
    REQUIRE(Has(d, "ProtectWound"));
    REQUIRE(Has(d, "Brace"));
    CHECK(d.targets.jointOffset[BodyPart::Spine].x > 0.1f);
    CHECK(d.targets.jointOffset[BodyPart::LeftLowerArm].x < -0.5f);
    CHECK(d.targets.jointOffset[BodyPart::RightLowerArm].x < -0.5f);
    CHECK(d.targets.speedLimit < 0.9f);
    CHECK(Contains(d.reason, "hunch"));
    CHECK(Contains(d.reason, "brace"));
}

TEST_CASE("Behavior: arm hit -> that arm is cradled, the other balances, body leans away") {
    Scene s;
    ShootFromFront(s, BodyPart::RightLowerArm, 40.0f);
    const Decision& d = s.Think();
    REQUIRE(Has(d, "ProtectWound"));
    CHECK(d.targets.jointOffset[BodyPart::RightLowerArm].x < -1.0f);
    CHECK(d.targets.jointOffset[BodyPart::LeftUpperArm].z > 0.2f); // other arm held out
    CHECK(Contains(d.reason, "cradle the right arm"));
    CHECK(Has(d, "TurnAndLeanAway"));
    CHECK(d.targets.leanDirection.z < -0.5f); // away from the shooter (shooter at +Z)
}

TEST_CASE("Behavior: fatal damage collapses and overrides controlled behaviours") {
    Scene s;
    ShootFromFront(s, BodyPart::Chest, 400.0f);
    for (int i = 0; i < 60; ++i) s.injury.Update(1.0f / 60.0f);
    const Decision& d = s.Think();
    REQUIRE(Has(d, "Collapse"));
    CHECK_FALSE(Has(d, "LimpAway"));
    CHECK_FALSE(Has(d, "SeekCover"));
    CHECK(d.targets.collapse > 0.3f);
    CHECK(d.targets.assistScale < 0.8f);
    CHECK(Contains(d.reason, "Critical"));
}

TEST_CASE("Behavior: stumble recovery requests a step when the capture point leaves the support polygon") {
    Scene s;
    s.body.supportMargin = 0.2f;
    const Decision& d = s.Think();
    REQUIRE(Has(d, "StumbleRecover"));
    CHECK(d.targets.wantStep);
    CHECK(d.targets.jointOffset[BodyPart::LeftUpperArm].z > 0.2f); // arms out for balance
}

TEST_CASE("Behavior: break fall when tipping over while still conscious") {
    Scene s;
    s.body.tilt = 1.1f;
    s.body.tiltDirection = {0, 0, 1};
    const Decision& d = s.Think();
    REQUIRE(Has(d, "BreakFall"));
    CHECK(d.targets.breakFall);
    CHECK(d.targets.jointOffset[BodyPart::LeftUpperArm].x < -0.3f);
}

TEST_CASE("Behavior: behaviours can be disabled, removed and added") {
    Scene s;
    ShootFromFront(s, BodyPart::RightLowerLeg, 35.0f);
    CHECK(s.layer.SetEnabled("LimpAway", false));
    CHECK_FALSE(Has(s.Think(), "LimpAway"));
    CHECK(s.layer.SetEnabled("LimpAway", true));
    CHECK(Has(s.Think(), "LimpAway"));
    CHECK(s.layer.RemoveBehavior("LimpAway"));
    CHECK_FALSE(Has(s.Think(), "LimpAway"));
    CHECK_FALSE(s.layer.RemoveBehavior("DoesNotExist"));

    struct Always final : Behavior {
        const char* Name() const override { return "Always"; }
        Channel GetChannel() const override { return Channel::Posture; }
        float Score(const BehaviorContext&) const override { return 1.0f; }
        void Apply(const BehaviorContext&, float, ReactionTargets& o, std::string& c) const override {
            o.lean = 0.5f;
            c += "custom";
        }
    };
    s.layer.AddBehavior(std::make_unique<Always>());
    const Decision& d = s.Think();
    CHECK(Has(d, "Always"));
    CHECK(Contains(d.reason, "custom"));
}

TEST_CASE("Behavior: every decision has a non-empty reason") {
    Scene s;
    CHECK_FALSE(s.Think().reason.empty());
    ShootFromFront(s, BodyPart::LeftLowerLeg, 35.0f);
    CHECK_FALSE(s.Think().reason.empty());
}
