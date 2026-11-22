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

TEST_CASE("Behavior: light torso hit -> hunch over the wound, protect with arms, slow down, brace") {
    Scene s;
    ShootFromFront(s, BodyPart::Chest, 22.0f);
    const Decision& d = s.Think();
    REQUIRE(Has(d, "ProtectWound"));
    REQUIRE(Has(d, "Brace"));
    CHECK(d.targets.jointOffset[BodyPart::Spine].x > 0.04f);
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

TEST_CASE("Behavior: past the lean point the character commits to the fall and reaches out") {
    Scene s;
    s.body.fallPhase = FallPhase::Tipping;
    s.body.tilt = 1.1f;
    s.body.supportMargin = 0.4f;
    s.body.tiltDirection = {0, 0, 1};
    const Decision& d = s.Think();
    REQUIRE(Has(d, "KnockedDown"));
    CHECK(d.targets.breakFall);
    CHECK(d.targets.assistScale < 0.1f); // no more balance help: it cannot recover by itself
    CHECK(d.targets.jointOffset[BodyPart::LeftUpperArm].x < -0.3f);
    CHECK(Contains(d.reason, "point of no return"));
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

TEST_CASE("Behavior: a bullet in the chest incapacitates: sink to the ground, never recover") {
    Scene s;
    ShootFromFront(s, BodyPart::Chest, 35.0f);
    s.injury.Update(0.016f);
    CHECK(s.injury.IsIncapacitated());
    CHECK_FALSE(s.injury.IsDead());
    const Decision& d = s.Think();
    REQUIRE(Has(d, "Incapacitated"));
    CHECK_FALSE(Has(d, "LimpAway"));
    CHECK_FALSE(Has(d, "StumbleRecover"));
    CHECK(Contains(d.reason, "sinking"));
    // The balance assist fades away completely.
    for (int i = 0; i < 60 * 3; ++i) s.injury.Update(1.0f / 60.0f);
    const Decision& later = s.Think();
    CHECK(later.targets.assistScale < 0.01f);
    CHECK(later.targets.groundPose > 0.9f);
}

TEST_CASE("Behavior: incapacitated on the back -> struggles; face down -> curls up and crawls") {
    Scene s;
    ShootFromFront(s, BodyPart::Chest, 35.0f);
    for (int i = 0; i < 60 * 3; ++i) s.injury.Update(1.0f / 60.0f);
    s.body.lying = true;
    s.body.supine = true;
    const Decision& back = s.Think();
    CHECK(back.targets.struggle);
    CHECK_FALSE(back.targets.crawl);
    CHECK(Contains(back.reason, "on the back"));
    CHECK(back.targets.authorityScale[BodyPart::Chest] < 0.3f); // far too weak to sit up
    s.body.supine = false;
    const Decision& front = s.Think();
    CHECK(front.targets.crawl);
    CHECK_FALSE(front.targets.struggle);
}

TEST_CASE("Behavior: down phase flails, get-up phase blends back to standing") {
    Scene s;
    s.body.lying = true;
    s.body.fallPhase = FallPhase::Down;
    s.body.phaseTime = 0.5f;
    s.body.phaseDuration = 1.5f;
    const Decision& down = s.Think();
    REQUIRE(Has(down, "KnockedDown"));
    CHECK(down.targets.struggle);
    CHECK(down.targets.assistScale < 0.01f); // no balance help: it does not pop back up
    CHECK(down.targets.groundPose > 0.9f);
    CHECK(Contains(down.reason, "flailing"));

    s.body.fallPhase = FallPhase::GetUp;
    s.body.phaseDuration = 2.4f;
    s.body.phaseTime = 0.1f;
    float earlyGround = 0, earlyAssist = 0;
    {
        const Decision& e = s.Think();
        earlyGround = e.targets.groundPose;
        earlyAssist = e.targets.assistScale;
    }
    s.body.phaseTime = 1.8f;
    const Decision& late = s.Think();
    CHECK(earlyGround > late.targets.groundPose);
    CHECK(earlyAssist < late.targets.assistScale);
    CHECK(late.targets.assistScale > 0.2f);
    CHECK(Contains(late.reason, "getting up"));
}
